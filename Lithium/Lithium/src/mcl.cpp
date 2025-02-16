#include "lemlib/api.hpp"
#include "externs.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

//TUNE THESE
const double SENSOR_NOISE = 10.0; // Adjust noise level as needed
const double THRESHOLD_DISTANCE = 2; //distance before MCL position is accepted
const int NUM_PARTICLES = 1000; //adjust number of particles
const double POSITION_VARIATION = 3; //maximum variation in position when resampling
const double ANGLE_VARIATION = 8; //maximum variation in theta when resampling
const float ACCEPTANCE_PERCENT = 0.125; //percent of highest weighted particles to accept for resampling

const double MM_TO_INCH = 0.0393701;
const double GRID_SIZE = 140.0;

struct Pose {
    double x, y, theta;
};

struct Sensor {
    double x_offset, y_offset, theta_offset;
};

struct Particle {
    Pose pose;
    double weight;
};

class MonteCarloLocalization {
private:
    std::vector<Particle> particles;
    std::default_random_engine generator;
    std::normal_distribution<double> noise_dist;
    Pose lastPose;

public:
    MonteCarloLocalization() : noise_dist(0.0, SENSOR_NOISE) {
        initializeParticles();
    }

    void initializeParticles() {
        std::uniform_real_distribution<double> dist(-GRID_SIZE / 2, GRID_SIZE / 2);
        std::uniform_real_distribution<double> angle_dist(0, 360);

        lastPose = {chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta};
        
        particles.clear();
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            particles.push_back({{dist(generator), dist(generator), angle_dist(generator)}, 1.0 / NUM_PARTICLES});
        }
    }

    double expectedSensorReading(const Pose& pose, const Sensor& sensor) {
        // Convert pose.theta from degrees to radians
        double theta_rad = -(pose.theta - 90) * M_PI / 180.0;

        // Transform sensor position based on robot's heading (adjusted for clockwise rotation)
        double rotated_x_offset = sqrt(pow(sensor.x_offset, 2) + pow(sensor.y_offset, 2)) * cos(theta_rad);
        double rotated_y_offset = sqrt(pow(sensor.x_offset, 2) + pow(sensor.y_offset, 2)) * sin(theta_rad);
    
        double sensor_x = pose.x + rotated_x_offset;
        double sensor_y = pose.y + rotated_y_offset;
    
        return computePQ(sensor_x*25.4, sensor_y*25.4, theta_rad + (sensor.theta_offset *(M_PI/180)), GRID_SIZE * 25.4);
        // Compute distance to nearest boundary of the grid
        // return std::min({fabs(sensor_x - (-GRID_SIZE / 2)), fabs(sensor_x - (GRID_SIZE / 2)),
        //                  fabs(sensor_y - (-GRID_SIZE / 2)), fabs(sensor_y - (GRID_SIZE / 2))});
    }
    
    //computes line PQ, which is distance from distance sensor to wall
    double computePQ(double p_x, double p_y, double theta, double L) {
        // Calculate the direction components of the ray (cos(theta), sin(theta))
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        
        // Initialize the intersection times with large values
        double t_x = std::numeric_limits<double>::infinity();
        double t_y = std::numeric_limits<double>::infinity();
        
        // Compute t_x (intersection with vertical sides of the square)
        if (cos_theta != 0) {
            if (cos_theta > 0) {
                t_x = (L / 2.0 - p_x) / cos_theta;  // Right side
                std::cout <<"right ";
            } else {
                t_x = (-L / 2.0 - p_x) / cos_theta; // Left side
                std::cout <<"left ";
            }
        }
        
        // Compute t_y (intersection with horizontal sides of the square)
        if (sin_theta != 0) {
            if (sin_theta > 0) {
                t_y = (L / 2.0 - p_y) / sin_theta;  // Top side
                std::cout <<"top ";
            } else {
                t_y = (-L / 2.0 - p_y) / sin_theta; // Bottom side
                std::cout <<"bottom ";
            }
        }
        
        // Return the smaller positive t value, which represents the first intersection
        std::cout << std::to_string(std::min(t_x, t_y)) + "\n";
        return std::min(t_x, t_y);
    }

    void updateWeights(double front, double left, double right, const Pose& robotPose, const Sensor sensors[3]) {
        double sum_weights = 0.0;
        for (auto& p : particles) {
            //update pose of all particles based on dead reckoning
            double deltaX = robotPose.x - lastPose.x;
            double deltaY = robotPose.y - lastPose.y;
            double deltaTheta = robotPose.theta - lastPose.theta;

            p.pose.x += deltaX;
            p.pose.y += deltaY;
            p.pose.theta += deltaTheta;

            double front_expected = expectedSensorReading(p.pose, sensors[0]) * MM_TO_INCH;
            double left_expected = expectedSensorReading(p.pose, sensors[1]) * MM_TO_INCH;
            double right_expected = expectedSensorReading(p.pose, sensors[2]) * MM_TO_INCH;
            //takes gaussian probability, might need to be changed
            double front_prob = exp(-pow(front - front_expected, 2) / (2 * SENSOR_NOISE * SENSOR_NOISE));
            double left_prob = exp(-pow(left - left_expected, 2) / (2 * SENSOR_NOISE * SENSOR_NOISE));
            double right_prob = exp(-pow(right - right_expected, 2) / (2 * SENSOR_NOISE * SENSOR_NOISE));
            
            p.weight = front_prob * left_prob * right_prob;
            sum_weights += p.weight;
            
            std::cout << "prob: " + std::to_string(front_prob)+ " " + std::to_string(left_prob) + " " + std::to_string(right_prob) + " " 
            + "expect: " + std::to_string(front_expected) + " " + std::to_string(left_expected) + " " + std::to_string(right_expected)
            + " read: " + std::to_string(front) + " " + std::to_string(left) + " " + std::to_string(right) + "\n";
        }

        if (sum_weights == 0) sum_weights = 1e-6;
        
        for (auto& p : particles) {
            p.weight /= sum_weights;
            std::cout << "weight" + std::to_string(p.weight) + "\n";
            std::cout << std::to_string(p.pose.x) + " " + std::to_string(p.pose.y) + " " + std::to_string(p.pose.theta) + "\n";
        }
    }

    void resampleParticles() {
        std::vector<Particle> new_particles;
        std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
            return a.weight > b.weight;
        });
        
        int top_count = NUM_PARTICLES * ACCEPTANCE_PERCENT;
        std::normal_distribution<double> pos_dist(-POSITION_VARIATION, POSITION_VARIATION);
        std::normal_distribution<double> angle_dist(-ANGLE_VARIATION, ANGLE_VARIATION);
        
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            const Particle& parent = particles[i % top_count];
            Particle new_particle;
            new_particle.pose.x = parent.pose.x + pos_dist(generator);
            new_particle.pose.y = parent.pose.y + pos_dist(generator);
            new_particle.pose.theta = parent.pose.theta + angle_dist(generator);
            new_particle.weight = 1.0 / NUM_PARTICLES;
            new_particles.push_back(new_particle);
        }
        
        particles = new_particles;
    }
    
    Pose getEstimatedPose() {
        double x_sum = 0.0, y_sum = 0.0, theta_sum = 0.0;
        for (const auto& p : particles) {
            x_sum += p.pose.x * p.weight;
            y_sum += p.pose.y * p.weight;
            theta_sum += p.pose.theta * p.weight;
            // pros::lcd::set_text(4, std::to_string(p.pose.x));
            // pros::lcd::set_text(5, std::to_string(p.pose.y));
            // pros::lcd::set_text(6, std::to_string(p.pose.theta));
        }
        // pros::lcd::set_text(4, std::to_string(x_sum));
        // pros::lcd::set_text(5, std::to_string(y_sum));
        return {x_sum, y_sum, theta_sum};
    }

    
};

//compares estimate 
    double compareEstimateToOdom(MonteCarloLocalization mcl){
        Pose estimatedPose = mcl.getEstimatedPose();
        lemlib::Pose chassisPose = chassis.getPose();

        double distance = sqrt(pow(estimatedPose.x - chassisPose.x, 2) + pow(estimatedPose.y - chassisPose.y, 2));
        return distance;
    }

//example function
int main() {
    MonteCarloLocalization mcl;
    
    Sensor sensors[3] = {
        {4.0, 0.0}, // Front sensor offset (in inches)
        {0.0, 4.0}, // Left sensor offset
        {0.0, -4.0} // Right sensor offset
    };

    double frontReading = 100;//frontSensor.get(); // Example sensor readings in mm
    double leftReading = 100;//leftSensor.get();
    double rightReading = 100;//rightSensor.get();
    
    Pose robotPose = {chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta}; // Assume we get this from chassis.getPose().theta
    
    mcl.updateWeights(frontReading, leftReading, rightReading, robotPose, sensors);
    mcl.resampleParticles();
    Pose estimatedPose = mcl.getEstimatedPose();
    
    chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
    
    return 0;
}

void testMCL(){
    chassis.setPose(-60, -60, 225);

    MonteCarloLocalization mcl;
    
    Sensor sensors[3] = {
        {4.5, 6.5, 0}, // Front sensor offset (in inches)
        {-5.9, 3.125, 90}, // Left sensor offset
        {5.9, 3.125, -90} // Right sensor offset
    };
    while(true){
        double frontReading = upSensor.get();
        double leftReading = leftSensor.get();
        double rightReading = rightSensor.get();

        pros::lcd::set_text(4, std::to_string(frontReading) + ", " + std::to_string(leftReading) + ", " + std::to_string(rightReading));
    
        Pose robotPose = {chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta}; // Assume we get this from chassis.getPose().theta
    
        mcl.updateWeights(frontReading * MM_TO_INCH, leftReading * MM_TO_INCH, rightReading * MM_TO_INCH, robotPose, sensors);
        mcl.resampleParticles();
        Pose estimatedPose = mcl.getEstimatedPose();
        pros::lcd::set_text(5, std::to_string(estimatedPose.x));
        pros::lcd::set_text(6, std::to_string(estimatedPose.y));
        pros::lcd::set_text(7, std::to_string(estimatedPose.theta));

        if(compareEstimateToOdom(mcl) < THRESHOLD_DISTANCE){
            chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
        }
        pros::delay(500);
    }
}