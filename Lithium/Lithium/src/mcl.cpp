#include "lemlib/api.hpp"
#include "externs.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

const double MM_TO_INCH = 0.0393701;
const int NUM_PARTICLES = 1000;
const double SENSOR_NOISE = 30.0; // Adjust noise level as needed
const double THRESHOLD_DISTANCE = 2; //distance before MCL position is accepted
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

public:
    MonteCarloLocalization() : noise_dist(0.0, SENSOR_NOISE) {
        initializeParticles();
    }

    void initializeParticles() {
        std::uniform_real_distribution<double> dist(-GRID_SIZE / 2, GRID_SIZE / 2);
        std::uniform_real_distribution<double> angle_dist(0, 2 * M_PI);
        
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
    
        return computePQ(sensor_x, sensor_y, theta_rad + sensor.theta_offset, GRID_SIZE);
        // Compute distance to nearest boundary of the grid
        // return std::min({fabs(sensor_x - (-GRID_SIZE / 2)), fabs(sensor_x - (GRID_SIZE / 2)),
        //                  fabs(sensor_y - (-GRID_SIZE / 2)), fabs(sensor_y - (GRID_SIZE / 2))});
    }
    
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
            } else {
                t_x = (-L / 2.0 - p_x) / cos_theta; // Left side
            }
        }
        
        // Compute t_y (intersection with horizontal sides of the square)
        if (sin_theta != 0) {
            if (sin_theta > 0) {
                t_y = (L / 2.0 - p_y) / sin_theta;  // Top side
            } else {
                t_y = (-L / 2.0 - p_y) / sin_theta; // Bottom side
            }
        }
        
        // Return the smaller positive t value, which represents the first intersection
        std::cout << std::to_string(std::min(t_x, t_y)) + "\n";
        return std::min(t_x, t_y);
    }

    void updateWeights(double front, double left, double right, const Pose& robotPose, const Sensor sensors[3]) {
        double sum_weights = 0.0;
        for (auto& p : particles) {
            double front_expected = expectedSensorReading(p.pose, sensors[0]) * MM_TO_INCH;
            double left_expected = expectedSensorReading(p.pose, sensors[1]) * MM_TO_INCH;
            double right_expected = expectedSensorReading(p.pose, sensors[2]) * MM_TO_INCH;

            double front_prob = exp(-pow(front - front_expected, 2) / (2 * SENSOR_NOISE * SENSOR_NOISE));
            double left_prob = exp(-pow(left - left_expected, 2) / (2 * SENSOR_NOISE * SENSOR_NOISE));
            double right_prob = exp(-pow(right - right_expected, 2) / (2 * SENSOR_NOISE * SENSOR_NOISE));
            
            p.weight = front_prob * left_prob * right_prob;
            sum_weights += p.weight;
            std::cout << std::to_string(p.weight) + " "+std::to_string(front_prob)+ " " + std::to_string(left_prob) + " " + std::to_string(right_prob) + " " 
            + std::to_string(front_expected) + " " + std::to_string(left_expected) + " " + std::to_string(right_expected) + "\n";
        }

        if (sum_weights == 0) sum_weights = 1e-6;
        
        for (auto& p : particles) {
            p.weight /= sum_weights;
        }
    }

    void resampleParticles() {
        std::vector<Particle> new_particles;
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        double beta = 0.0;
        double max_weight = 0.0;
        for (const auto& p : particles) {
            if (p.weight > max_weight) max_weight = p.weight;
        }
        int index = rand() % NUM_PARTICLES;
        for (int i = 0; i < NUM_PARTICLES; ++i) {
            beta += dist(generator) * 2.0 * max_weight;
            while (beta > particles[index].weight) {
                beta -= particles[index].weight;
                index = (index + 1) % NUM_PARTICLES;
            }
            new_particles.push_back(particles[index]);
        }
        particles = new_particles;
    }
    
    Pose getEstimatedPose() {
        double x_sum = 0.0, y_sum = 0.0;
        for (const auto& p : particles) {
            x_sum += p.pose.x * p.weight;
            y_sum += p.pose.y * p.weight;
            // pros::lcd::set_text(4, std::to_string(p.pose.x));
            // pros::lcd::set_text(5, std::to_string(p.pose.y));
            // pros::lcd::set_text(6, std::to_string(p.pose.theta));
        }
        // pros::lcd::set_text(4, std::to_string(x_sum));
        // pros::lcd::set_text(5, std::to_string(y_sum));
        return {x_sum, y_sum, chassis.getPose().theta};
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
    chassis.setPose(-60, 0, 90);

    MonteCarloLocalization mcl;
    
    Sensor sensors[3] = {
        {4.5, 6.5, 0}, // Front sensor offset (in inches)
        {-5.9, 3.125, -90}, // Left sensor offset
        {5.9, 3.125, 90} // Right sensor offset
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

        if(compareEstimateToOdom(mcl) < THRESHOLD_DISTANCE){
            chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
        }
        pros::delay(50);
    }
}