#include "lemlib/api.hpp"
#include "externs.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <cmath>

const double MM_TO_INCH = 0.0393701;
const int NUM_PARTICLES = 1000;
const double SENSOR_NOISE = 1.0; // Adjust noise level as needed
const double GRID_SIZE = 140.0;

struct Pose {
    double x, y, theta;
};

struct Sensor {
    double x_offset, y_offset;
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
        double sensor_x = pose.x + sensor.x_offset * cos(pose.theta) - sensor.y_offset * sin(pose.theta);
        double sensor_y = pose.y + sensor.x_offset * sin(pose.theta) + sensor.y_offset * cos(pose.theta);
        return std::min({fabs(sensor_x - (-GRID_SIZE / 2)), fabs(sensor_x - (GRID_SIZE / 2)),
                         fabs(sensor_y - (-GRID_SIZE / 2)), fabs(sensor_y - (GRID_SIZE / 2))});
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
        }
        
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
        {6.5, 4.5}, // Front sensor offset (in inches)
        {-5.9, 3.125}, // Left sensor offset
        {5.9, 3.125} // Right sensor offset
    };
    while(true){
        double frontReading = upSensor.get();
        if(!(frontReading > 0 || frontReading < 1750)){
            frontReading = 1750;
        } // Example sensor readings in mm
        double leftReading = leftSensor.get();
        if(!(leftReading > 0 || leftReading < 1750)){
            leftReading = 1750;
        } 
        double rightReading = rightSensor.get();
        if(!(rightReading > 0 || rightReading < 1750)){
            rightReading = 1750;
        } 

        pros::lcd::set_text(4, std::to_string(frontReading));
            pros::lcd::set_text(5, std::to_string(leftReading));
            pros::lcd::set_text(6, std::to_string(rightReading));
    
        Pose robotPose = {chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta}; // Assume we get this from chassis.getPose().theta
    
        mcl.updateWeights(frontReading, leftReading, rightReading, robotPose, sensors);
        mcl.resampleParticles();
        Pose estimatedPose = mcl.getEstimatedPose();
    
        chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
        pros::delay(100);
    }
}