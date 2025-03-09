#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

/*
* @brief Class defining a particle
*/
class Particle {
public:
    double x;          // x position in inches
    double y;          // y position in inches
    double weight;     // particle weight

    Particle() : x(0), y(0), weight(1.0) {}
    
    Particle(double x, double y) : x(x), y(y), weight(1.0) {}
};

/*
* @brief Class for modelling distance sensor
*/
class DistanceSensorModel {
public:
    int port;          // PROS port number
    double xOffset;    // x offset from robot center (inches), positive is right
    double yOffset;    // y offset from robot center (inches), positive is front
    double thetaOffset; // angular offset from robot heading (degrees), positive is CCW
    pros::Distance sensor;

    DistanceSensorModel(int port, double xOffset, double yOffset, double thetaOffset) :
        port(port), xOffset(xOffset), yOffset(yOffset), thetaOffset(thetaOffset), sensor(port) {}

    /*
    * @brief Gets the expected distance reading for a particle at a given position and robot heading
    */
    double getExpectedReading(const Particle& particle, double robotTheta) const {
        // Calculate global position of the sensor
        // Convert robot theta to radians
        double robotThetaRad = robotTheta * M_PI / 180.0;
        
        // Calculate sensor global theta (robot theta + sensor offset)
        double sensorGlobalTheta = robotTheta + thetaOffset;
        double sensorThetaRad = sensorGlobalTheta * M_PI / 180.0;
        
        // Calculate sensor global position using robot-relative coordinate system
        // For robot-relative coordinates: 
        // - positive x is right
        // - positive y is front
        // To convert to global coordinates, we rotate these offsets by the robot's heading
        double sensorGlobalX = particle.x + (xOffset * cos(robotThetaRad) - yOffset * sin(robotThetaRad));
        double sensorGlobalY = particle.y + (xOffset * sin(robotThetaRad) + yOffset * cos(robotThetaRad));

        // Calculate expected distance to nearest wall
        double distToWall = calculateDistanceToWall(sensorGlobalX, sensorGlobalY, sensorGlobalTheta);
        
        // Convert from inches to mm for comparison with real sensor
        return distToWall * 25.4;
    }
    
    /*
    * @brief Calculate distance to nearest wall from a given position and orientation
    */
    double calculateDistanceToWall(double x, double y, double theta) const {
        const double FIELD_SIZE = 70.0; // half-width of field in inches
        const double MAX_RANGE = 2000.0 / 25.4; // max range in inches (converted from mm)
        
        double thetaRad = theta * M_PI / 180.0;
        double cosTheta = cos(thetaRad);
        double sinTheta = sin(thetaRad);
        
        double dist = MAX_RANGE;
        
        // Check distance to each wall
        // Right wall (x = FIELD_SIZE)
        if (cosTheta > 0.0001) {
            double d = (FIELD_SIZE - x) / cosTheta;
            if (d > 0 && d < dist) {
                double intersectY = y + d * sinTheta;
                if (intersectY >= -FIELD_SIZE && intersectY <= FIELD_SIZE) {
                    dist = d;
                }
            }
        }
        
        // Left wall (x = -FIELD_SIZE)
        if (cosTheta < -0.0001) {
            double d = (-FIELD_SIZE - x) / cosTheta;
            if (d > 0 && d < dist) {
                double intersectY = y + d * sinTheta;
                if (intersectY >= -FIELD_SIZE && intersectY <= FIELD_SIZE) {
                    dist = d;
                }
            }
        }
        
        // Top wall (y = FIELD_SIZE)
        if (sinTheta > 0.0001) {
            double d = (FIELD_SIZE - y) / sinTheta;
            if (d > 0 && d < dist) {
                double intersectX = x + d * cosTheta;
                if (intersectX >= -FIELD_SIZE && intersectX <= FIELD_SIZE) {
                    dist = d;
                }
            }
        }
        
        // Bottom wall (y = -FIELD_SIZE)
        if (sinTheta < -0.0001) {
            double d = (-FIELD_SIZE - y) / sinTheta;
            if (d > 0 && d < dist) {
                double intersectX = x + d * cosTheta;
                if (intersectX >= -FIELD_SIZE && intersectX <= FIELD_SIZE) {
                    dist = d;
                }
            }
        }
        
        return dist;
    }
    
    /*
    * @brief Get actual distance reading from the sensor (in mm)
    */
    double getActualReading() {
        return sensor.get();
    }
    
    /*
    * @brief Calculate likelihood of getting actual reading given expected reading
    */ 
    double calculateLikelihood(double expected, double actual) const {
        // Convert to mm for calculation
        const double MAX_RANGE_MM = 2000.0;
        
        // Handle max range readings
        if (actual >= MAX_RANGE_MM) {
            return (expected >= MAX_RANGE_MM) ? 1.0 : 0.1;
        }
        
        // Calculate error
        double error = fabs(expected - actual);
        
        // Apply sensor error model
        double expectedError;
        if (actual < 200.0) {
            expectedError = 15.0; // 15mm error for distances under 200mm
        } else {
            expectedError = 0.05 * actual; // 5% error for distances above 200mm
        }
        
        // Gaussian likelihood
        double variance = expectedError * expectedError;
        return exp(-(error * error) / (2 * variance));
    }
};

/*
* @brief Class for MCL
*/
class MonteCarloLocalization {
private:
    std::vector<Particle> particles;
    std::vector<DistanceSensorModel> sensors;
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<> noise_pos;
    pros::Imu imu;
    lemlib::Chassis& chassis;
    const int NUM_PARTICLES = 500;
    const double FIELD_SIZE = 70.0; // inches
    
    // Most recent odometry values
    double lastX = chassis.getPose().x;
    double lastY = chassis.getPose().y;

public:
    MonteCarloLocalization(lemlib::Chassis& chassis, int IMU_PORT) : 
        gen(rd()),
        noise_pos(0.0, 1.0),
        imu(IMU_PORT), // assuming port 21 for the IMU
        chassis(chassis) {
        
        // Initialize particles with uniform distribution
        initializeParticles();
    }
    
    /*
    * @brief Adds a distance sensor input to the MonteCarloLocalization. Offsets should be measured from the turning center of the robot.
    * @param port Port number of the distance sensor
    * @param xOffset Offset relative to the robot in the right-left direction, with right being positive.
    * @param yOffset Offset relative to the robot in the front-back direction, with front being positive.
    * @param thetaOffset Angular offset relative to the front of the robot, with positive being a counterclockwise rotation from forwards
    */
    void addDistanceSensor(int port, double xOffset, double yOffset, double thetaOffset) {
        sensors.emplace_back(port, xOffset, yOffset, thetaOffset);
    }
    
    /*
    * @brief Initialize particles with uniform distribution over the field
    */ 
    void initializeParticles() {
        particles.clear();
        
        std::uniform_real_distribution<> dist_pos(-FIELD_SIZE, FIELD_SIZE);
        
        for (int i = 0; i < NUM_PARTICLES; i++) {
            Particle p;
            p.x = dist_pos(gen);
            p.y = dist_pos(gen);
            p.weight = 1.0 / NUM_PARTICLES;
            particles.push_back(p);
        }
    }
    
    /*
    * @brief Convert IMU heading to standard position angle
    */ 
    double getStandardAngle() {
        double imuHeading = imu.get_heading();
        // Apply the conversion formula: y = 90 - x
        double standardAngle = 90.0 - imuHeading;
        
        // Normalize angle to [0, 360)
        while (standardAngle < 0) standardAngle += 360.0;
        while (standardAngle >= 360.0) standardAngle -= 360.0;
        
        return standardAngle;
    }
    
    /*
    * @brief Update particle positions based on odometry change
    */ 
    void motionUpdate() {
        // Get current pose
        lemlib::Pose currentPose = chassis.getPose();
        
        // Calculate odometry change
        double deltaX = currentPose.x - lastX;
        double deltaY = currentPose.y - lastY;
        
        // Add some noise to the motion model based on movement amount
        double noiseScale = sqrt(deltaX * deltaX + deltaY * deltaY) * 0.1; // 10% of distance
        
        // Update each particle
        for (auto& p : particles) {
            // Apply motion update with added noise
            p.x += deltaX + noise_pos(gen) * noiseScale;
            p.y += deltaY + noise_pos(gen) * noiseScale;
            
            // Enforce field boundaries
            p.x = std::max(-FIELD_SIZE, std::min(FIELD_SIZE, p.x));
            p.y = std::max(-FIELD_SIZE, std::min(FIELD_SIZE, p.y));
        }
        
        // Update last pose
        lastX = currentPose.x;
        lastY = currentPose.y;
    }
    
    /*
    * @brief Update particle weights based on sensor measurements
    */ 
    void sensorUpdate() {
        if (sensors.empty()) return;
        
        double totalWeight = 0.0;
        double robotTheta = getStandardAngle();
        
        // Update weight for each particle
        for (auto& p : particles) {
            double particleWeight = 1.0;
            
            // Calculate likelihood for each sensor
            for (auto& sensor : sensors) {
                double expected = sensor.getExpectedReading(p, robotTheta);
                double actual = sensor.getActualReading();
                double likelihood = sensor.calculateLikelihood(expected, actual);
                
                // Multiply particle weight by likelihood
                particleWeight *= likelihood;
            }
            
            p.weight = particleWeight;
            totalWeight += particleWeight;
        }
        
        // Normalize weights
        if (totalWeight > 0) {
            for (auto& p : particles) {
                p.weight /= totalWeight;
            }
        } else {
            // If all weights are zero, reinitialize
            initializeParticles();
        }
    }
    
    /*
    * @brief Resample particles based on weights
    */
    void resample() {
        std::vector<Particle> newParticles;
        
        // // Create distribution based on weights
        // std::discrete_distribution<> dist(particles.begin(), particles.end(), 
        //     [](const Particle& p) { return p.weight; });

        // Create a vector of weights
        std::vector<double> weights;
        weights.reserve(particles.size());
        for (const auto& p : particles) {
            weights.push_back(p.weight);
        }

        // Create distribution based on weights
        std::discrete_distribution<> dist(weights.begin(), weights.end());
        
        // Resample with replacement
        for (int i = 0; i < NUM_PARTICLES; i++) {
            Particle newP = particles[dist(gen)];
            
            // Add some noise to avoid particle depletion
            newP.x += noise_pos(gen) * 0.5; // 0.5 inch noise
            newP.y += noise_pos(gen) * 0.5;
            
            // Enforce field boundaries
            newP.x = std::max(-FIELD_SIZE, std::min(FIELD_SIZE, newP.x));
            newP.y = std::max(-FIELD_SIZE, std::min(FIELD_SIZE, newP.y));
            
            newP.weight = 1.0 / NUM_PARTICLES;
            newParticles.push_back(newP);
        }
        
        particles = newParticles;
    }
    
    /*
    * @brief Calculate the best estimate of the robot's position
    */ 
    lemlib::Pose getBestPose() {
        double sumX = 0, sumY = 0;
        double weightSum = 0;
        
        for (const auto& p : particles) {
            sumX += p.x * p.weight;
            sumY += p.y * p.weight;
            weightSum += p.weight;
        }
        
        double x = sumX / weightSum;
        double y = sumY / weightSum;
        
        // Use IMU for heading (converted to standard position)
        double theta = imu.get_heading();
        
        return lemlib::Pose{x, y, theta};
    }
    
    /*
    * @brief Check if particles are converged (localization is confident). Might not be necessary due to resource intensity
    */ 
    bool isConverged() {
        // Calculate variance of particle positions
        double meanX = 0, meanY = 0;
        double varX = 0, varY = 0;
        
        for (const auto& p : particles) {
            meanX += p.x / NUM_PARTICLES;
            meanY += p.y / NUM_PARTICLES;
        }
        
        for (const auto& p : particles) {
            varX += (p.x - meanX) * (p.x - meanX) / NUM_PARTICLES;
            varY += (p.y - meanY) * (p.y - meanY) / NUM_PARTICLES;
        }
        
        // Consider converged if standard deviation is less than 5 inches
        return sqrt(varX) < 5.0 && sqrt(varY) < 5.0;
    }
    
    /*
    * @brief Main update loop
    */
    void update() {
        motionUpdate();
        sensorUpdate();
        resample();
        
        // If particles have converged, update the robot's pose
        if (isConverged()) {
            lemlib::Pose bestPose = getBestPose();
            chassis.setPose(bestPose.x, bestPose.y, bestPose.theta);
        }
    }
    
    // get particles (for debug)
    const std::vector<Particle>& getParticles() const {
        return particles;
    }
};