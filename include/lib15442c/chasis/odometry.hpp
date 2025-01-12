#pragma once

#include <memory>

#include "../math/vector.hpp"
#include "../math/pose.hpp"
#include "../math/angle.hpp"

#ifndef LIB15442C_MOCK_DEVICES_ONLY
#include "pros/rtos.hpp"
#include "pros/rotation.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"
#include <random>
#endif

namespace lib15442c
{
    /**
     * An abstract position tracking system
     */
    class IOdometry
    {
    public:
        /**
         * @brief Initialize the odometry algorithm. If the algorithm runs in a seperate task, it starts the task
         * 
         * @param x The initial X position
         * @param y The inital Y position
         * @param theta The initial heading
         */
        virtual void initialize(double x, double y, Angle theta) = 0;

        /**
         * @brief Set whether the position should be mirrored over the y-axis
         * 
         * @param mirrored 
         */
        virtual void set_mirrored(bool mirrored) = 0;
        /**
         * @brief Get whether the position is mirrored over the y-axis
         * 
         * @return bool
         */
        virtual bool get_mirrored() = 0;

        /**
         * @brief Get the x position of the robot
         */
        virtual double get_x() = 0;
        /**
         * @brief Get the y position of the robot
         */
        virtual double get_y() = 0;
        /**
         * @brief Get the position of the robot
         */
        virtual lib15442c::Vec get_position() = 0;
        /**
         * @brief Get the pose of the robot
         */
        virtual lib15442c::Pose get_pose() = 0;

        /**
         * @brief Get the rotation of the robot
         */
        virtual lib15442c::Angle get_rotation(bool ignore_offset = false) = 0;
        /**
         * @brief Set the rotation of the robot
         *
         * @param rotation The new rotation of the robot
         */
        virtual void set_rotation(lib15442c::Angle rotation) = 0;
    };
    
    #ifndef LIB15442C_MOCK_DEVICES_ONLY

    /**
     * Settings for an inertial sensor
     */
    struct TrackerIMU
    {
        /**
         * The inertial sensor
         */
        std::shared_ptr<pros::IMU> imu;
        /**
         * How much to scale the output of the inertial by
         */
        double scale;
    };
    /**
     * Stettings for a tracking wheel
     */
    struct TrackerWheel
    {
        /**
         * The rotation sensor of the wheel
         */
        std::shared_ptr<pros::Rotation> tracker;
        /**
         * How far away the wheel is from the center of rotation of the robot
         */
        double offset;
        /**
         * The diameter of the wheel
         */
        double diameter;
    };

    /**
     * Position tracking using two tracking wheels
     */
    class TrackerOdom : public virtual IOdometry
    {
    private:
        // Sensors
        std::shared_ptr<pros::Rotation> parallel_tracker;
        std::shared_ptr<pros::Rotation> perpendicular_tracker;
        std::shared_ptr<pros::v5::IMU> inertial;

        // Settings
        double inertial_scale;
        double parallel_tracker_offset;
        double parallel_tracker_circumfrance;
        double perpendicular_tracker_offset;
        double perpendicular_tracker_circumfrance;
        bool mirrored;

        bool updated_offsets = false;

        // Inertial 2
        std::shared_ptr<pros::v5::IMU> inertial_2;
        double inertial_scale_2;

        // Rotation Offset
        Angle rotation_offset = 0_deg;

        // Robot position
        pros::rtos::Mutex position_mutex = pros::rtos::Mutex();
        Vec position = Vec(0, 0);

        // Tasks
        pros::Task task = pros::Task([]
                                     { return; });

    public:
        TrackerOdom(
            TrackerWheel parallel_tracker,
            TrackerWheel perpendicular_tracker,
            bool mirrored,
            TrackerIMU inertial,
            TrackerIMU inertial_2 = { .imu = std::make_shared<pros::IMU>(22), .scale = 0});

        ~TrackerOdom();
        
        void initialize(double initial_x, double initial_y, Angle initial_theta);

        void set_mirrored(bool mirrored);
        bool get_mirrored();

        void set_parallel_offset(double offset);
        void set_perpendicular_offset(double offset);

        double get_x();
        double get_y();
        Vec get_position();
        Pose get_pose();

        /**
         * @brief Set the x position of the robot
         *
         * @param x The new x position
         */
        void set_x(double x);
        /**
         * @brief Set the y position of the robot
         *
         * @param y The new y position
         */
        void set_y(double y);
        /**
         * @brief Set the position of the robot
         *
         * @param position The new position
         */
        void set_position(lib15442c::Vec position);

        Angle get_rotation(bool ignore_offset = false);
        void set_rotation(Angle rotation);

        /**
         * @brief Start the odometry background task
         */
        void start_task();
        /**
         * @brief Stop the odometry background task
         */
        void stop_task();
    };

    /**
     * Position tracking using the Vex GPS sensor
     */
    class GPSOdom : public virtual IOdometry
    {
    private:
        pros::GPS gps;

        double rotation_offset;
        bool mirrored;

        static constexpr double inches_per_meter = 39.3701;

    public:
        GPSOdom(int port, double x_offset, double y_offset, double rotation_offset = 0, bool mirrored = false);
        
        void initialize(double initial_x, double initial_y, Angle initial_theta);

        void set_mirrored(bool mirrored);
        bool get_mirrored();

        double get_x();
        double get_y();
        Vec get_position();
        Pose get_pose();

        void set_x(double x);
        void set_y(double y);
        void set_position(Vec position);

        Angle get_rotation(bool ignore_offset = false);
        void set_rotation(Angle rotation);
    };

    struct MCLSensorParams
    {
        int port;
        double x_offset;
        double y_offset;
        double theta_offset;
    };

    struct MCLConfig
    {
        // The number of particles to use for the particle filter
        int particle_count;
        // What percent of the particles should be uniformly random; the rest are based on the previous probability distribution
        double uniform_random_percent;
        // The standard deviation of the tracking wheel measurements
        double tracker_odom_sd;
    };

    class MCLOdom : public virtual IOdometry
    {
    private:
        struct MCLSensor
        {
            std::shared_ptr<pros::Distance> sensor;
            double x_offset;
            double y_offset;
            double theta_offset;  
        };

        struct MCLParticle
        {
            double x;
            double y;
            double weight;
        };

        std::shared_ptr<TrackerOdom> tracker_odom;
        double last_x;
        double last_y;

        std::vector<MCLSensor> sensors;

        std::vector<MCLParticle> particles;

        pros::Mutex position_mutex;
        double predicted_x = 0;
        double predicted_y = 0;

        std::mt19937 rng;

        int particle_count;
        double uniform_random_percent;
        double tracker_odom_sd;
        
        bool mirrored = false;

        pros::Task task = pros::Task([] { return; });

        static double sensor_sd(double distance);
        static double get_particle_chance(double x, double y, MCLSensor sensor, double theta);

        static double gaussian_distribution(double x, double mean, double sd);

        double random();
        double gaussian_random(double mean, double sd);

        void motion_update();
        void resample();
        void sensor_update();

    public:
        MCLOdom(MCLConfig config, std::shared_ptr<TrackerOdom> tracker_odom, std::vector<MCLSensorParams>);
        ~MCLOdom();
        
        void initialize(double initial_x, double initial_y, Angle initial_theta);

        void set_mirrored(bool mirrored);
        bool get_mirrored();

        double get_x();
        double get_y();
        Vec get_position();
        Pose get_pose();

        Angle get_rotation(bool ignore_offset = false);
        void set_rotation(Angle rotation);

        /**
         * @brief Start the odometry background task
         */
        void start_task(double initial_x, double initial_y, Angle initial_theta);
        /**
         * @brief Stop the odometry background task
         */
        void stop_task();
    };

    #endif
}