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
#endif

namespace lib15442c
{
    class IOdometry
    {
    public:
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
         * @brief Set the x position of the robot
         *
         * @param x The new x position
         */
        virtual void set_x(double x) = 0;
        /**
         * @brief Set the y position of the robot
         *
         * @param y The new y position
         */
        virtual void set_y(double y) = 0;
        /**
         * @brief Set the position of the robot
         *
         * @param position The new position
         */
        virtual void set_position(lib15442c::Vec position) = 0;

        /**
         * @brief Get the rotation of the robot
         */
        virtual lib15442c::Angle get_rotation() = 0;
        /**
         * @brief Set the rotation of the robot
         *
         * @param rotation The new rotation of the robot
         */
        virtual void set_rotation(lib15442c::Angle rotation) = 0;
    };
    
    #ifndef LIB15442C_MOCK_DEVICES_ONLY

    struct TrackerIMU
    {
        std::shared_ptr<pros::IMU> imu;
        double scale;
    };
    struct TrackerWheel
    {
        std::shared_ptr<pros::Rotation> tracker;
        double offset;
        double diameter;
    };

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

        // Inertial 2
        std::shared_ptr<pros::v5::IMU> inertial_2;
        double inertial_scale_2;

        // Rotation Offset
        double rotation_offset = 0;

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

        void set_mirrored(bool mirrored);
        bool get_mirrored();

        double get_x();
        double get_y();
        Vec get_position();
        Pose get_pose();

        void set_x(double x);
        void set_y(double y);
        void set_position(Vec position);

        Angle get_rotation();
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

    class GPSOdom : public virtual IOdometry
    {
    private:
        pros::GPS gps;

        double rotation_offset;
        bool mirrored;

        static constexpr double inches_per_meter = 39.3701;

    public:
        GPSOdom(int port, double x_offset, double y_offset, double rotation_offset = 0, bool mirrored = false);

        void set_mirrored(bool mirrored);
        bool get_mirrored();

        double get_x();
        double get_y();
        Vec get_position();
        Pose getPose();

        void set_x(double x);
        void set_y(double y);
        void set_position(Vec position);

        Angle get_rotation();
        void set_rotation(Angle rotation);
    };

    #endif
}