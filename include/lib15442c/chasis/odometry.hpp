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
        virtual void setMirrored(bool mirrored) = 0;
        /**
         * @brief Get whether the position is mirrored over the y-axis
         * 
         * @return bool
         */
        virtual bool getMirrored() = 0;

        /**
         * @brief Get the x position of the robot
         */
        virtual double getX() = 0;
        /**
         * @brief Get the y position of the robot
         */
        virtual double getY() = 0;
        /**
         * @brief Get the position of the robot
         */
        virtual lib15442c::Vec getPosition() = 0;
        /**
         * @brief Get the pose of the robot
         */
        virtual lib15442c::Pose getPose() = 0;

        /**
         * @brief Set the x position of the robot
         *
         * @param x The new x position
         */
        virtual void setX(double x) = 0;
        /**
         * @brief Set the y position of the robot
         *
         * @param y The new y position
         */
        virtual void setY(double y) = 0;
        /**
         * @brief Set the position of the robot
         *
         * @param position The new position
         */
        virtual void setPosition(lib15442c::Vec position) = 0;

        /**
         * @brief Get the rotation of the robot
         */
        virtual lib15442c::Angle getRotation() = 0;
        /**
         * @brief Set the rotation of the robot
         *
         * @param rotation The new rotation of the robot
         */
        virtual void setRotation(lib15442c::Angle rotation) = 0;
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
        double tracker_circumfrance;
        double perpendicular_tracker_offset;
        double parallel_tracker_offset;
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
            double tracker_circumfrance,
            bool mirrored,
            TrackerIMU inertial,
            TrackerIMU inertial_2 = { .imu = NULL, .scale = 0}) :
                 parallel_tracker(parallel_tracker.tracker), perpendicular_tracker(perpendicular_tracker.tracker),
                 inertial(inertial.imu), inertial_scale(inertial.scale), tracker_circumfrance(tracker_circumfrance),
                 parallel_tracker_offset(parallel_tracker.offset), perpendicular_tracker_offset(perpendicular_tracker.offset), 
                 mirrored(mirrored), inertial_2(inertial_2.imu), inertial_scale_2(inertial_2.scale){};

        ~TrackerOdom();

        void setMirrored(bool mirrored);
        bool getMirrored();

        double getX();
        double getY();
        Vec getPosition();
        Pose getPose();

        void setX(double x);
        void setY(double y);
        void setPosition(Vec position);

        Angle getRotation();
        void setRotation(Angle rotation);

        /**
         * @brief Start the odometry background task
         */
        void startTask();
        /**
         * @brief Stop the odometry background task
         */
        void stopTask();
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

        void setMirrored(bool mirrored);
        bool getMirrored();

        double getX();
        double getY();
        Vec getPosition();
        Pose getPose();

        void setX(double x);
        void setY(double y);
        void setPosition(Vec position);

        Angle getRotation();
        void setRotation(Angle rotation);
    };

    #endif
}