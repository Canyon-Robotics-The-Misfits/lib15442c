#pragma once

#include <memory>

#include "lib15442c/math/vector.hpp"
#include "lib15442c/math/angle.hpp"

#include "pros/rtos.hpp"
#include "pros/rotation.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"

namespace lib15442c
{
    class IOdometry
    {
    public:
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
            std::shared_ptr<pros::Rotation> parallel_tracker,
            std::shared_ptr<pros::Rotation> perpendicular_tracker,
            std::shared_ptr<pros::IMU> inertial, double inertial_scale,
            double tracker_circumfrance, double perpendicular_tracker_offset, double parallel_tracker_offset,
            std::shared_ptr<pros::IMU> inertial_2 = NULL, double inertial_scale_2 = 0) :

                                                                                         parallel_tracker(parallel_tracker), perpendicular_tracker(perpendicular_tracker),
                                                                                         inertial(inertial), inertial_scale(inertial_scale), tracker_circumfrance(tracker_circumfrance),
                                                                                         perpendicular_tracker_offset(perpendicular_tracker_offset), parallel_tracker_offset(parallel_tracker_offset),
                                                                                         inertial_2(inertial_2), inertial_scale_2(inertial_scale_2){};

        ~TrackerOdom();

        double getX();
        double getY();
        Vec getPosition();

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

        static constexpr double inches_per_meter = 39.3701;

    public:
        GPSOdom(int port, double x_offset, double y_offset, double rotation_offset);

        double getX();
        double getY();
        Vec getPosition();

        void setX(double x);
        void setY(double y);
        void setPosition(Vec position);

        Angle getRotation();
        void setRotation(Angle rotation);
    };
}