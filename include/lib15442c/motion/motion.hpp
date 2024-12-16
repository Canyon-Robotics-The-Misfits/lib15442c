#pragma once

#include "lib15442c/math/pose.hpp"
#include "lib15442c/chasis/drivetrain.hpp"
#include "lib15442c/chasis/odometry.hpp"
#include "lib15442c/rtos.hpp"
#include "lib15442c/controller/pid.hpp"

#include <utility>
#include <variant>

namespace lib15442c {
    struct MotionOutputVolts {
        double linear_output;
        double rotational_output;
    };
    
    struct MotionOutputSpeeds {
        double drive_velocity; // in/s
        double rotational_velocity; // rad/s
        double drive_accel; // in/s/s
        double rotational_accel; // rad/s/s
    };

    struct MotionOutputExit {};

    using MotionOutput = std::variant<MotionOutputVolts, MotionOutputSpeeds, MotionOutputExit>;

    /**
     * An abstract motion algorithm
     */
    class IMotion {
    protected:
        virtual bool is_async() = 0;
        virtual std::string get_name() = 0;

        lib15442c::Mutex async_mutex;
        bool task_enabled = false;

        pros::Task task = pros::Task([] { return; });

    public:
        /**
         * @brief Tick the algorithm once
         * 
         * @param pose The current robot pose
         * @param time_since_start How much time has elapsed in milliseconds
         * @param delta_time How much time has passed since the last tick
         * @return MotionOutput The output of the algorithm for that tick
         */
        virtual MotionOutput calculate(Pose pose, double time_since_start, double delta_time) = 0;
        /**
         * @brief Initialize the algorithm
         * 
         * @param drivetrain The drivetrain
         * @param pose The initial robot pose
         */
        virtual void initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose) = 0;

        /**
         * @brief Execute the motion. Will block until done if not async
         * 
         */
        void execute(std::shared_ptr<IDrivetrain> drivetrain, std::shared_ptr<IOdometry> odometry, bool ignore_async = false);

        /**
         * @brief Check if the motion is currently running
         * 
         * @return Whether it is running 
         */
        bool is_running();
        /**
         * @brief Block until the motion ends
         */
        void await();
        /**
         * @brief Stop the motion if it is currently running
         */
        void stop();
    };
}