#pragma once

#include "lib15442c/math/pose.hpp"
#include "lib15442c/chasis/drivetrain.hpp"
#include "lib15442c/chasis/odometry.hpp"
#include "lib15442c/rtos.hpp"
#include "lib15442c/controller/pid.hpp"

#include <utility>
#include <variant>

namespace lib15442c {
    struct MotionOutputSpeeds {
        double linear_output;
        double rotational_output;
    };

    struct MotionOutputExit {};

    using MotionOutput = std::variant<MotionOutputSpeeds, MotionOutputExit>;

    class IMotion {
    protected:
        virtual bool isAsync() = 0;
        virtual std::string getName() = 0;

        lib15442c::Mutex async_mutex;
        bool is_running = false;

        pros::Task task = pros::Task([] { return; });

    public:
        virtual MotionOutput calculate(Pose pose, double time_since_start, double delta_time) = 0;
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
        bool isRunning();
        /**
         * @brief Stop the motion if it is currently running
         * 
         */
        void stop();
    };
}