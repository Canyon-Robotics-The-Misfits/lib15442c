#pragma once

#include "lib15442c/math/pose.hpp"
#include "lib15442c/chasis/drivetrain.hpp"
#include "lib15442c/chasis/odometry.hpp"
#include "lib15442c/rtos.hpp"

#include <utility>

namespace lib15442c {
    class IMotion {
    protected:
        virtual bool isAsync() = 0;

        lib15442c::Mutex async_mutex;
        bool is_running = false;

    public:
        virtual std::pair<double, double> calculate(std::shared_ptr<IDrivetrain> drivetrain, Pose pose, double time_since_start) = 0;

        /**
         * @brief Execute the motion. Will block until done if not async
         * 
         */
        void execute(std::shared_ptr<IDrivetrain> drivetrain, std::shared_ptr<IOdometry> odometry, bool ignore_async = false);
    };
}