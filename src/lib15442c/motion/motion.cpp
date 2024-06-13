#include "motion.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "Motion"

void lib15442c::IMotion::execute(std::shared_ptr<IDrivetrain> drivetrain, std::shared_ptr<IOdometry> odometry, bool ignore_async)
{
    if (!ignore_async && isAsync()) {
        task = pros::Task([this, drivetrain, odometry] {
            execute(drivetrain, odometry, true);
        });

        pros::delay(10);

        return;
    }
    
    INFO("starting \"%s\"", getName().c_str());

    async_mutex.lock();
    is_running = true;
    async_mutex.unlock();

    int start_time = pros::millis();
    int last_time = pros::millis();

    while (true) {
        async_mutex.lock();
        if (!is_running) {
            break;
        }
        async_mutex.unlock();

        Pose current_pose = odometry->getPose();

        int now = pros::millis();
        int time_since_start = now - start_time;

        auto out = calculate(current_pose, time_since_start, now - last_time);

        last_time = now;

        if (MotionOutputSpeeds *speeds = std::get_if<MotionOutputSpeeds>(&out))
        {
            drivetrain->move_ratio(speeds->linear_output, speeds->rotational_output);
        }
        else
        {
            break;
        }

        pros::delay(10);
    }

    if (!isAsync()) {
        drivetrain->move(0, 0);
    }

    async_mutex.lock();
    is_running = false;
    async_mutex.unlock();
    
    INFO("ending \"%s\"", getName().c_str());
}

bool lib15442c::IMotion::isRunning() {
    async_mutex.lock();
    bool temp = is_running;
    async_mutex.unlock();

    return temp;
}

void lib15442c::IMotion::stop() {
    async_mutex.lock();
    is_running = false;
    async_mutex.unlock();

    task.join(); // wait for the task to end
}