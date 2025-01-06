#include "motion.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "motion.cpp"

void lib15442c::IMotion::execute(std::shared_ptr<IDrivetrain> drivetrain, std::shared_ptr<IOdometry> odometry, bool ignore_async)
{
    if (!ignore_async && is_async()) {
        task = pros::Task([this, drivetrain, odometry] {
            execute(drivetrain, odometry, true);
        });

        pros::delay(10);

        return;
    }
    
    INFO("starting \"%s\"", get_name().c_str());

    async_mutex.lock();
    task_enabled = true;
    async_mutex.unlock();

    int start_time = pros::millis();
    int last_time = pros::millis();

    auto start_status = pros::competition::get_status();

    initialize(drivetrain, odometry->get_pose());

    while (pros::competition::get_status() == start_status) {
        async_mutex.lock();
        if (!task_enabled) {
            break;
        }
        async_mutex.unlock();

        Pose current_pose = odometry->get_pose();

        int now = pros::millis();
        int time_since_start = now - start_time;

        auto out = calculate(current_pose, time_since_start, now - last_time);

        last_time = now;

        if (MotionOutputVolts *speeds = std::get_if<MotionOutputVolts>(&out))
        {
            drivetrain->move_ratio(speeds->linear_output, speeds->rotational_output);
        }
        else if (MotionOutputSpeeds *speeds = std::get_if<MotionOutputSpeeds>(&out))
        {
            drivetrain->move_speed(speeds->drive_velocity, speeds->rotational_velocity, speeds->drive_accel, speeds->rotational_accel);
        }
        else
        {
            break;
        }

        pros::delay(20);
    }

    // if (!is_async()) {
        drivetrain->move(0, 0);
    // }

    async_mutex.lock();
    task_enabled = false;
    async_mutex.unlock();
    
    INFO("ending \"%s\"", get_name().c_str());
}

bool lib15442c::IMotion::is_running() {
    async_mutex.lock();
    bool temp = task_enabled;
    async_mutex.unlock();

    return temp;
}

void lib15442c::IMotion::await()
{
    auto inital_status = pros::competition::get_status();
    while (is_running() && pros::competition::get_status() == inital_status)
    {
        pros::delay(10);
    }
}

void lib15442c::IMotion::stop() {
    async_mutex.lock();
    task_enabled = false;
    async_mutex.unlock();

    task.join(); // wait for the task to end
}