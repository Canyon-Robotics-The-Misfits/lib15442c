#include "motion.hpp"

void lib15442c::IMotion::execute(std::shared_ptr<IDrivetrain> drivetrain, std::shared_ptr<IOdometry> odometry, bool ignore_async)
{
    if (!ignore_async && isAsync()) {
        task = pros::Task([this, drivetrain, odometry] {
            execute(drivetrain, odometry, true);
        });

        pros::delay(10);

        return;
    }

    async_mutex.lock();
    is_running = true;
    async_mutex.unlock();

    int start_time = pros::millis();

    while (true) {
        async_mutex.lock();
        if (!is_running) {
            break;
        }
        async_mutex.unlock();

        Pose current_pose = odometry->getPose();
        int time_since_start = pros::millis() - start_time;

        auto out = calculate(drivetrain, current_pose, time_since_start);

        drivetrain->move_ratio(out.linear_output, out.rotational_output);

        pros::delay(10);
    }

    if (!isAsync()) {
        drivetrain->move(0, 0);
    }

    async_mutex.lock();
    is_running = false;
    async_mutex.unlock();
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