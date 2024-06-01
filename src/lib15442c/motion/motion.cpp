#include "motion.hpp"

void lib15442c::IMotion::execute(std::shared_ptr<IDrivetrain> drivetrain, std::shared_ptr<IOdometry> odometry, bool ignore_async)
{
    if (!ignore_async && isAsync()) {
        pros::Task([this, drivetrain, odometry] {
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

        drivetrain->move_ratio(out.first, out.second);

        pros::delay(10);
    }

    if (!isAsync()) {
        drivetrain->move(0, 0);
    }

    async_mutex.lock();
    is_running = false;
    async_mutex.unlock();
}