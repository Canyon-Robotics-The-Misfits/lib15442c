#include "drive_controller.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#include <algorithm>

#define LOGGER "Drive Controller"

lib15442c::DriveController::DriveController(
            std::shared_ptr<IDrivetrain> drivetrain,
            std::shared_ptr<IOdometry> odometry,
            std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid,
            double default_min_speed) : drivetrain(drivetrain), odometry(odometry), drive_pid(drive_pid), turn_pid(turn_pid), default_min_speed(default_min_speed){};

bool lib15442c::DriveController::isDone(int timeout)
{
    bool done = async_mutex.try_lock_for(std::chrono::milliseconds(timeout));
    if (done)
    {
        async_mutex.unlock();
    }

    return done;
}

void lib15442c::DriveController::awaitDone()
{
    async_mutex.take(TIMEOUT_MAX);
    async_mutex.unlock();
}

bool lib15442c::DriveController::awaitNear(lib15442c::Vec pos, double distance)
{
    while (true)
    {
        if (odometry->getPosition().distance_to(pos) < distance)
        {
            return true;
        }

        if (isDone(5))
        {
            return false;
        }
    }
}

bool lib15442c::DriveController::awaitAngle(Angle angle, Angle threshold)
{
    while (true)
    {
        Angle rotation = odometry->getRotation();

        if (fabs(rotation.error_from(angle).deg()) <= threshold.deg())
        {
            return true;
        }

        if (isDone(5))
        {
            return false;
        }
    }
}