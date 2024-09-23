#include "lib15442c/chasis/odometry.hpp"
#include "lib15442c/math/vector.hpp"
#include "lib15442c/math/pose.hpp"
#include "lib15442c/math/angle.hpp"
#include "lib15442c/logger.hpp"

#include <cmath>

#define LOGGER "odometry.cpp"

#ifndef LIB15442C_MOCK_DEVICES_ONLY

lib15442c::TrackerOdom::TrackerOdom(
    TrackerWheel parallel_tracker,
    TrackerWheel perpendicular_tracker,
    bool mirrored,
    TrackerIMU inertial,
    TrackerIMU inertial_2) : parallel_tracker(parallel_tracker.tracker), perpendicular_tracker(perpendicular_tracker.tracker),
                             inertial(inertial.imu), inertial_scale(inertial.scale),
                             parallel_tracker_offset(parallel_tracker.offset), parallel_tracker_circumfrance(parallel_tracker.diameter * M_PI),
                             perpendicular_tracker_offset(perpendicular_tracker.offset), perpendicular_tracker_circumfrance(perpendicular_tracker.diameter * M_PI),
                             mirrored(mirrored), inertial_2(inertial_2.imu), inertial_scale_2(inertial_2.scale)
{
    if (!this->parallel_tracker->is_installed())
    {
        ERROR("Parallel Tracker is not detected on port %d!", this->parallel_tracker->get_port());
    }
    if (!this->perpendicular_tracker->is_installed())
    {
        ERROR("Perpendicular Tracker is not detected on port %d!", this->perpendicular_tracker->get_port());
    }
    if (!this->inertial->is_installed())
    {
        ERROR("IMU is not detected on port %d!", this->inertial->get_port());
    }
    if (this->inertial_2->get_port() != 22 && !this->inertial_2->is_installed())
    {
        ERROR("IMU 2 is not detected on port %d!", this->inertial_2->get_port());
    }

    this->parallel_tracker->reset_position();
    this->perpendicular_tracker->reset_position();
}

lib15442c::TrackerOdom::~TrackerOdom()
{
    stopTask();
}

void lib15442c::TrackerOdom::setMirrored(bool mirrored)
{
    position_mutex.lock();
    this->mirrored = mirrored;
    position_mutex.unlock();
}

bool lib15442c::TrackerOdom::getMirrored()
{
    position_mutex.lock();
    bool temp = mirrored;
    position_mutex.unlock();

    return temp;
}

void lib15442c::TrackerOdom::setX(double val)
{
    position_mutex.lock();
    position.x = val;
    position_mutex.unlock();
}

void lib15442c::TrackerOdom::setY(double val)
{
    position_mutex.lock();
    position.y = val;
    position_mutex.unlock();
}

double lib15442c::TrackerOdom::getX()
{
    position_mutex.lock();
    double temp = position.x;
    position_mutex.unlock();

    return temp;
}

double lib15442c::TrackerOdom::getY()
{
    position_mutex.lock();
    double temp = position.y;
    position_mutex.unlock();

    return temp;
}

lib15442c::Vec lib15442c::TrackerOdom::getPosition()
{
    position_mutex.lock();
    Vec temp = position;
    position_mutex.unlock();

    // std::cout << inertial->get_rotation() * inertial_scale << ", " << parallel_tracker->get_position() / 100.0 * parallel_tracker_circumfrance / 360 / 1.00771827217 << ", " << perpendicular_tracker->get_position() / 100.0 * perpendicular_tracker_circumfrance / 360 / 1.00771827217 << std::endl;

    return temp;
}
lib15442c::Pose lib15442c::TrackerOdom::getPose()
{
    position_mutex.lock();
    Vec temp = position;
    position_mutex.unlock();
    Angle rotation = getRotation();

    return pose(temp.x, temp.y, rotation);
}

void lib15442c::TrackerOdom::setPosition(lib15442c::Vec position)
{
    position_mutex.lock();
    this->position = position;
    position_mutex.unlock();
}

lib15442c::Angle lib15442c::TrackerOdom::getRotation()
{
    position_mutex.lock();
    double imu_1 = inertial->get_rotation() * inertial_scale;
    double imu_2 = imu_1;

    if (inertial_2->is_installed())
    {
        imu_2 = inertial_2->get_rotation() * inertial_scale_2;
    }
    position_mutex.unlock();

    return Angle::from_deg((imu_1 + imu_2) / 2.0 * (getMirrored() ? -1 : 1));
}

void lib15442c::TrackerOdom::setRotation(Angle rotationOffset)
{
    position_mutex.lock();
    rotation_offset = rotationOffset.deg();
    position_mutex.unlock();
    // Makes sure odometry updates before continuing
    pros::delay(15);
}

void lib15442c::TrackerOdom::startTask()
{

#ifndef LIB15442C_MOCK_DEVICES_ONLY

    task = pros::Task([this]
                      {
        inertial->tare();
        if (inertial_scale_2 != 0) {
            inertial_2->tare();
        }

        double last_parallel = parallel_tracker->get_position() / 100.0;
        double last_perpendicular = perpendicular_tracker->get_position() / 100.0;

        int time = 0;
        int tickTimer = 0;

        double offset_zero = 0;
        double last_angle = getRotation().rad();

        double degrees_per_inch_parallel = parallel_tracker_circumfrance / 360 / 1.00771827217;
        double degrees_per_inch_perpendicular = perpendicular_tracker_circumfrance / 360 / 1.00771827217;
        // double degrees_per_inch = tracker_circumfrance / 360;

        while (true)
        {

            if (rotation_offset != 0)
            {
                inertial->set_rotation(rotation_offset / inertial_scale);
                if (inertial_scale_2 != 0) {
                    inertial_2->set_rotation(rotation_offset / inertial_scale_2);
                }
                offset_zero = rotation_offset;
                rotation_offset = 0;
            }
            else
            {
                // Get the tracker wheel encoder positions
                double parallel = parallel_tracker->get_position() / 100.0;
                double perpendicular = perpendicular_tracker->get_position() / 100.0;

                // Get the current robot rotation
                double angle = getRotation().rad();

                if (std::isnan(angle))
                {
                    last_angle = 0;
                    continue;
                }

                // Modify the horizontal encoder to compensate for turning
                perpendicular -= (angle - offset_zero) * perpendicular_tracker_offset;
                parallel -= (angle - offset_zero) * parallel_tracker_offset;

                // Calculate the change in the horizontal and vertical encoder
                double deltaTheta = angle - last_angle;
                double deltaParallel = (parallel - last_parallel) * degrees_per_inch_parallel;
                double deltaPerpendicular = (perpendicular - last_perpendicular) * degrees_per_inch_perpendicular;

                position_mutex.lock();

                if (deltaTheta == 0)
                {
                    position += Vec(
                        cos(angle) * deltaParallel +
                            cos(angle + M_PI/2) * deltaPerpendicular,
                        sin(angle) * deltaParallel +
                            sin(angle + M_PI/2) * deltaPerpendicular);
                }
                else
                {
                    double radiusParallel = deltaParallel / deltaTheta;
                    double radiusPerpendicular = deltaPerpendicular / deltaTheta;

                    double delta_x = (cos(angle) - cos(last_angle)) * radiusParallel;
                    delta_x += (cos(angle + M_PI/2) - cos(last_angle + M_PI/2)) * radiusPerpendicular;

                    double delta_y = (sin(angle) - sin(last_angle)) * radiusParallel;
                    delta_y += (sin(angle + M_PI/2) - sin(last_angle + M_PI/2)) * radiusPerpendicular;

                    position += Vec(
                        delta_x,
                        delta_y
                    );
                }

                // Log position in terminal
                // if (tickTimer % 10 == 0)
                //     std::cout << angle << ", " << position.x << ", " << position.y << std::endl;

                position_mutex.unlock(); // unlock the mutex

                // Set the last variables
                last_perpendicular = perpendicular;
                last_parallel = parallel;
                last_angle = angle;
            }

            if (pros::Task::notify_take(true, 10) > 0 || ((pros::c::competition_get_status() & COMPETITION_DISABLED) != 0)) {
                break;
            }
            time += 10;
            tickTimer++;
        } });
#endif
}

void lib15442c::TrackerOdom::stopTask()
{
    task.notify();
    task.join();
}

// GPS odom

lib15442c::GPSOdom::GPSOdom(int port, double x_offset, double y_offset, double rotation_offset, bool mirrored)
    : gps(pros::GPS(port)), rotation_offset(rotation_offset), mirrored(mirrored)
{
    gps.set_offset(x_offset, y_offset);
};

void lib15442c::GPSOdom::setMirrored(bool mirrored)
{
    this->mirrored = mirrored;
}

bool lib15442c::GPSOdom::getMirrored()
{
    return mirrored;
}

void lib15442c::GPSOdom::setX(double val)
{
    gps.set_position((val - 72) / inches_per_meter, (getY() - 72) / inches_per_meter, getRotation().deg() - rotation_offset);
}

void lib15442c::GPSOdom::setY(double val)
{
    gps.set_position((getX() - 72) / inches_per_meter, (val - 72) / inches_per_meter, getRotation().deg() - rotation_offset);
}

double lib15442c::GPSOdom::getX()
{
    return getPosition().x;
}

double lib15442c::GPSOdom::getY()
{
    return getPosition().y;
}

lib15442c::Vec lib15442c::GPSOdom::getPosition()
{
    auto gps_position = gps.get_position();
    lib15442c::Vec position = Vec(gps_position.x * (mirrored ? -1 : 1) + 72 / inches_per_meter, gps_position.y + 72 / inches_per_meter);

    return position * inches_per_meter;
}
lib15442c::Pose lib15442c::GPSOdom::getPose()
{
    auto gps_position = getPosition();
    Angle rotation = getRotation();
    Pose position = pose(gps_position.x, gps_position.y, rotation);

    return position;
}

void lib15442c::GPSOdom::setPosition(lib15442c::Vec position)
{
    gps.set_position((position.x - 72) / inches_per_meter, (position.y - 72) / inches_per_meter, getRotation().deg() - rotation_offset);
}

lib15442c::Angle lib15442c::GPSOdom::getRotation()
{
    return Angle::from_deg((gps.get_heading() + rotation_offset) * (getMirrored() ? -1 : 1));
}

void lib15442c::GPSOdom::setRotation(Angle rotation)
{
    rotation_offset = rotation.deg();
}

#endif