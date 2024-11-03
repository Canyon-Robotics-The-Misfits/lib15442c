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
    stop_task();
}

void lib15442c::TrackerOdom::set_mirrored(bool mirrored)
{
    position_mutex.lock();
    this->mirrored = mirrored;
    position_mutex.unlock();
}

bool lib15442c::TrackerOdom::get_mirrored()
{
    position_mutex.lock();
    bool temp = mirrored;
    position_mutex.unlock();

    return temp;
}

void lib15442c::TrackerOdom::set_x(double val)
{
    position_mutex.lock();
    position.x = val;
    position_mutex.unlock();
}

void lib15442c::TrackerOdom::set_y(double val)
{
    position_mutex.lock();
    position.y = val;
    position_mutex.unlock();
}

double lib15442c::TrackerOdom::get_x()
{
    position_mutex.lock();
    double temp = position.x;
    position_mutex.unlock();

    return temp;
}

double lib15442c::TrackerOdom::get_y()
{
    position_mutex.lock();
    double temp = position.y;
    position_mutex.unlock();

    return temp;
}

lib15442c::Vec lib15442c::TrackerOdom::get_position()
{
    position_mutex.lock();
    Vec temp = position;
    position_mutex.unlock();

    // std::cout << inertial->get_rotation() * inertial_scale << ", " << parallel_tracker->get_position() / 100.0 * parallel_tracker_circumfrance / 360 / 1.00771827217 << ", " << perpendicular_tracker->get_position() / 100.0 * perpendicular_tracker_circumfrance / 360 / 1.00771827217 << std::endl;

    return temp;
}
lib15442c::Pose lib15442c::TrackerOdom::get_pose()
{
    position_mutex.lock();
    Vec temp = position;
    position_mutex.unlock();
    Angle rotation = get_rotation();

    return pose(temp.x, temp.y, rotation);
}

void lib15442c::TrackerOdom::set_position(lib15442c::Vec position)
{
    position_mutex.lock();
    this->position = position;
    position_mutex.unlock();
}

lib15442c::Angle lib15442c::TrackerOdom::get_rotation()
{
    position_mutex.lock();
    double imu_1 = inertial->get_rotation() * inertial_scale;
    double imu_2 = imu_1;

    if (inertial_2->is_installed())
    {
        imu_2 = inertial_2->get_rotation() * inertial_scale_2;
    }
    position_mutex.unlock();

    return Angle::from_deg((imu_1 + imu_2) / 2.0) * (get_mirrored() ? -1 : 1);
}

void lib15442c::TrackerOdom::set_rotation(Angle rotationOffset)
{
    position_mutex.lock();
    rotation_offset = rotationOffset.deg();
    position_mutex.unlock();
    // Makes sure odometry updates before continuing
    pros::delay(15);
}

void lib15442c::TrackerOdom::start_task()
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
        double last_angle = get_rotation().rad();

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
                double angle = get_rotation().rad();

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
                
                if (std::isnan(position.x) || std::isnan(position.y))
                {
                    position.x = 0;
                    position.y = 0;
                    continue;
                }

                if (deltaTheta == 0)
                {
                    position += Vec(
                        cos(-angle) * deltaParallel +
                            cos(-angle + M_PI / 2.0) * deltaPerpendicular,
                        sin(-angle) * deltaParallel +
                            sin(-angle + M_PI / 2.0) * deltaPerpendicular);
                }
                else
                {
                    double radiusParallel = deltaParallel / deltaTheta;
                    double radiusPerpendicular = deltaPerpendicular / deltaTheta;

                    double delta_x = (cos(-angle) - cos(-last_angle)) * radiusParallel;
                    delta_x += (cos(-angle + M_PI / 2.0) - cos(-last_angle + M_PI / 2.0)) * radiusPerpendicular;

                    double delta_y = (sin(-angle) - sin(-last_angle)) * radiusParallel;
                    delta_y += (sin(-angle + M_PI / 2.0) - sin(-last_angle + M_PI / 2.0)) * radiusPerpendicular;

                    position += Vec(
                        delta_x,
                        delta_y
                    );
                }

                // Log position in terminal
                // if (tickTimer % 10 == 0)
                //     std::cout << position.x << ", " << position.y << std::endl;

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

void lib15442c::TrackerOdom::stop_task()
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

void lib15442c::GPSOdom::set_mirrored(bool mirrored)
{
    this->mirrored = mirrored;
}

bool lib15442c::GPSOdom::get_mirrored()
{
    return mirrored;
}

void lib15442c::GPSOdom::set_x(double val)
{
    gps.set_position((val - 72) / inches_per_meter, (get_y() - 72) / inches_per_meter, get_rotation().deg() - rotation_offset);
}

void lib15442c::GPSOdom::set_y(double val)
{
    gps.set_position((get_x() - 72) / inches_per_meter, (val - 72) / inches_per_meter, get_rotation().deg() - rotation_offset);
}

double lib15442c::GPSOdom::get_x()
{
    return get_position().x;
}

double lib15442c::GPSOdom::get_y()
{
    return get_position().y;
}

lib15442c::Vec lib15442c::GPSOdom::get_position()
{
    auto gps_position = gps.get_position();
    lib15442c::Vec position = Vec(gps_position.x * (mirrored ? -1 : 1) + 72 / inches_per_meter, gps_position.y + 72 / inches_per_meter);

    return position * inches_per_meter;
}
lib15442c::Pose lib15442c::GPSOdom::get_pose()
{
    auto gps_position = get_position();
    Angle rotation = get_rotation();
    Pose position = pose(gps_position.x, gps_position.y, rotation);

    return position;
}

void lib15442c::GPSOdom::set_position(lib15442c::Vec position)
{
    gps.set_position((position.x - 72) / inches_per_meter, (position.y - 72) / inches_per_meter, get_rotation().deg() - rotation_offset);
}

lib15442c::Angle lib15442c::GPSOdom::get_rotation()
{
    return Angle::from_deg((gps.get_heading() + rotation_offset) * (get_mirrored() ? -1 : 1));
}

void lib15442c::GPSOdom::set_rotation(Angle rotation)
{
    rotation_offset = rotation.deg();
}

#endif