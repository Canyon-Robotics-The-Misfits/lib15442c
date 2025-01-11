#include "ramsete.hpp"

lib15442c::RAMSETE::RAMSETE(lib15442c::Trajectory trajectory, double b, double zeta, bool async, std::string name)
    : trajectory(trajectory), b(b), zeta(zeta), async(async), name(name) {};

bool lib15442c::RAMSETE::is_async()
{
    return async;
}

std::string lib15442c::RAMSETE::get_name()
{
    return name;
}

void lib15442c::RAMSETE::initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose)
{
    track_width = drivetrain->get_track_width();

    last_drive_velocity = 0;
    last_rotational_velocity = 0;
    last_time = 0;
}

lib15442c::MotionOutput lib15442c::RAMSETE::calculate(Pose pose, double time_since_start, double delta_time)
{
    if (time_since_start > trajectory.get_total_time() * 1000.0)
    {
        return MotionOutputExit{};
    }
        
    TrajectoryState target_state = trajectory.get_state(time_since_start / 1000.0);

    // std::cout << time_since_start << ", " << pose.x << ", " << pose.y << ", " << target_state.position.x << ", " << target_state.position.y << std::endl;

    double heading_rad = -pose.angle.rad() + M_PI / 2.0;
    double target_heading_rad = -target_state.heading.rad() + M_PI / 2.0;

    Vec error_global = target_state.position - pose.vec();
    double error_local_x = error_global.x * cos(heading_rad) + error_global.x * sin(heading_rad);
    double error_local_y = -error_global.y * sin(heading_rad) + error_global.y * cos(heading_rad);
    double error_theta = target_heading_rad - heading_rad;

    double k = 2 * zeta * sqrt(target_state.rotational_velocity * target_state.rotational_velocity + b * target_state.drive_velocity * target_state.drive_velocity);

    double drive_velocity = target_state.drive_velocity * cos(error_theta) + k * error_local_x;
    double rotational_velocity = target_state.rotational_velocity + k * error_theta + (b * target_state.drive_velocity * error_local_y) * (error_theta != 0 ? (sin(error_theta) / error_theta) : 1);
    
    // drive_velocity = target_state.drive_velocity;
    // rotational_velocity = target_state.rotational_velocity;

    double drive_accel = (drive_velocity - last_drive_velocity) / ((time_since_start - last_time) / 1000.0);
    double rotational_accel = (rotational_velocity - last_rotational_velocity) / ((time_since_start - last_time) / 1000.0);

    if (time_since_start - last_time == 0)
    {
        drive_accel = 100000;
        rotational_accel = 100000;
    }

    last_drive_velocity = drive_velocity;
    last_rotational_velocity = rotational_velocity;
    last_time = time_since_start;

    // std::cout << time_since_start / 1000.0 << ", ";
    return MotionOutputSpeeds {
        drive_velocity: drive_velocity,
        rotational_velocity: rotational_velocity,
        drive_accel: drive_accel,
        rotational_accel: rotational_accel
    };
}