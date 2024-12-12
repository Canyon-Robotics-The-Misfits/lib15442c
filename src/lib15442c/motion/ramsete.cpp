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
}

lib15442c::MotionOutput lib15442c::RAMSETE::calculate(Pose pose, double time_since_start, double delta_time)
{
    TrajectoryState target_state = trajectory.get_state(time_since_start / 1000.0);

    double heading_rad = -pose.angle.rad() + M_PI / 2.0;
    double target_heading_rad = -target_state.heading.rad() + M_PI / 2.0;

    Vec error_global = target_state.position - pose.vec();
    double error_local_x = error_global.x * cos(heading_rad) + error_global.x * sin(heading_rad);
    double error_local_y = -error_global.y * sin(heading_rad) + error_global.y * cos(heading_rad);
    double error_theta = target_heading_rad - heading_rad;

    double k = 2 * zeta * sqrt(target_state.rotational_velocity * target_state.rotational_velocity + b * target_state.drive_velocity * target_state.drive_velocity);

    double drive_velocity = target_state.drive_velocity * cos(error_theta) + k * error_local_x;
    double rotational_velocity = target_state.rotational_velocity + k * error_theta + (b * target_state.drive_velocity * sin(error_theta) * error_local_y) / error_theta;

    return MotionOutputSpeeds {
        drive_velocity: drive_velocity,
        rotational_velocity: rotational_velocity
    };
}