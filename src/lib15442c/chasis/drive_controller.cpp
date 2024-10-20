#include "drive_controller.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

// #include <algorithm>

#define LOGGER "drive_controller.cpp"

lib15442c::DriveController::DriveController(
            std::shared_ptr<IDrivetrain> drivetrain,
            std::shared_ptr<IOdometry> odometry,
            std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid,
            double default_min_speed) : drivetrain(drivetrain), odometry(odometry), drive_pid(drive_pid), turn_pid(turn_pid), default_min_speed(default_min_speed){};


std::shared_ptr<lib15442c::Face> lib15442c::DriveController::turn(lib15442c::Angle angle, FaceParameters parameters, std::string name)
{
    return faceAngle(
        odometry->getRotation() + angle,
        parameters,
        name
    );
}

std::shared_ptr<lib15442c::Face> lib15442c::DriveController::faceAngle(lib15442c::Angle angle, FaceParameters parameters, std::string name)
{
    return face(
        FaceAngleTarget{ angle: angle },
        parameters,
        name
    );
}

std::shared_ptr<lib15442c::Face> lib15442c::DriveController::facePoint(lib15442c::Vec pos, lib15442c::Angle angle_offset, FaceParameters parameters, std::string name)
{
    return face(
        FacePointTarget{ pos: pos, angle_offset: angle_offset },
        parameters,
        name
    );
}

std::shared_ptr<lib15442c::Face> lib15442c::DriveController::face(FaceTarget face_target, FaceParameters parameters, std::string name)
{
    std::shared_ptr<lib15442c::Face> movement = std::make_shared<lib15442c::Face>(
        face_target,
        turn_pid,
        parameters,
        name
    );

    movement->initialize(drivetrain, odometry->getPose());
    movement->execute(drivetrain, odometry);

    return movement;
}

std::shared_ptr<lib15442c::DriveStraight> lib15442c::DriveController::drive(double distance, DriveParameters parameters, std::string name)
{
    std::shared_ptr<lib15442c::DriveStraight> movement = std::make_shared<lib15442c::DriveStraight>(
        distance,
        drive_pid,
        turn_pid,
        parameters,
        name
    );

    movement->initialize(drivetrain, odometry->getPose());
    movement->execute(drivetrain, odometry);

    return movement;
}

void lib15442c::DriveController::drive_time(double voltage, double time, DriveTimeParameters parameters)
{
    if (parameters.async)
    {
        parameters.async = false;
        pros::Task([this, voltage, time, parameters] {
            drive_time(voltage, time, parameters);
        });

        pros::delay(10);

        return;
    }

    INFO_TEXT("starting drive time");

    double start_time = pros::millis();

    if (parameters.angle.is_none())
    {
        parameters.angle = odometry->getRotation();
    }

    while (pros::millis() - start_time < time)
    {
        double current_time = pros::millis() - start_time;

        double ramp_up = fabs((parameters.ramp_speed / 1000.0) * current_time);
        double ramp_down = fabs(-(parameters.ramp_speed / 1000.0) * (current_time - time));

        double out;

        if (parameters.ramp_up && parameters.ramp_down)
        {
            out = std::min(std::min(ramp_up, ramp_down), fabs(voltage));
        }
        else if  (parameters.ramp_up)
        {
            out = std::min(ramp_up, fabs(voltage));
        }
        else if (parameters.ramp_down)
        {
            out = std::min(ramp_down, fabs(voltage));
        }
        else
        {
            out = fabs(voltage);
        }
        out *= lib15442c::sgn(voltage);


        double error = odometry->getRotation().error_from(parameters.angle).deg();
        double turn_speed = turn_pid->calculateError(error);


        drivetrain->move(out, turn_speed);
    }

    drivetrain->move(0, 0);

    INFO_TEXT("ending drive time");
}

std::shared_ptr<lib15442c::Boomerang> lib15442c::DriveController::boomerang(lib15442c::Pose pos, BoomerangParameters parameters, std::string name)
{
    std::shared_ptr<lib15442c::Boomerang> movement = std::make_shared<lib15442c::Boomerang>(
        pos,
        drive_pid,
        turn_pid,
        parameters,
        name
    );

    movement->initialize(drivetrain, odometry->getPose());
    movement->execute(drivetrain, odometry);

    return movement;
}

std::shared_ptr<lib15442c::DriveToIntermediate> lib15442c::DriveController::drive_to(lib15442c::Pose pos, DriveToIntermediateParameters parameters, std::string name)
{
    std::shared_ptr<lib15442c::DriveToIntermediate> movement = std::make_shared<lib15442c::DriveToIntermediate>(
        pos,
        drive_pid,
        turn_pid,
        parameters,
        name
    );

    movement->initialize(drivetrain, odometry->getPose());
    movement->execute(drivetrain, odometry);

    return movement;
}

std::shared_ptr<lib15442c::DriveToAB> lib15442c::DriveController::drive_to_ab(lib15442c::Pose pos, DriveToABParameters parameters, std::string name)
{
    std::shared_ptr<lib15442c::DriveToAB> movement = std::make_shared<lib15442c::DriveToAB>(
        pos,
        drive_pid,
        parameters,
        name
    );

    movement->initialize(drivetrain, odometry->getPose());
    movement->execute(drivetrain, odometry);

    return movement;
}