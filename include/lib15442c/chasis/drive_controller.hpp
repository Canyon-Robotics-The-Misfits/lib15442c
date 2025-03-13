#pragma once

#include "drivetrain.hpp"
#include "odometry.hpp"
#include "../controller/pid.hpp"
#include "../math/pose.hpp"
#include "../rtos.hpp"

#include "lib15442c/motion/motion.hpp"
#include "lib15442c/motion/pid_motions.hpp"
#include "lib15442c/motion/drive_to.hpp"

#include <variant>

namespace lib15442c
{
    struct DriveTimeParameters {
        /**
         * @brief The angle to drive at to keep straight, defaults to the current angle
         */
        lib15442c::Angle angle = lib15442c::Angle::none();

        /**
         * @brief Whether to speed up at the start of the drive
         */
        bool ramp_up = false;
        /**
         * @brief Whether to speed down at the end of the drive
         */
        bool ramp_down = false;
        /**
         * @brief How fast to speed up/slow down at the start or end (voltage / second)
         */
        double ramp_speed = 127.0/0.5;

        /**
         * @brief An extra end condition for special cases
         */
        EndCondition end_condition = default_end_condition;

        /**
         * @brief Whether to run asynchronously
         */
        bool async = false;
    };

    /**
     * A wrapper class for the drivetrain and movement classes to make using them more convenient
     */
    class DriveController
    {
    private:
        std::shared_ptr<IDrivetrain> drivetrain;
        std::shared_ptr<IOdometry> odometry;

        std::shared_ptr<PID> drive_pid;
        std::shared_ptr<PID> turn_pid;

    public:
        DriveController(
            std::shared_ptr<IDrivetrain> drivetrain,
            std::shared_ptr<IOdometry> odometry,
            std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid);

        // turn functions

        /**
         * @brief Turn a certain amount relative to the robot's current rotation
         *
         * @param angle The angle to end at
         * @param parameters Any extra parameters
         * @param name The name of the movement
         */
        std::shared_ptr<lib15442c::Face> turn(lib15442c::Angle angle, FaceParameters parameters = {}, std::string name = "turn");

        /**
         * @brief Face a global rotation
         * @param angle The angle to face
         * @param parameters Any extra parameters
         * @param name The name of the movement
         */
        std::shared_ptr<lib15442c::Face> face_angle(lib15442c::Angle angle, FaceParameters parameters = {}, std::string name = "face angle");

        /**
         * @brief Face a point in x-y space
         *
         * @param pos The position to face
         * @param angle_offset An amount to offset the faced angle from the target point
         * @param parameters Any extra parameters
         * @param name The name of the movement
         */
        std::shared_ptr<lib15442c::Face> face_point(lib15442c::Vec pos, lib15442c::Angle angle_offset = 0_rad, FaceParameters parameters = {}, std::string name = "face point");

        /**
         * @brief Face either an absolute angle or a point
         * 
         * @param face_target The target of the face command
         * @param parameters Any extra parameters
         * @param name The name of the movement
         */
        std::shared_ptr<lib15442c::Face> face(FaceTarget face_target, FaceParameters parameters = {}, std::string name = "face");

        // drive functions

        /**
         * @brief Drive a distance using the drive PID
         *
         * @param distance The distance to drive (inches)
         * @param parameters Any extra parameters
         * @param name The name of the movement
         */
        std::shared_ptr<lib15442c::DriveStraight> drive(double distance, DriveParameters parameters = {}, std::string name = "drive");

        /**
         * @brief Drive for a set amount of time
         * 
         * @param voltage The voltage/speed to drive at
         * @param time How long to drive for
         * @param parameters Any extra parameters
         */
        void drive_time(double voltage, double time, DriveTimeParameters parameters = {});

        /**
         * @brief Drive to a point with the boomerang controller
         * 
         * @param pos The position to drive to (if angle is specified, will try to end at that angle)
         * @param parameters Any extra parameters
         * @param name The name of the movement
         */
        std::shared_ptr<Boomerang> boomerang(lib15442c::Pose pos, BoomerangParameters parameters = {}, std::string name = "boomerang");

        /**
         * @brief Drive to a pose
         * 
         * @param pos The target pose
         * @param parameters The paameters for the movement
         * @param name The name of the movement
         * @return std::shared_ptr<DriveToIntermediate> The movement object
         */
        std::shared_ptr<DriveToIntermediate> drive_to(lib15442c::Pose pos, DriveToIntermediateParameters parameters = {}, std::string name = "drive to");
        
        /**
         * @brief Drive to a pose
         * 
         * @param pos The target pose
         * @param parameters The parameters for the movement
         * @param name The name of the movment
         * @return std::shared_ptr<DriveToAB> The movement object
         */
        std::shared_ptr<DriveToAB> drive_to_ab(lib15442c::Pose pos, DriveToABParameters parameters = {}, std::string name = "drive to ab");
    };
}