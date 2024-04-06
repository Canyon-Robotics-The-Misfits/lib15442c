#pragma once

#include "drivetrain.hpp"
#include "odometry.hpp"
#include "lib15442c/controller/pid.hpp"
#include "lib15442c/math/pose.hpp"

#include <variant>

namespace lib15442c
{
    /**
     * @brief The parameters for the turning functions
     */
    struct AngleParameters
    {
        /**
         * @brief The radius of the curved path to follow, 0 to turn in place
         *
         */
        double arc_radius = 0;

        /**
         * @brief The threshold which error need to be in to end
         *
         */
        lib15442c::Angle threshold = 0.75_deg;

        /**
         * @brief The timeout in case the move takes too long (ms)
         */
        double timeout = 1000;

        /**
         * @brief The maximum speed
         */
        double max_speed = 127;
        /**
         * @brief The minimum speed, defaults to the one set by the DriveController constructor
         */
        double min_speed = -1;

        /**
         * @brief Whether to use the chain exit condition instead of the normal one
         */
        bool chained = false;
        /**
         * @brief Whether to run asynchronously
         */
        bool async = false;
    };

    struct FaceAngle {
        lib15442c::Angle angle;
    };

    struct FacePoint {
        lib15442c::Pose pos;
        lib15442c::Angle angle_offset;
    };

    using FaceTarget = std::variant<FaceAngle, FacePoint>;

    /**
     * @brief The parameters for the drive function
     */
    struct DriveParameters
    {
        /**
         * @brief The angle to drive at to keep straight, defaults to the current angle
         */
        lib15442c::Angle angle = lib15442c::Angle::none();

        /**
         * @brief The threshold which error must be in to end
         */
        double threshold = 0.1;

        /**
         * @brief The timeout in case the move takes too long (ms)
         */
        double timeout = 1000;

        /**
         * @brief The maximum speed
         */
        double max_speed = 127;
        /**
         * @brief The minimum speed, defaults to the one set by the DriveController constructor
         */
        double min_speed = -1;

        /**
         * @brief Whether to use the chain exit condition instead of the normal one
         */
        bool chained = false;
        /**
         * @brief Whether to run asynchronously
         */
        bool async = false;
    };

    /**
     * @brief The parameters for the boomerang controller
     */
    struct BoomerangParameters
    {
        /**
         * @brief Whether to go in reverse
         */
        bool backwards = false;
        /**
         * @brief The lead, should be between 0 and 1. Affects how far away the path should curve into the target
         */
        double lead = 0.6;
        /**
         * @brief How far away the robot should be from the end before the movement ends
         */
        double threshold = 1;
        /**
         * @brief The distance from the end in which the target point is ignored, and the program focuses on just getting the right angle
         */
        double cross_track_threshold = 3;
        /**
         * @brief How much to prioritize turning over driving; Outside of this error threshold (degrees), the drive speed is reduced to prioritize turning
         */
        double turn_priority = 15;

        /**
         * @brief The timeout in case the move takes too long (ms)
         */
        double timeout = 5;

        /**
         * @brief The maximum speed
         */
        double max_speed = 127;
        /**
         * @brief The minimum speed, defaults to the one set by the DriveController constructor
         */
        double min_speed = -1;

        /**
         * @brief Whether to use the chain exit condition instead of the normal one
         */
        bool chained = false;
        /**
         * @brief Whether to run asynchronously
         */
        bool async = false;
    };

    class DriveController
    {
    private:
        std::shared_ptr<IDrivetrain> drivetrain;
        std::shared_ptr<IOdometry> odometry;

        std::shared_ptr<PID> drive_pid;
        std::shared_ptr<PID> turn_pid;

        pros::Mutex async_mutex;

        double default_min_speed;

    public:
        DriveController(
            std::shared_ptr<IDrivetrain> drivetrain,
            std::shared_ptr<IOdometry> odometry,
            std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid,
            double default_min_speed = 12);

        // turn functions

        /**
         * @brief Turn a certain amount relative to the robot's current rotation
         *
         * @param angle The angle to end at
         * @param parameters Any extra parameters
         */
        void turn(lib15442c::Angle angle, AngleParameters parameters = {});

        /**
         * @brief Face a global rotation
         * @param angle The angle to face
         * @param parameters Any extra parameters
         */
        void faceAngle(lib15442c::Angle angle, AngleParameters parameters = {});

        /**
         * @brief Face a point in x-y space
         *
         * @param pos The position to face
         * @param angle_offset An amount to offset the faced angle from the target point
         * @param parameters Any extra parameters
         */
        void facePoint(lib15442c::Pose pos, lib15442c::Angle angle_offset = 0_rad, AngleParameters parameters = {});

        /**
         * @brief Face either an absolute angle or a point
         * 
         * @param face_target The target of the face command
         * @param parameters Any extra parameters
         */
        void face(FaceTarget face_target, AngleParameters parameters = {});

        // drive functions

        /**
         * @brief Drive a distance using the drive PID
         *
         * @param distance The distance to drive (inches)
         * @param parameters Any extra parameters
         */
        void drive(double distance, DriveParameters parameters = {});

        /**
         * @brief Drive to a point with the boomerang controller
         * 
         * @param pos The position to drive to (if angle is specified, will try to end at that angle)
         * @param parameters Any extra parameters
         */
        void boomerang(lib15442c::Pose pos, BoomerangParameters parameters = {});

        // async functions

        /**
         * @brief Whether the robot is done moving
         *
         * @param timeout How long to wait before giving up on waiting
         *
         * @return true The robot is done moving
         * @return false The robot is not done moving
         */
        bool isDone(int timeout = 0);

        /**
         * @brief Wait for the robot to finish it's current motion
         */
        void awaitDone();

        /**
         * @brief Wait for the robot to reach a certain point, or for the robot's
         * current motion to end
         *
         * @param pos The point to wait until the robot is near (angle is ignored)
         * @param distance The distance the robot must be closer then to the point
         *
         * @return bool Whether the wait ended at the right time or at the end of
         * the movement
         */
        bool awaitNear(lib15442c::Vec pos, double distance = 1);

        /**
         * @brief Wait for the robot to be close to a certain angle, or for the
         * robot's current motion to end
         *
         * @param angle The angle to wait until the robot is close to
         * @param threshold How close the robot needs to be to the right angle
         *
         * @return bool Whether the wait ended at the right time or at the end of
         * the movement
         */
        bool awaitAngle(lib15442c::Angle angle, lib15442c::Angle threshold = 2_deg);
    };
}