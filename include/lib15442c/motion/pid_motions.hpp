#pragma once

#include "motion.hpp"

namespace lib15442c
{
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
         * @brief The time the error needs to be within the threshold for in order to exit
         */
        double threshold_time = 40;

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

    class DriveStraight : public virtual IMotion
    {
    protected:
        bool isAsync();
        std::string getName();

    private:
        double target_distance;

        std::shared_ptr<PID> drive_pid;
        std::shared_ptr<PID> turn_pid;
        DriveParameters params;

        double time_correct = 0;
        Pose starting_position = Pose::none();

        std::string name;

    public:
        DriveStraight(double target_distance, std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid, DriveParameters params = {}, std::string name = "drive");
        void initialize(Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };

    /**
     * @brief The parameters for the turning functions
     */
    struct FaceParameters
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
         * @brief The time the error needs to be within the threshold for in order to exit
         */
        double threshold_time = 40;

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
         * @brief Whether to spin the long way or the short way to get to a target
         */
        bool long_direction = false;

        /**
         * @brief Whether to use the chain exit condition instead of the normal one
         */
        bool chained = false;
        /**
         * @brief Whether to run asynchronously
         */
        bool async = false;
    };

    struct FaceAngleTarget
    {
        lib15442c::Angle angle;
    };

    struct FacePointTarget
    {
        lib15442c::Pose pos;
        lib15442c::Angle angle_offset;
    };

    using FaceTarget = std::variant<FaceAngleTarget, FacePointTarget>;

    class Face : public virtual IMotion
    {
    protected:
        bool isAsync();
        std::string getName();

    private:
        FaceTarget target;

        std::shared_ptr<PID> pid;
        FaceParameters params;

        double time_correct = 0;
        Angle initial_error = Angle::none();

        std::string name;

        static Angle getTargetAngle(FaceTarget target, Pose pose);

    public:
        Face(FaceTarget target_distance, std::shared_ptr<PID> pid, FaceParameters params = {}, std::string name = "face");
        void initialize(Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };
}