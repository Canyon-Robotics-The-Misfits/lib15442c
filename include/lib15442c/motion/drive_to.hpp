#pragma once

#include "motion.hpp"
#include "end_condition.hpp"

namespace lib15442c
{
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
        double angle_priority_threshold = 3;

        /**
         * @brief The timeout in case the move takes too long (ms)
         */
        double timeout = 2500;

        /**
         * @brief The maximum speed
         */
        double max_speed = 127;
        /**
         * @brief The minimum speed, defaults to the one set by the DriveController constructor
         */
        double min_speed = -1;

        /**
         * @brief An extra end condition for special cases
         */
        EndCondition end_condition = default_end_condition;

        /**
         * @brief Whether to use the chain exit condition instead of the normal one
         */
        bool chained = false;
        /**
         * @brief Whether to run asynchronously
         */
        bool async = false;
        
        /**
         * @brief How far away the robot should be from the end before the the chain exit condition can succeed
         */
        double chain_threshold = 5;
    };

    class Boomerang: public IMotion
    {
    protected:
        bool isAsync();
        std::string getName();

    private:
        Pose target_pose;

        std::shared_ptr<PID> drive_pid;
        std::shared_ptr<PID> turn_pid;
        BoomerangParameters params;

        bool initial_above_approach_line = false;

        std::string name;

    public:
        Boomerang(Pose target_pose, std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid, BoomerangParameters params = {}, std::string name = "boomerang");
        void initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };
    
    struct DriveToABParameters
    {
        /**
         * @brief weight of facing the target point
         */
        double kp_alpha = 0.6;
        /**
         * @brief weight of ending at the target angle
         */
        double kp_beta = 0.6;


        /**
         * @brief Whether to go in reverse
         */
        bool backwards = false;
        /**
         * @brief How far away the robot should be from the end before the movement ends
         */
        double threshold = 1;

        /**
         * @brief The timeout in case the move takes too long (ms)
         */
        double timeout = 2000;

        /**
         * @brief The maximum speed
         */
        double max_speed = 127;
        /**
         * @brief The minimum speed, defaults to the one set by the DriveController constructor
         */
        double min_speed = -1;

        /**
         * @brief An extra end condition for special cases
         */
        EndCondition end_condition = default_end_condition;

        /**
         * @brief Whether to run asynchronously
         */
        bool async = false; 
    };

    class DriveToAB: public IMotion
    {
    protected:
        bool isAsync();
        std::string getName();

    private:
        Pose target_pose;

        std::shared_ptr<PID> drive_pid;
        DriveToABParameters params;

        std::string name;

    public:
        DriveToAB(Pose target_pose, std::shared_ptr<PID> drive_pid, DriveToABParameters params = {}, std::string name = "drive to AB");
        void initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };
    
    struct DriveToIntermediateParameters
    {
        /**
         * @brief The threshold in which the algorithm should just try to stop at the right angle
         */
        double settle_threshold = 5;
        /**
         * @brief How far away to make the intermediate angle
         */
        double r = 12;


        /**
         * @brief Whether to go in reverse
         */
        bool backwards = false;
        /**
         * @brief How far away the robot should be from the end before the movement ends
         */
        double threshold = 0.75;

        /**
         * @brief The timeout in case the move takes too long (ms)
         */
        double timeout = 2500;

        /**
         * @brief The maximum speed
         */
        double max_speed = 127;
        /**
         * @brief The minimum speed, defaults to the one set by the DriveController constructor
         */
        double min_speed = -1;

        /**
         * @brief An extra end condition for special cases
         */
        EndCondition end_condition = default_end_condition;

        /**
         * @brief Whether to run asynchronously
         */
        bool async = false; 
    };

    class DriveToIntermediate: public IMotion
    {
    protected:
        bool isAsync();
        std::string getName();

    private:
        Pose target_pose;

        std::shared_ptr<PID> drive_pid;
        std::shared_ptr<PID> turn_pid;
        DriveToIntermediateParameters params;

        std::string name;

    public:
        DriveToIntermediate(Pose target_pose, std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid, DriveToIntermediateParameters params = {}, std::string name = "Drive To Intermediate");
        void initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };
} // namespace lib15442c
