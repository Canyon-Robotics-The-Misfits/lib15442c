#pragma once

#include "motion.hpp"
#include "end_condition.hpp"

#include "lib15442c/trajectory/trajectory.hpp"

namespace lib15442c
{
    class RAMSETE: public IMotion
    {
    protected:
        bool is_async();
        std::string get_name();

    private:
        Trajectory trajectory;

        std::string name;

    public:
        RAMSETE(Trajectory trajectory, std::string name = "RAMSETE");
        void initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };
} // namespace lib15442c
