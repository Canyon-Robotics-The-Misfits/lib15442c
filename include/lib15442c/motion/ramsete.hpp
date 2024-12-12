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
        lib15442c::Trajectory trajectory;

        double track_width = 0;

        double b;
        double zeta;

        bool async;
        std::string name;

    public:
        RAMSETE(lib15442c::Trajectory trajectory, double b, double zeta, bool async = false, std::string name = "RAMSETE");
        void initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose);

        MotionOutput calculate(Pose pose, double time_since_start, double delta_time);
    };
} // namespace lib15442c
