#pragma once

#include <vector>
#include <memory>

namespace lib15442c
{
    enum class MotorBrakeMode {
        COAST,
        BRAKE,
        HOLD
    };

    constexpr double MOTOR_RED = 1.0 / 36.0;
    constexpr double MOTOR_GREEN = 1.0 / 18.0;
    constexpr double MOTOR_BLUE = 1.0 / 6.0;

    class IMotor {
    public:
        virtual void move(double voltage) = 0;
        virtual void move_velocity(double speed) = 0;

        virtual void set_brake_mode(MotorBrakeMode brake_mode) = 0;
        virtual MotorBrakeMode get_brake_mode() = 0;

        virtual void set_reversed(bool reversed) = 0;
        virtual bool get_reversed() = 0;

        virtual double get_temp() = 0;

        virtual void set_ratio(double ratio) = 0;
        virtual double get_ratio() = 0;

        virtual bool is_installed() = 0;
    };

    #ifndef LIB15442C_MOCK_DEVICES_ONLY

    struct MotorGroupParameters {
        bool reversed = false;
        MotorBrakeMode brake_mode = MotorBrakeMode::COAST;
        double ratio = MOTOR_GREEN;
    };

    struct MotorParameters : public MotorGroupParameters {
        int port;
        bool reversed = false;
        MotorBrakeMode brake_mode = MotorBrakeMode::COAST;
        double ratio = MOTOR_GREEN;
    };

    class Motor : public virtual IMotor {
        int port;
        bool reversed;
        MotorBrakeMode brake_mode;
        double ratio;

    public:
        Motor(MotorParameters parameters);

        void move(double voltage) override;
        void move_velocity(double velocity) override;

        void set_brake_mode(MotorBrakeMode brake_mode) override;
        MotorBrakeMode get_brake_mode() override;

        void set_reversed(bool reversed) override;
        bool get_reversed() override;

        double get_temp() override;

        void set_ratio(double ratio) override;
        double get_ratio() override;

        bool is_installed();

        int get_port();
    };

    class MotorGroup : public virtual IMotor {
        std::vector<std::unique_ptr<Motor>> motors;

    public:
        MotorGroup(std::initializer_list<MotorParameters> parameters);
        MotorGroup(MotorGroupParameters parameters, std::initializer_list<int> ports);

        void move(double voltage);
        void move_velocity(double velocity);

        void set_brake_mode(MotorBrakeMode brake_mode);
        MotorBrakeMode get_brake_mode();

        void set_reversed(bool reversed);
        bool get_reversed();

        double get_temp();

        void set_ratio(double ratio);
        double get_ratio();

        bool is_installed();
        std::vector<int> get_uninstalled_motors();
    };

    #endif
} // namespace lib15442c
