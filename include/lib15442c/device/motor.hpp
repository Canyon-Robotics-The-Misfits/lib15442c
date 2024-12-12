#pragma once

#include <vector>
#include <memory>

namespace lib15442c
{
    enum class MotorBrakeMode {
        COAST, // Don't brake
        BRAKE, // Brake
        HOLD   // Brake and try to return to original stopping position
    };

    constexpr double MOTOR_RED = 1.0 / 36.0; // Gear ratio of a red cartridge
    constexpr double MOTOR_GREEN = 1.0 / 18.0; // Gear ratio of a green cartridge
    constexpr double MOTOR_BLUE = 1.0 / 6.0; // Gear ratio of a blue cartridge

    /**
     * An abstract motor
     */
    class IMotor {
    public:
        /**
         * @brief Move the motor with a set voltage from -127 to 127
         * 
         * -127 means max voltage reverse, 127 is max forwards. 127 is not the real motor voltage
         * 
         * @param voltage The target voltage
         */
        virtual void move(double voltage) = 0;
        /**
         * @brief Move the motor at a set velocity
         * 
         * @param velocity The target velocity
         */
        virtual void move_velocity(double velocity) = 0;

        /**
         * @brief Get the motor velocity
         * 
         * @return double the motor velocity
         */
        virtual double get_velocity() = 0;

        /**
         * @brief Set the brake mode of the motor
         * 
         * @param brake_mode The new brake mode
         */
        virtual void set_brake_mode(MotorBrakeMode brake_mode) = 0;
        /**
         * @brief Get the brake mode of the motor
         * 
         * @return MotorBrakeMode 
         */
        virtual MotorBrakeMode get_brake_mode() = 0;

        /**
         * @brief Reverse or unreverse the motor
         * 
         * @param reversed Whether the motor should be reversed
         */
        virtual void set_reversed(bool reversed) = 0;
        /**
         * @brief Get whether the motor is reversed
         * 
         * @return bool 
         */
        virtual bool get_reversed() = 0;

        /**
         * @brief Get the rough motor tempature in celsius
         * 
         * @return double The tempature
         */
        virtual double get_temp() = 0;

        /**
         * @brief Set the gear ratio on the motor
         * 
         * @param ratio The new ratio
         */
        virtual void set_ratio(double ratio) = 0;
        /**
         * @brief Get the gear ratio of the motor
         * 
         * @return double The gear ratio
         */
        virtual double get_ratio() = 0;

        /**
         * @brief Get whether the motor is installed or not
         * 
         * @return bool 
         */
        virtual bool is_installed() = 0;

        /**
         * @brief Get a list of the uninstalled motors by port. Returns the port of the motor if not a motor group
         * 
         * @return std::vector<int> 
         */
        virtual std::vector<int> get_uninstalled_motors() = 0;
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

        double get_velocity() override;

        void set_brake_mode(MotorBrakeMode brake_mode) override;
        MotorBrakeMode get_brake_mode() override;

        void set_reversed(bool reversed) override;
        bool get_reversed() override;

        double get_temp() override;

        void set_ratio(double ratio) override;
        double get_ratio() override;

        bool is_installed();

        int get_port();

        std::vector<int> get_uninstalled_motors() override;
    };

    class MotorGroup : public virtual IMotor {
        std::vector<std::unique_ptr<Motor>> motors;

    public:
        MotorGroup(std::initializer_list<MotorParameters> parameters);
        MotorGroup(MotorGroupParameters parameters, std::initializer_list<int> ports);

        void move(double voltage) override;
        void move_velocity(double velocity) override;

        double get_velocity() override;

        void set_brake_mode(MotorBrakeMode brake_mode) override;
        MotorBrakeMode get_brake_mode() override;

        void set_reversed(bool reversed) override;
        bool get_reversed() override;

        double get_temp() override;

        void set_ratio(double ratio) override;
        double get_ratio() override;

        bool is_installed() override;

        /**
         * @brief Get a list of the uninstalled motors by port
         * 
         * @return std::vector<int> 
         */
        std::vector<int> get_uninstalled_motors() override;
    };

    #endif
} // namespace lib15442c
