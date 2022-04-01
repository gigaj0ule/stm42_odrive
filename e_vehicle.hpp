#ifndef __E_VEHICLE_HPP
#define __E_VEHICLE_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class E_Vehicle {
public:
    enum Error_t : int32_t {
        ERROR_NONE = 0,
        ERROR_FEATURE_UNIMPLEMENTED = 0x01,
        ERROR_MODE_NOT_ENABLED      = 0x02 
    };

    Error_t error_ = ERROR_NONE;

    enum DriveMode_t : int32_t {
        DRIVE_MODE_UNDEFINED = 0,
        DRIVE_MODE_PARK = 1,
        DRIVE_MODE_NEUTRAL = 2,
        DRIVE_MODE_FREEWHEEL = 3,
        DRIVE_MODE_DRIVE = 4,
        DRIVE_MODE_REVERSE = 5,       
    };

    enum DriveConfig_t : int32_t {
        DRIVE_CONFIG_1WD = 0,
        DRIVE_CONFIG_2WD_INLINE = 1,
        DRIVE_CONFIG_2WD_AXEL = 2,
        DRIVE_CONFIG_4WD_AXEL = 3
    };

    enum MasterAxis_t : int32_t {
        MASTER_AXIS_0 = 0,
        MASTER_AXIS_1 = 1
    };

    struct ControllerParams_t{
        float drive_vel_gain                = 0.05f;
        float drive_vel_integrator_gain     = 0.1f;
        float drive_vel_derivative_gain     = 0.0f;

        float park_vel_gain                 = 0.11f;
        float park_vel_integrator_gain      = 1.5f;
        float park_vel_derivative_gain      = 0.0f;
        float park_current_limit            = 40.0f;

        float throttle_current_limit        = 50.0f;
        float brake_current_limit           = 40.0f;
        float regen_deceleration_current    = 1.0f;
        float max_current_limit             = 60.0f;
    };

    struct PhysicsParams_t
    {
        DriveConfig_t drive_config  = DRIVE_CONFIG_1WD;
        MasterAxis_t master_axis    = MASTER_AXIS_0;
        float axle_width            = 1.0f;
    };

    struct Config_t {
        ControllerParams_t controller_params;
        PhysicsParams_t physics;

        bool ev_controller_enabled = 0;
        bool abs_enabled = 0;

        float gas_pedal_adc_min_voltage = 0.05f;
        float gas_pedal_adc_max_voltage = 1.0f;

        float brake_pedal_adc_min_voltage = 0.05f;
        float brake_pedal_adc_max_voltage = 1.0f;

        osPriority thread_priority = osPriorityAboveNormal;
    };

    Config_t& config_;

    struct ParkingBrake_t
    {
        bool engaged = false;
        bool last_state = false;
        DriveMode_t cached_drive_mode  = DRIVE_MODE_UNDEFINED;
    };
    
    struct CruiseControl_t 
    {
        bool engaged = false;
        float setpoint = 0.0f;
        float gas_pedal_cached_value = 0.0f;
    };

    struct States_t
    {
        DriveMode_t drive_mode = DRIVE_MODE_DRIVE;
        DriveMode_t last_drive_mode = DRIVE_MODE_UNDEFINED;

        float gas_pedal_value = 0;

        float brake_pedal_value = 0;
        bool  brake_locked = 0;
        bool  brake_locked_last_state = 0;

        ParkingBrake_t parking_brake;
        CruiseControl_t cruise_control;
        
        bool configured_gpio_pins_for_ev = false;

        uint8_t number_of_active_axes   = 0;
        uint8_t master_axis             = 0;
        uint8_t subordinate_axis        = 0;
    };
        
    States_t states_;

    E_Vehicle(Config_t& config);

    bool update(void);

    void toggle_state_cruise_control(bool start_debounce_timer);
    void toggle_state_parking_brake(bool start_debounce_timer);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_number("error", &error_),
            make_protocol_number("drive_mode", &states_.drive_mode),
            make_protocol_number("gas_pedal_value", &states_.gas_pedal_value),
            make_protocol_number("brake_pedal_value", &states_.brake_pedal_value),
            make_protocol_number("parking_brake_engaged", &states_.parking_brake.engaged),
            make_protocol_number("cruise_control_engaged", &states_.cruise_control.engaged),
            make_protocol_number("cruise_control_setpoint", &states_.cruise_control.setpoint),
            make_protocol_object("config",
                make_protocol_number("ev_controller_enabled", &config_.ev_controller_enabled),
                make_protocol_number("abs_enabled", &config_.abs_enabled),
                make_protocol_number("gas_adc_min_voltage", &config_.gas_pedal_adc_min_voltage),
                make_protocol_number("gas_adc_max_voltage", &config_.gas_pedal_adc_max_voltage),
                make_protocol_number("brake_adc_min_voltage", &config_.brake_pedal_adc_min_voltage),
                make_protocol_number("brake_adc_max_voltage", &config_.brake_pedal_adc_max_voltage),
                make_protocol_object("physics", 
                    make_protocol_number("drive_config", &config_.physics.drive_config),
                    make_protocol_number("master_axis", &config_.physics.master_axis),
                    make_protocol_number("axle_width", &config_.physics.axle_width)
                ),
                make_protocol_object("controller",
                    make_protocol_number("max_current_limit", &config_.controller_params.max_current_limit),
                    make_protocol_number("throttle_current_limit", &config_.controller_params.throttle_current_limit),
                    make_protocol_number("brake_current_limit", &config_.controller_params.brake_current_limit),
                    make_protocol_number("regen_deceleration_current", &config_.controller_params.regen_deceleration_current),
                    make_protocol_number("park_current_limit", &config_.controller_params.park_current_limit),

                    make_protocol_number("park_vel_gain", &config_.controller_params.park_vel_gain),
                    make_protocol_number("park_vel_integrator_gain", &config_.controller_params.park_vel_integrator_gain),
                    make_protocol_number("park_vel_derivative_gain", &config_.controller_params.park_vel_derivative_gain),
                    make_protocol_number("drive_vel_gain", &config_.controller_params.drive_vel_gain),
                    make_protocol_number("drive_vel_integrator_gain", &config_.controller_params.drive_vel_integrator_gain),
                    make_protocol_number("drive_vel_derivative_gain", &config_.controller_params.drive_vel_derivative_gain)
                )
            )
        );
    };

    osThreadId thread_id_;
    volatile bool thread_id_valid_ = false;
    void start_thread();

private:
    
    void configure_gpio(bool initialize_gpio);
    void configure_motors(DriveMode_t drive_mode);
    void read_gpio(void);  

    TimerHandle_t debounce_timers[5] = { NULL };
};

DEFINE_ENUM_FLAG_OPERATORS(E_Vehicle::Error_t)

extern E_Vehicle *ev;

#endif
