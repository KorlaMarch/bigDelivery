#ifndef REGISTERS_STRUCTS_H_
#define REGISTERS_STRUCTS_H_

#include <iostream>

typedef struct {
    uint8_t position_profile_mask;
    uint8_t velocity_profile_mask;
    uint8_t homing_mask;
    uint8_t control_enable;
    uint8_t position_mask;
    uint8_t velocity_mask;
    uint8_t current_mask;
    uint8_t reserved[1];
} ControlMasks;

typedef struct {
    uint8_t Error0;        // attempt to write eeprom and eeprom bank offset error
    uint8_t Error1;
    uint8_t Error2;
    uint8_t Error3;
    uint8_t Error4;
    uint8_t Error5;
    uint8_t Error6;
    uint8_t Error7;
} DigitalInput;


typedef struct{
    uint8_t BLDC_mode;
    uint8_t Sensorless_mode;
    uint8_t BLDC_QDEC;
    uint8_t reserved_0[1];
    uint8_t QDEC_velocity;
    uint8_t drive_enable;
    uint8_t EEPROM_write;
    uint8_t EEPROM_read;
    uint8_t load_default;
    uint8_t test_pwm;
    uint8_t can_over_modbus;
    uint8_t inverted_encoder;
    uint8_t reserved[12];           // 1 Page end
}GeneralMode;

typedef struct {
    ControlMasks mask_M0;
    GeneralMode gen_mode;
} DigitalOutput;

typedef struct {
    int32_t position;               // (actual_position)
    int32_t velocity;               // (actual_velocity)
    int32_t current;                // (actual_current)
    int32_t position_error;         // (following_error)
    int32_t velocity_error;
    int32_t current_error;
    int32_t position_int;
    int32_t velocity_int;
    int32_t current_int;
    int32_t output;                 // (pwm value)
    int32_t result_position;
    int32_t result_velocity;
    int32_t result_current;
    int32_t demand_position;
    int32_t demand_velocity;
    int32_t demand_current;
} MotorStat;

typedef struct {
    int16_t temperature;
    int16_t poten;
    int32_t profile_position_start_point;
    uint32_t profile_position_step_count;
    uint16_t hall_sensor_pattern;
    int16_t reserved0[1];
    uint32_t inc_enc_cnt;
    int32_t current_PhA;
    int32_t current_PhB;
    int32_t current_PhC;
} MotorExStat;

typedef struct {
    uint16_t node_id;
    uint16_t can_bitrate;
    uint32_t mb_baudrate;
    uint32_t reserved[6];
} NodeSetting;

typedef struct {
    uint16_t cpu_temp;
    uint16_t supply_voltage;
    uint16_t min_pwm_hz;            // (bound to ADC & ACC sampling windows)
    uint16_t control_hz;
    uint16_t pwm_period;
    uint16_t pwm_kHz;
    uint32_t cpu_Hz;                // fixed
    uint16_t adc_kHz;               // fixed
    int16_t operation_mode_display;
    uint16_t status_word;
    uint16_t reserved[5];           // padding byte
}GeneralStatus;

typedef struct {
    MotorStat M0_stat;              // 2 Pages
    MotorExStat M0_ex_stat;         // 1 Page

    GeneralStatus gen_stat;         // 1 Page

    NodeSetting node_setting;       // 1 Page
} AnalogInput;

typedef struct {
    int16_t gain_P;
    int16_t gain_I;
    int16_t gain_D;
    int16_t reserved0[1];
    uint32_t limit_I;
    uint32_t reserved1[1];
} PIDSetting;

typedef struct {
    PIDSetting pid_setting;
    int32_t target;
    int32_t limit_minimal;
    int32_t limit_maximal;
    int16_t scaling_bit;
    int16_t reserved[1];
} ControlSetting;                 // 1 Page

typedef struct {
    uint16_t velocity_feedforward;
    uint16_t acceleration_feedforward;
    uint32_t reserved[3];
    uint32_t maximal_profile_velocity;
    uint32_t profile_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
} ProfileSetting;                 // 1 Page

typedef struct {
    uint32_t quick_stop_deceleration;
    int32_t profile_position_target;
    int32_t profile_velocity_target;
    uint32_t reserved[5];
} ProfileExSetting;                 // 1 Page

typedef struct {
    int16_t pwm_value;
    uint16_t pwm_limit;
    uint32_t max_acceleration;
    uint32_t homing_acceleration;
    uint16_t current_threshold_for_homing_mode;
    uint16_t reserved[1];
    uint32_t speed_for_switch_search;
    uint32_t speed_for_zero_search;
    uint32_t max_following_error;
    uint16_t deadband_plus;
    uint16_t deadband_minus;
} LimitSetting;                   // 1 Page

typedef struct {
    uint16_t motor_type;
    uint16_t motor_size;
    uint16_t motor_continuous_current_limit;
    uint16_t motor_current_limit;
    uint16_t motor_pole_pair;
    uint16_t motor_slot;
    uint16_t reserved0[1];
    uint16_t motor_max_speed;
    uint16_t motor_thermal_time_constant;
    uint16_t gear_ratio_denominator;
    uint32_t gear_ratio_numerator;
    uint32_t gear_maximal_speed;
    uint16_t velocity_average_order;
    uint16_t current_average_order;
} MotorSetting;

typedef struct {
    uint32_t encoder_pulse_number;
    uint32_t reserved[7];
} EncoderSetting;

typedef struct {
    int16_t operation_mode;
    uint16_t bldc_lock_current;
    uint16_t control_word;
    uint16_t reserved[13];
} OtherSetting;

typedef struct {
    ControlSetting position_setting;      // 1 Page
    ControlSetting velocity_setting;      // 1 Page
    ControlSetting current_setting;       // 1 Page
    ProfileSetting profile_setting;       // 1 Page
    ProfileExSetting profile_ex_setting;  // 1 Page
    LimitSetting limit_setting;           // 1 Page
    MotorSetting motor_setting;           // 1 Page
    EncoderSetting encoder_setting;       // 1 Page
    OtherSetting other_setting;           // 1 Page
} MotorControl;

typedef struct {
    uint16_t control_Hz_set;
    uint16_t PWM_period_set;
    uint16_t PWM_kHz_set;
    uint16_t eeprom_bank;                 // 32 Pages per bank
    uint16_t supply_range;
    uint16_t reserved[11];
} GeneralSetting;


typedef struct {
    MotorControl M0_control;              // 9 Pages : 288 bytes
    GeneralSetting general_setting;       // 1 Page
    NodeSetting node_setting;             // 1 Page
} AnalogOutput;

#endif