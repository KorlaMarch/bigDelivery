#include <iostream>

struct SerialInfo
{
    std::string     port_name_;
    uint32_t        baud_rate_;
};

struct GangliaInfo
{
    uint32_t    motor_id;
    bool        is_read_encoder_abs_;
    bool        is_set_bldc_;
    bool        is_motor_enable_;
    int         motor_control_mode_;
};