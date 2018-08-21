#ifndef _GANGLIA_MULTI_DRIVER_H_
#define _GANGLIA_MULTI_DRIVER_H_

#include <vector>
// #include <stdint.h>
//#include <ros/ros.h>

#include "../libmodbus/modbus.h"
#include "ganglia_driver_lib/ganglia_info.h"
// #include "obo_ganglia_driver_lib/ganglia_driver.h"

#undef ERROR

namespace ganglia{

typedef struct GangliaState_
{
    uint8_t id;
    uint8_t model_id;
    int8_t  state;
    int32_t encoder;
    int16_t encoder_abs;
    int32_t vel;
    int32_t current;

    friend std::ostream& operator<< (std::ostream& os, const GangliaState_& data)
    {
        return os << data.encoder << " " << data.encoder_abs << " " << data.vel << " " << data.current;
    }
}GangliaState;

typedef struct GangliaStateArray_
{
    std::vector<ganglia::GangliaState> data;
}GangliaStateArray;

class GangliaMultiDriver
{
public:
    enum GangliaStateName { ERROR = 0, NORMAL };

    enum ControlMode { Current, Velocity, Position };

    GangliaMultiDriver(int motor_count);
    ~GangliaMultiDriver();

    bool init(std::string port_name, uint32_t baud_rate);
    bool setId(std::vector<uint32_t> motor_id);
    bool setTimeout(uint32_t byte_timeout_sec, uint32_t byte_timeout_usec, uint32_t response_timeout_sec, uint32_t response_timeout_usec);

    //Connection Function
    bool connect();
    bool disconnect();
    bool isConnected();

    //Profile Config Function
    bool setReadEncoderAbs(std::vector<bool> status);
    bool setMotorBLDC(std::vector<bool> status);
    bool setMotorEnable(std::vector<bool> status);
    bool setMotorControlMode(std::vector<int> status);

    //Profile Action Function
    bool setProfile();
    bool setMotorBLDC();
    bool setCurrentControl(uint32_t id);
    bool setAllCurrentControl();
    bool setVelocityControl(uint32_t id);
    bool setAllVelocityControl();
    bool setPositionControl(uint32_t id);
    bool setAllPositionControl();
    // bool setMotorEnable();

    //Action Function
    bool startMotor(uint32_t id);
    bool startAllMotor();
    bool stopMotor(uint32_t id);
    bool stopAllMotor();
    bool setCurrent(uint32_t id, int value);
    bool setAllCurrent(std::vector<int> value);
    bool setVelocity(uint32_t id, int value);
    bool setAllVelocity(std::vector<int> value);
    bool setPosition(uint32_t id, int value);
    bool setAllPosition(std::vector<int> value);


    //Read status
    bool getGangliaState(size_t id, GangliaState &data);
    bool getAllGangliaState(GangliaStateArray &data);
    // std::vector<bool> getMotorPosition(std::vector<int32_t> &value);

private:
    bool  setMotorId(int idx);

    int   setMotorBLDC(int id);
    int   setMotorControlMode(int id);
    int   setCurrentMode(int id);
    int   setVelocityMode(int id);
    int   setPositionMode(int id);
    int   setMotorEnable(int id, bool is_enable);
    int   setMotorStart(int id, bool is_start);

    bool  breakAllMotor();

    void initStateArray();

public:
    GangliaStateArray  state_;

private:
    modbus_t        *modbus_;
    bool            is_connected_;

    uint32_t        motor_count_;

    SerialInfo                  serial_info_;
    // std::vector<GangliaInfo>    info_;


    std::vector<uint32_t>       motor_id_;
    std::vector<bool>           is_read_encoder_abs_;
    std::vector<bool>           is_set_bldc_;
    std::vector<bool>           is_motor_enable_;
    std::vector<ControlMode>    motor_control_mode_;

};
}

#endif //_GANGLIA_MULTI_DRIVER_H_
