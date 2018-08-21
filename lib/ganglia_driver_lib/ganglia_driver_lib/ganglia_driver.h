/******************************
*  [Header] Ganglia Driver Board Serial Connector & Controller
*  Author : Thiruch Rakthao
*  Date : 9-Feb-2018
*******************************/
#ifndef _GANGLIA_DRIVER_LIB_H_
#define _GANGLIA_DRIVER_LIB_H_

// #include <stdint.h>
// #include <string>
// #include <iostream>

#include <ros/ros.h>

#include "../../3rdparty/libmodbus/modbus.h"
// #include "structs.h"

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

class GangliaDriver
{
public:
    enum GangliaStateName { ERROR = 0, NORMAL };

    enum GangliaMode { READ=0, READWRITE };

    // Constructor
    GangliaDriver();
    ~GangliaDriver();

    bool    init(std::string port_name, uint32_t baud_rate);
    bool    setId(int motor_id);
    bool    setTimeout(uint32_t byte_timeout_sec, uint32_t byte_timeout_usec, uint32_t response_timeout_sec, uint32_t response_timeout_usec);
    bool    setReadEncoderAbs(bool status);
    bool    setMotorBLDC(bool status);
    bool    setMotorEnable(bool status);
    bool    setMotorMode(uint8_t mode);

    //Connection Function
    bool    connect();                    // Connect to Driver board via Serial
    bool    disconnect();
    bool    isConnected();

    //Profile Function
    bool setProfile();

    //Action Function
    bool start();
    bool stop();
    bool setVelCmd(int value);

    // Read status
    bool         getGangliaState(GangliaState &data);
    // bool         motorLoop(int rpm_value, GangliaState &data);
    bool         getMotorPosition(int32_t &value);

private:
    bool  setMotorId(int idx);

    int   setMotorBLDC();
    int   setVelocityMode();
    int   setMotorEnable();
    int   setMotorStart(bool status);

public:

private:
    modbus_t        *modbus_;    // modbus class from modbus lib
    bool            is_connected_;       // Flag for Connection Status
    
    uint32_t        id_;
    bool            is_read_encoder_abs_;
    bool            is_set_bldc_; // Flag for set motor BLDC mode
    bool            is_motor_enable_;

};
}

#endif // _GANGLIA_DRIVER_LIB_H_