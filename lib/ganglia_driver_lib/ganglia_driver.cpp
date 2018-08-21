/******************************
 *  [Header] Ganglia Driver Board Serial Connector & Controller
 *  Author : Thiruch Rakthao
 *  Date : 9-Feb-2018
 *******************************/

#include "obo_ganglia_driver_lib/ganglia_driver.h"

namespace ganglia
{
// Constructor
GangliaDriver::GangliaDriver():
    is_connected_(false),
    motor_id_(0),
    is_read_encoder_abs_(false),
    is_set_bldc_(false),
    is_motor_enable_(false),
{
    std::cout << "[Ganglia] ganglia_driver : Class Initialized" << std::endl;
    is_connected_ = false;
}

GangliaDriver::~GangliaDriver()
{
}

bool GangliaDriver::init(std::string port_name, uint32_t baud_rate)
{
    modbus_ = modbus_new_ascii(port_name.c_str(), baud_rate, 'N', 8, 1);
    return true;
}

void GangliaDriver::setId(int motor_id)
{
    motor_id_ = motor_id;
    modbus_->setSlaveAddress(address_);
}

void GangliaDriver::setReadEncoderAbs(bool is_read)
{
    is_read_encoder_abs_ = is_read;
}

void GangliaDriver::setMotorBLDC(bool status)
{
    is_set_bldc_ = status;
}

void GangliaDriver::setMotorGearRatio(uint16_t denominator, uint32_t numerator)
{
    uint64_t value = denominator | (numerator << 16);
    if(setMotorGearRatio(value) < 0)
    {
        // ROS_ERROR("[Gear Ratio]Invalid Receive Discard !");
    }
}

bool GangliaDriver::connect()
{
    if(!serial_.isOpen())
    {
        serial_.open();
    }
    std::cout << "GangliaDriver::connect" << std::endl;
    if(is_set_bldc_)
        if( setMotorBLDC() < 0 )
            return is_connected_ = false;

    if( setVelocityMode() < 0 )
        return is_connected_ = false;

    if( setMotorEnable(true) < 0 )
        return is_connected_ = false;

    if( setMotorStart(true) < 0 )
        return is_connected_ = false;

    setVelCmd(0);

    return is_connected_ = true;
}

bool GangliaDriver::disconnect()
{
    // std::cout << "GangliaDriver::disconnect : " << address_ << std::endl;
    is_connected_ = false;
    serial_.close();
    setVelCmd(0);
    setMotorStart(false);
    setMotorEnable(false);
}

bool GangliaDriver::isConnected()
{
    return is_connected_;
}

int GangliaDriver::getGangliaState(GangliaState &data)
{
    uint16_t dest[6];
    if( modbus_->readInputRegisters(0x0000, 6, dest) < 0 )
        return -1;

    data.id        = address_;
    data.encoder   = dest[0] | (dest[1] << 16);
    data.vel       = dest[2] | (dest[3] << 16);
    data.current   = dest[4] | (dest[5] << 16);

    if( is_read_encoder_abs_ )
    {
        if( modbus_->readInputRegisters(0x0021, 1, dest) < 0 )
            return -1;
        data.encoder_abs   = dest[0];
    }
    else
        data.encoder_abs = 0;

    return 0;
}

int GangliaDriver::setVelCmd(int rpm_value)
{
    if( modbus_->writeMultipleRegisters(0x0018, 2, (uint16_t*)(&rpm_value)) < 0 )
        return -1;

    return 0;
}

int GangliaDriver::motorLoop(int rpm_value, GangliaState &data)
{
    uint16_t dest[6];

    if( modbus_->writeAndReadRegisters(0x0018, 2, (uint16_t*)(&rpm_value), 0x0000, 6, dest) < 0 )
        return -1;

    data.motor_id_        = address_;
    data.encoder_   = dest[0] | (dest[1] << 16);
    data.vel_       = dest[2] | (dest[3] << 16);
    data.current_   = dest[4] | (dest[5] << 16);

    if( is_read_encoder_abs_ )
    {
        if( modbus_->readInputRegisters(0x0021, 1, dest) < 0 )
            return -1;
        data.encoder_abs_   = dest[0];
    }
    else
        data.encoder_abs_ = 0;

    return 0;
}

int32_t GangliaDriver::getMotorPosition()
{
    uint16_t dest[2];
    if( modbus_->readInputRegisters(0x0000, 2, dest) < 0 )
        return -1;
    return dest[0] | (dest[1] << 16);
}



int GangliaDriver::setMotorBLDC()
{
    std::cout << "GangliaDriver::setMotorBLDC(" << address_ << ")" << std::endl;
    uint8_t src[1] = {0x05};
    if( modbus_->writeMultipleCoils(0x0008, 3, src) < 0 )
    {
        ROS_WARN("[Motor-BLDC]Invalid Receive Discard !");
        return -1;
    }

    return 0;
}

int GangliaDriver::setVelocityMode()
{
    std::cout << "GangliaDriver::setVelocityMode(" << address_ << ")" << std::endl;
    // int writeSingleRegister(int addr, int value);
    if( modbus_->writeSingleRegister(0x0080, 0xFFFE) < 0 )
    {
        ROS_WARN("[Vel-Mode]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaDriver::setMotorEnable(bool is_enable)
{
    std::cout << "GangliaDriver::setMotorEnable(" << address_ << ") : " << is_enable << std::endl;
    // int enable = ( is_enable ) ? 0xFF00 : 0x0000;
    if( modbus_->writeSingleCoil(0x000D, is_enable) < 0 )
    {
        ROS_WARN("[Motor-Enable]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaDriver::setMotorStart(bool is_start)
{
    std::cout << "GangliaDriver::setMotorStart(" << address_ << ") : " << is_start << std::endl;
    int enable = ( is_start ) ? 0xFF00 : 0x0000;
    if( modbus_->writeSingleCoil(0x0003, enable) < 0 )
    {
        ROS_WARN("[Motor-Start]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaDriver::setMotorGearRatio(uint64_t value)
{
    std::cout << "GangliaDriver::setMotorGearRatio(" << address_ << ")  "  << std::endl;
    if( modbus_->writeMultipleRegisters(0x0069, 3, (uint16_t*)(&value)) < 0 )
        return -1;
}
}
