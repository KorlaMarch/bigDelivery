#include "ganglia_driver_lib/ganglia_multi_driver.h"
#include <stdlib.h>
#include <stdio.h>

namespace ganglia
{

GangliaMultiDriver::GangliaMultiDriver(int motor_count):
    is_connected_(false)
{
    std::cout << "[Ganglia] ganglia_multi_driver : Class Initialized" << std::endl;
    motor_count_ = motor_count;
    motor_id_.resize(motor_count, -1);
    // is_read_encoder_abs_.resize(motor_count);
    // is_set_bldc_.resize(motor_count);
    // is_motor_enable_.resize(motor_count);
    // motor_control_mode_.resize(motor_count);

    //Init GangliaStateArray
    initStateArray();
}

GangliaMultiDriver::~GangliaMultiDriver()
{
}

bool GangliaMultiDriver::init(std::string port_name, uint32_t baud_rate)
{
    //ROS_INFO("[Ganglia] ganglia_multi_driver: set serial %s", port_name.c_str());
	std::cout << port_name.c_str() << std::endl;
    modbus_ = modbus_new_ascii(port_name.c_str(), baud_rate, 'N', 8, 1);
	//modbus_set_debug(modbus_, true);
    // port_name_ = port_name;
    // baud_rate_ = baud_rate;
    return true;
}

bool GangliaMultiDriver::setId(std::vector<uint32_t> motor_id)
{
    if(motor_count_ == motor_id.size())
    {
        motor_id_ = motor_id;
        for (size_t i = 0; i < motor_id.size(); i++)
        {
            std::cout << "[Ganglia] Completed set id " << motor_id[i] << " to motor " << i << std::endl;
        }
        return true;
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set id: size is mismatch");
        return false;
    }
}

bool GangliaMultiDriver::setTimeout(uint32_t byte_timeout_sec, uint32_t byte_timeout_usec, uint32_t response_timeout_sec, uint32_t response_timeout_usec)
{
    int ret = 0;
    ret = modbus_set_byte_timeout(modbus_, byte_timeout_sec, byte_timeout_usec);
    if(ret == -1)
    {
        //ROS_ERROR("[Ganglia] Failed to set byte timout");
        return false;
    }
    ret = modbus_set_response_timeout(modbus_, response_timeout_sec, response_timeout_usec);
    if(ret == -1)
    {
        //ROS_ERROR("[Ganglia] Failed to set response timout");
        return false;
    }
    uint32_t response_to_sec;
    uint32_t response_to_usec;
    modbus_get_byte_timeout(modbus_, &response_to_sec, &response_to_usec);
    printf("[Ganglia] Byte Timeout: %u %u\n", response_to_sec, response_to_usec);
    modbus_get_response_timeout(modbus_, &response_to_usec, &response_to_usec);
    printf("[Ganglia] Response Timeout: %u %u\n", response_to_sec, response_to_usec);
    return true;
}

bool GangliaMultiDriver::setReadEncoderAbs(std::vector<bool> status)
{
    if (motor_count_ == status.size())
    {
        is_read_encoder_abs_ = status;
        return true;
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set ReadEncoderAbs: size is mismatch");
        return false;
    }
}

bool GangliaMultiDriver::setMotorBLDC(std::vector<bool> status)
{
    if (motor_count_ == status.size())
    {
        is_set_bldc_ = status;
        return true;
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set MotorBLDC: size is mismatch");
        return false;
    }
}

bool GangliaMultiDriver::setMotorEnable(std::vector<bool> status)
{
    if(motor_count_ == status.size())
    {
        is_motor_enable_ = status;
        return true;
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set MotorEnable: size is mismatch");
        return false;
    }
}

bool GangliaMultiDriver::setMotorControlMode(std::vector<int> status)
{
    if(motor_count_ == status.size())
    {
        std::vector<ControlMode> tmp_mode(motor_count_);
        for(size_t i = 0; i < tmp_mode.size(); i++)
        {
            tmp_mode[i] = static_cast<ControlMode>(status[i]);
        }
        motor_control_mode_ = tmp_mode;
        return true;
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set MotorEnable: size is mismatch");
        return false;
    }
}

bool GangliaMultiDriver::connect()
{
    std::cout << "[Ganglia] ganglia_multi_driver : connect" << std::endl;
    if (modbus_connect(modbus_) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        is_connected_ = false;
    }
    else
    {
        is_connected_ = true;
    }
    return is_connected_;
}

bool GangliaMultiDriver::disconnect()
{
    std::cout << "[Ganglia] ganglia_multi_driver : disconnect" << std::endl;
    if(modbus_ != NULL)
    {
        modbus_close(modbus_);
    }
    is_connected_ = false;
    return true;
}

bool GangliaMultiDriver::setProfile()
{
    for (size_t i = 0; i < motor_count_; i++)
    {
        if (is_set_bldc_[i])
            if (setMotorBLDC(i) < 0)
            {
                return false;
            }

        if (setMotorControlMode(i) < 0)
        {
            return false;
        }
    }
    return true;
}

bool GangliaMultiDriver::setMotorBLDC()
{
    for (size_t i = 0; i < motor_count_; i++)
    {
        if (is_set_bldc_[i])
            if (setMotorBLDC(i) < 0)
            {
                return false;
            }
    }
    return true;
}

bool GangliaMultiDriver::setCurrentControl(uint32_t id)
{
    if (setCurrentMode(id) < 0)
    {
        return false;
    }
    motor_control_mode_[id] = Current;
    return true;
}

bool GangliaMultiDriver::setAllCurrentControl()
{
    for (size_t i = 0; i < motor_count_; i++)
    {
        if (!setCurrentControl(i))
        {
            return false;
        }
    }
    return true;
}

bool GangliaMultiDriver::setVelocityControl(uint32_t id)
{
    if (setVelocityMode(id) < 0)
    {
        return false;
    }
    motor_control_mode_[id] = Velocity;
    return true;
}

bool GangliaMultiDriver::setAllVelocityControl()
{
    for (size_t i = 0; i < motor_count_; i++)
    {
        if (!setVelocityControl(i))
        {
            return false;
        }
    }
    return true;
}

bool GangliaMultiDriver::setPositionControl(uint32_t id)
{
    if (setPositionMode(id) < 0)
    {
        return false;
    }
    motor_control_mode_[id] = Position;
    return true;
}

bool GangliaMultiDriver::setAllPositionControl()
{
    for (size_t i = 0; i < motor_count_; i++)
    {
        if (!setPositionControl(i))
        {
            return false;
        }
    }
    return true;
}

bool GangliaMultiDriver::isConnected()
{
    return is_connected_;
}

bool GangliaMultiDriver::startMotor(uint32_t id)
{
    //ROS_INFO("[GangliaMultiDriver] startMotor %d", id);
    if (setMotorEnable(id, is_motor_enable_[id]) < 0)
    {
        return false;
    }

    if (setMotorStart(id, true) < 0)
    {
        return false;
    }
    return true;
}

bool GangliaMultiDriver::startAllMotor()
{
    //ROS_INFO("[GangliaMultiDriver] startAllMotor");
    for (size_t i = 0; i < motor_count_; i++)
    {
        if(!startMotor(i))
            return false;
    }
    return breakAllMotor();

}

bool GangliaMultiDriver::stopMotor(uint32_t id)
{
    if (setMotorEnable(id, false) < 0)
    {
        return false;
    }

    if (setMotorStart(id, false) < 0)
    {
        return false;
    }
    return true;
}

bool GangliaMultiDriver::stopAllMotor()
{
    //ROS_INFO("[GangliaMultiDriver] stop");
    if(breakAllMotor())
    {
        for (size_t i = 0; i < motor_count_; i++)
        {
            if(!stopMotor(i))
                return false;
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool GangliaMultiDriver::setCurrent(uint32_t id, int value)
{
    if(id >= motor_id_.size())
    {
        return false;
    }

    bool ret = true;
    if (setMotorId(id))
    {
        if (modbus_write_registers(modbus_, 0x0028, 2, (uint16_t *)(&value)) < 0)
        {
            //ROS_ERROR("[Ganglia] Failed Write Wheel Current at %d", id);
            ret = false;
        }
    }
    else
    {
        ret = false;
    }
    return ret;
}

bool GangliaMultiDriver::setAllCurrent(std::vector<int> value)
{
    bool ret = true;
    if (motor_count_ == value.size())
    {
        for (size_t i = 0; i < motor_id_.size(); i++)
        {
            setCurrent(i, value[i]);
        }
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set Current: size is mismatch");
        ret = false;
    }
    return ret;
}

bool GangliaMultiDriver::setVelocity(uint32_t id, int value)
{
    if(id >= motor_id_.size())
    {
        return false;
    }

    bool ret = true;
    if (setMotorId(id))
    {
        if (modbus_write_registers(modbus_, 0x0018, 2, (uint16_t *)(&value)) < 0)
        {
			printf("[Ganglia] Failed Write Wheel Velocity at %d", id);
            //ROS_ERROR("[Ganglia] Failed Write Wheel Velocity at %d", id);
            ret = false;
        }
    }
    else
    {
        ret = false;
    }
    return ret;
}

bool GangliaMultiDriver::setAllVelocity(std::vector<int> value)
{
    bool ret = true;
    if (motor_count_ == value.size())
    {
        for (size_t i = 0; i < motor_id_.size(); i++)
        {
            setVelocity(i, value[i]);
        }
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set VelCmd: size is mismatch");
        ret = false;
    }
    return ret;
}

bool GangliaMultiDriver::setPosition(uint32_t id, int value)
{
    if(id >= motor_id_.size())
    {
        return false;
    }

    bool ret = true;
    if (setMotorId(id))
    {
        if (modbus_write_registers(modbus_, 0x0008, 2, (uint16_t *)(&value)) < 0)
        {
            //ROS_ERROR("[Ganglia] Failed Write Wheel Position at %d", id);
            ret = false;
        }
    }
    else
    {
        ret = false;
    }
    return ret;
}

bool GangliaMultiDriver::setAllPosition(std::vector<int> value)
{
    bool ret = true;
    if (motor_count_ == value.size())
    {
        for (size_t i = 0; i < motor_id_.size(); i++)
        {
            setPosition(i, value[i]);
        }
    }
    else
    {
        //ROS_WARN("[Ganglia] Cannot set Position: size is mismatch");
        ret = false;
    }
    return ret;
}

bool GangliaMultiDriver::getGangliaState(size_t id, GangliaState &data)
{
    if(id >= motor_id_.size())
    {
        return false;
    }

    if (!setMotorId(id))
    {
        return false;
    }

    GangliaState this_data;
    bool ret = true;
    uint16_t dest[6];
    if (modbus_read_input_registers(modbus_, 0x0000, 6, dest) < 0)
    {
        //ROS_ERROR("[Ganglia] Failed Read Wheel Data at %d", motor_id_[id]);
        this_data.id = id + 1;
        this_data.model_id = state_.data[id].model_id;
        this_data.state = GangliaStateName::ERROR;
        this_data.encoder = state_.data[id].encoder;
        this_data.vel = state_.data[id].vel;
        this_data.current = state_.data[id].current;
        ret = false;
    }
    else
    {
        this_data.id = id + 1;
        this_data.model_id = motor_id_[id];
        this_data.state = GangliaStateName::NORMAL;
        this_data.encoder = dest[0] | (dest[1] << 16);
        this_data.vel = dest[2] | (dest[3] << 16);
        this_data.current = dest[4] | (dest[5] << 16);
    }

    if (is_read_encoder_abs_[id])
    {
        if (modbus_read_input_registers(modbus_, 0x0021, 1, dest) < 0)
        {
            //ROS_ERROR("[Ganglia] Failed Read Wheel Data at %d", motor_id_[id]);
            this_data.state = GangliaStateName::ERROR;
            ret = false;
            this_data.encoder_abs = state_.data[id].encoder_abs;
        }
        else
        {
            this_data.encoder_abs = dest[0];
        }

    }
    else
        this_data.encoder_abs = 0;

    data = this_data;
    return ret;
}

bool GangliaMultiDriver::getAllGangliaState(GangliaStateArray &data)
{
    // std::cout << "[Ganglia] ganglia_multi_driver: get motor data" << std::endl;

    GangliaStateArray this_data;
    bool ret = true;
    this_data.data.resize(motor_count_);
    for (int i = 0; i < motor_count_; i++)
    {
        if(getGangliaState(i, this_data.data[i]))
        {

        }
        else
        {
            this_data.data[i] = state_.data[i];
        }
    }
    data = this_data;
    state_ = this_data;
    return ret;
}

// std::vector<bool> GangliaMultiDriver::getMotorPosition(std::vector<int32_t> &value)
// {
//     std::vector<bool> flag(motor_count_, true);
//     std::vector<int32_t> res(motor_count_, 0);
//     for (size_t i = 0; i < motor_count_; i++)
//     {
//         if (!setMotorId(i))
//         {
//             return std::vector<bool>(motor_count_, false);
//         }

//         uint16_t dest[2];
//         if (modbus_read_input_registers(modbus_, 0x0000, 2, dest) < 0)
//         {
//             flag[i] = false;
//             res[i] = 0;
//         }
//         else
//             res[i] = dest[0] | (dest[1] << 16);
//     }
//     value = res;
//     return flag;
// }

bool GangliaMultiDriver::setMotorId(int idx)
{
    if(idx >= motor_id_.size())
    {
        return false;
    }
    if (modbus_set_slave(modbus_, motor_id_[idx]) == -1)
    {
        return false;
    }
    return true;
}

int GangliaMultiDriver::setMotorBLDC(int id)
{
    if(id >= motor_id_.size())
    {
        return -1;
    }
	modbus_flush(modbus_);
    std::cout << "GangliaDriver::setMotorBLDC(" << motor_id_[id] << ")" << std::endl;
    //uint8_t src[1] = {0x05};
	int src = 1;

    //Set Board Id
    if (!setMotorId(id))
    {
        return -1;
    }
    if(modbus_write_bit(modbus_, 0x0008, src) < 0)
    {
        printf("[Motor-BLDC]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaMultiDriver::setMotorControlMode(int id)
{
    switch(motor_control_mode_[id])
    {
        case 0:
            return setCurrentMode(id);
            break;
        case 1:
            return setVelocityMode(id);
            break;
        case 2:
            return setPositionMode(id);
            break;
        default:
            return -1;
            break;
    }
}

int GangliaMultiDriver::setCurrentMode(int id)
{
    if(id >= motor_id_.size())
    {
        return -1;
    }
    std::cout << "GangliaDriver::setCurrentMode(" << motor_id_[id] << ")" << std::endl;

    if (!setMotorId(id))
    {
        return -1;
    }
    if (modbus_write_register(modbus_, 0x0080, 0xFFFD) < 0)
    {
        //ROS_WARN("[Cur-Mode]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaMultiDriver::setVelocityMode(int id)
{
    if(id >= motor_id_.size())
    {
        return -1;
    }
    std::cout << "GangliaDriver::setVelocityMode(" << motor_id_[id] << ")" << std::endl;

    if (!setMotorId(id))
    {
        return -1;
    }
    if (modbus_write_register(modbus_, 0x0080, 0xFFFE) < 0)
    {
		std::cout << "[Vel-Mode]Invalid Receive Discard !" << std::endl;
        return -1;
    }
    return 0;
}

int GangliaMultiDriver::setPositionMode(int id)
{
    if(id >= motor_id_.size())
    {
        return -1;
    }
    std::cout << "GangliaDriver::setPositionMode(" << motor_id_[id] << ")" << std::endl;
    if (!setMotorId(id))
    {
        return -1;
    }
    if (modbus_write_register(modbus_, 0x0080, 0xFFFF) < 0)
    {
        //ROS_WARN("[Pos-Mode]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaMultiDriver::setMotorEnable(int id, bool is_enable)
{
    if(id >= motor_id_.size())
    {
        return -1;
    }
    std::cout << "GangliaDriver::setMotorEnable(" << motor_id_[id] << ") : " << is_enable << std::endl;
    if (!setMotorId(id))
    {
        return -1;
    }
    // int enable = ( is_enable ) ? 0xFF00 : 0x0000;
    if (modbus_write_bit(modbus_, 0x000D, is_enable) < 0)
    {
        //ROS_WARN("[Motor-Enable]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

int GangliaMultiDriver::setMotorStart(int id, bool is_start)
{
    if(id >= motor_id_.size())
    {
        return -1;
    }
    std::cout << "GangliaDriver::setMotorStart(" << motor_id_[id] << ") : " << is_start << std::endl;
    if (!setMotorId(id))
    {
        return -1;
    }
    int enable = (is_start) ? 0xFF00 : 0x0000;
    if (modbus_write_bit(modbus_, 0x0003, enable) < 0)
    {
        //ROS_WARN("[Motor-Start]Invalid Receive Discard !");
        return -1;
    }
    return 0;
}

bool GangliaMultiDriver::breakAllMotor()
{
    std::vector<int> rpm_value(motor_count_, 0);
    if (setAllVelocity(rpm_value))
    {
        return true;
    }
    else
    {
        //ROS_ERROR("[Ganglia] Failed to break motor");
        return false;
    }
}

void GangliaMultiDriver::initStateArray()
{
    state_.data.resize(motor_count_);
    for(size_t id = 0; id < motor_count_; id++)
    {
        state_.data[id].id = id + 1;
        state_.data[id].model_id = motor_id_[id];
        state_.data[id].state = GangliaStateName::NORMAL;
        state_.data[id].encoder = 0;
        state_.data[id].vel = 0;
        state_.data[id].current = 0;
    }
}

}
