
#include "ganglia_driver_lib/hgr_modbus_ascii.h"
#include <iostream>

HgrModbusAscii::HgrModbusAscii() : serial_ptr_(0)
{
    memset(ascii_to_binary_, 0x00, 256*sizeof(ascii_to_binary_[0]));
    for(char i = '0'; i <= '9'; i++)
        ascii_to_binary_[i] = i - '0';
    for(char i = 'A'; i <= 'F'; i++)
        ascii_to_binary_[i] = 10 + i - 'A';
    for(char i = 'a'; i <= 'f'; i++)
        ascii_to_binary_[i] = 10 + i - 'a';

    for(int i = 0; i < 10; i++)
        binary_to_ascii_[i] = i + '0';
    for(int i = 10; i < 16; i++)
        binary_to_ascii_[i] = i - 10 + 'A';
}

int HgrModbusAscii::setSlaveAddress(int slave_address)
{
    if (slave_address < 0 || slave_address > 247)
        return -1;

    slave_address_ = slave_address;
    return 0;
}

int HgrModbusAscii::setSerial(serial::Serial  *serial_ptr)
{
    if(!serial_ptr)
        return -1;

    serial_ptr_ = serial_ptr;
    return 0;
}

int HgrModbusAscii::readCoils(int addr, int nb, uint8_t *dest)
{
    int rc = readIoStatus(MODBUS_FC_READ_COILS, addr, nb, dest);
    return ( rc == -1 ) ? -1 : rc;
}

int HgrModbusAscii::readDiscreateInputs(int addr, int nb, uint8_t *dest)
{
    int rc = readIoStatus(MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, dest);
    return ( rc == -1 ) ? -1 : rc;
}

int HgrModbusAscii::readHoldingRegisters(int addr, int nb, uint16_t *dest)
{
    int rc = readRegisters(MODBUS_FC_READ_HOLDING_REGISTERS, addr, nb, dest);
    return ( rc == -1 ) ? -1 : rc;
}

int HgrModbusAscii::readInputRegisters(int addr, int nb, uint16_t *dest)
{
    int rc = readRegisters(MODBUS_FC_READ_INPUT_REGISTERS, addr, nb, dest);
    return ( rc == -1 ) ? -1 : rc;
}

int HgrModbusAscii::writeSingleCoil(int addr, int value)
{
    value = (value) ? 0xFF00 : 0x0000;
    int rc = writeSingle(MODBUS_FC_WRITE_SINGLE_COIL, addr, value);
    return ( rc == -1 ) ? -1 : rc;
}

int HgrModbusAscii::writeSingleRegister(int addr, int value)
{
    int rc = writeSingle(MODBUS_FC_WRITE_SINGLE_REGISTER, addr, value);
    return ( rc == -1 ) ? -1 : rc;
}

int HgrModbusAscii::writeMultipleCoils(int addr, int nb, const uint8_t *src)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;

    int function = MODBUS_FC_WRITE_MULTIPLE_COILS;
    int req_length = buildRequestBasis(function, addr, nb, req_buff_);
    int byte_count = (nb + 7) / 8;
    req_buff_[req_length++] = byte_count;

    int bit_count = 0;
    for(int i = 0; i < byte_count; i++)
    {
        req_buff_[req_length] = 0;
        for(int bit = 0x01; bit < 0x100 && bit_count < nb; bit <<= 1)
        {
            if( src[bit_count] )
                req_buff_[req_length] |= bit;
            bit_count++;
        }
        req_length++;
    }

    int computedRspLength = computeRspLength(function, nb);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);
    if( ascii_length != computedRspLength )
        return -1;

    int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function )
        return -1;

    int rsp_address = rsp_buff_[2] << 8 | rsp_buff_[3];
    if( rsp_address != addr )
        return -1;

    int rsp_nb = rsp_buff_[4] << 8 | rsp_buff_[5];
    if( rsp_nb != nb )
        return -1;

    return nb;
}
int HgrModbusAscii::writeMultipleRegisters(int addr, int nb, const uint16_t *src)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;

    //Clear Buffer
    std::string clear_data = serial_ptr_->read(serial_ptr_->available());

    int function = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    int req_length = buildRequestBasis(function, addr, nb, req_buff_);

    req_buff_[req_length++] = nb*2;
    for(int i = 0; i < nb; i++)
    {
        req_buff_[req_length++] = src[i] >> 8;
        req_buff_[req_length++] = src[i] & 0x00FF;
    }

    int computedRspLength = computeRspLength(function, nb);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    // std::cout << serial_ptr_->available() << std::endl;
    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);
    
    // std::cout << "NOOOOO: " << ascii_length << " " << computedRspLength << " " << serial_ptr_->available() << std::endl;
    if( ascii_length != computedRspLength )
        return -1;

    int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function )
        return -1;

    int rsp_address = rsp_buff_[2] << 8 | rsp_buff_[3];
    if( rsp_address != addr )
        return -1;

    int rsp_nb = rsp_buff_[4] << 8 | rsp_buff_[5];
    if( rsp_nb != nb )
        return -1;

    return nb;
}

int HgrModbusAscii::maskWriteRegisters(int addr, uint16_t and_mask, uint16_t or_mask)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;

    int function = MODBUS_FC_MASK_WRITE_REGISTER;
    int req_length = buildRequestBasis(function, addr, 0, req_buff_) - 2; // nb is not used

    req_buff_[req_length++] = and_mask >> 8;
    req_buff_[req_length++] = and_mask & 0x00ff;
    req_buff_[req_length++] = or_mask >> 8;
    req_buff_[req_length++] = or_mask & 0x00ff;

    int computedRspLength = computeRspLength(function, 0);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);
    // std::cout << "maskWriteRegisters : " << computedRspLength << "/" << ascii_length << std::endl;
    // std::cout << "buff_cnt : " << ascii_length << std::endl;
    // for(int i = 0; i < ascii_length; i++)
    //     std::cout << ascii_buff_[i];
    // std::cout << std::endl;

    if( ascii_length != computedRspLength )
        return -1;

    // std::cout << "maskWriteRegisters : " << std::endl;
    int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function )
        return -1;

    // std::cout << "maskWriteRegisters : " << std::endl;
    int rsp_and = rsp_buff_[2] << 8 | rsp_buff_[3];
    if( rsp_and != and_mask )
        return -1;

    // std::cout << "maskWriteRegisters : " << std::endl;
    int rsp_or = rsp_buff_[4] << 8 | rsp_buff_[5];
    if( rsp_or != or_mask )
        return -1;

    // std::cout << "maskWriteRegisters : " << std::endl;
    return 1;
}

int HgrModbusAscii::writeAndReadRegisters(int write_addr, int write_nb, const uint16_t *src, int read_addr, int read_nb, uint16_t *dest)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;

    int function = MODBUS_FC_WRITE_AND_READ_REGISTERS;
    int req_length = buildRequestBasis(function, read_addr, read_nb, req_buff_);
    req_buff_[req_length++] = write_addr >> 8;
    req_buff_[req_length++] = write_addr & 0x00FF;
    req_buff_[req_length++] = write_nb >> 8;
    req_buff_[req_length++] = write_nb & 0x00FF;

    int byte_count = write_nb * 2;
    req_buff_[req_length++] = byte_count;

    for(int i = 0; i < write_nb; i++)
    {
        req_buff_[req_length++] = src[i] >> 8;
        req_buff_[req_length++] = src[i] & 0x00FF;
    }

    int computedRspLength = computeRspLength(function, read_nb);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);
    if( ascii_length != computedRspLength )
        return -1;

    int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function || rsp_buff_[2] != 2*read_nb )
        return -1;

    int dest_cnt = 0;
    for(int i = 0; i < rsp_buff_[2]; i+=2)
        dest[dest_cnt++] = rsp_buff_[3+i] << 8 | rsp_buff_[4+i];

    if( dest_cnt != read_nb )
        return -1;

    return read_nb;
}

const char* HgrModbusAscii::modbusStrError(int errnum)
{
    switch(errnum)
    {
    case EMBXILFUN:     return "Illegal function";
    case EMBXILADD:     return "Illegal data address";
    case EMBXILVAL:     return "Illegal data value";
    case EMBXSFAIL:     return "Slave device or server failure";
    case EMBXACK:       return "Acknowledge";
    case EMBXSBUSY:     return "Slave device or server is busy";
    case EMBXNACK:      return "Negative acknowledge";
    case EMBXMEMPAR:    return "Memory parity error";
    case EMBXGPATH:     return "Gateway path unavailable";
    case EMBXGTAR:      return "Target device failed to respond";
    case EMBBADCRC:     return "Invalid CRC";
    case EMBBADDATA:    return "Invalid data";
    case EMBBADEXC:     return "Invalid exception code";
    case EMBMDATA:      return "Too many data";
    case EMBBADSLAVE:   return "Response not from requested slave";
    default:            return strerror(errnum);
    }
}

int HgrModbusAscii::buildRequestBasis(int function, int addr, int nb, uint8_t *req)
{
    req_buff_[0] = slave_address_;
    req_buff_[1] = function;
    req_buff_[2] = addr >> 8;
    req_buff_[3] = addr & 0x00ff;
    req_buff_[4] = nb >> 8;
    req_buff_[5] = nb & 0x00ff;

    return 6;
}

int HgrModbusAscii::sendMessage(const unsigned char *cmd, int size)
{
    serial_ptr_->flush();
    int buff_cnt = 0;

    ascii_buff_[buff_cnt++] = ':';
    for (int i = 0; i < size; i++)
    {
        ascii_buff_[buff_cnt++] = binary_to_ascii_[cmd[i] >> 4];
        ascii_buff_[buff_cnt++] = binary_to_ascii_[cmd[i] & 0x0f];
    }

    // check sum
    unsigned char lrc8 = 0;
    for(int i = 0; i < size; i++)
        lrc8 += cmd[i];
    lrc8 = 0x100 - lrc8;

    ascii_buff_[buff_cnt++] = binary_to_ascii_[lrc8 >> 4];
    ascii_buff_[buff_cnt++] = binary_to_ascii_[lrc8 & 0x0f];

    // Two Ending bytes [CR LF]
    ascii_buff_[buff_cnt++] = 0x0D;
    ascii_buff_[buff_cnt++] = 0x0A;

    // std::cout << "buff_cnt : " << buff_cnt << std::endl;
    // for(int i = 0; i < buff_cnt; i++)
    //     std::cout << ascii_buff_[i];
    // std::cout << std::endl;
    // Send Command via Serial connection
    int write_res = serial_ptr_->write((unsigned char *)ascii_buff_, buff_cnt);
    return 0; // TODO: check for error
}

int HgrModbusAscii::computeRspLength(int function, int nb)
{
    // ADU = Address + PDU + Error check
    // PDU = Function code + Data
    // ':', {Address, Function, [Data], chksum}, CR, LF
    
    const int pre_post_cnt = 1 + 2 * (2 + 0 + 1) + 2;
    int length = 0; // length of Data

    switch(function)
    {
    case MODBUS_FC_READ_COILS:                  length = 1 + ((nb+7)/8); break;
    case MODBUS_FC_READ_DISCRETE_INPUTS:        length = 1 + ((nb+7)/8); break;
    case MODBUS_FC_READ_HOLDING_REGISTERS:      length = 1 + 2*nb; break;
    case MODBUS_FC_READ_INPUT_REGISTERS:        length = 1 + 2*nb; break;
    case MODBUS_FC_WRITE_SINGLE_COIL:           length = 4; break;
    case MODBUS_FC_WRITE_SINGLE_REGISTER:       length = 4; break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS:        length = 2 * (1 + 1); break;
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:    length = 2 * (1 + 1); break;
    case MODBUS_FC_MASK_WRITE_REGISTER:         length = 2 * (1 + 1 + 1); break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS:    length = 1 + 2*nb; break;

    default:                                    length = 5;
    }

    int adu_length = pre_post_cnt + 2*length;
    return (adu_length < MODBUS_ASCII_MAX_ADU_LENGTH) ? adu_length : -1;
}

int HgrModbusAscii::receiveMessage(uint8_t *dst, const unsigned char *src, int size)
{
    // ADU = Address + PDU + Error check
    // PDU = Function code + Data
    // ':', {address, function, [data], chksum}, CR, LF

    // std::cout << "receiveMessage : " << size << "," << src[0] << std::endl;

    if( size < 9 || src[0] != ':' )
        return -1;

    // std::cout << "size : " << size << std::endl;
    // for(int i = 0; i < size; i++)
    //     std::cout << src[i];
    // std::cout << std::endl;

    int dst_cnt = 0;

    for (int i = 1; i < size; i+=2) // exclude 
        dst[dst_cnt++] = (ascii_to_binary_[src[i]] << 4) | ascii_to_binary_[src[i+1]];

    unsigned char lrc8 = 0;
    for(int i = 0; i < dst_cnt; i++)
        lrc8 += dst[i];

    if(lrc8 != 0)
        return -1;

    return dst_cnt;
}

int HgrModbusAscii::readIoStatus(int function, int addr, int nb, uint8_t *dest)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;

    int req_length = buildRequestBasis(function, addr, nb, req_buff_);

    int computedRspLength = computeRspLength(function, nb);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);
    if( ascii_length != computedRspLength )
        return -1;

    int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function || rsp_buff_[2] != ((nb+7)/8) )
        return -1;

    int dest_cnt = 0;
    for(int i = 0; i < rsp_buff_[2]; i++)
    {
        int tmp = rsp_buff_[3+i];
        for(int j = 0x01; j < 0x100 && dest_cnt < nb; j <<= 1)
            dest[dest_cnt++] = (tmp & j) ? 1 : 0;
    }

    if( dest_cnt != nb )
        return -1;

    return nb;
}

int HgrModbusAscii::readRegisters(int function, int addr, int nb, uint16_t *dest)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;
    // //Clear Buffer
    std::string clear_data = serial_ptr_->read(serial_ptr_->available());

    int req_length = buildRequestBasis(function, addr, nb, req_buff_);

    int computedRspLength = computeRspLength(function, nb);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);    
    // std::cout << "YESSSSSS: " << ascii_length << " " << computedRspLength << " " << serial_ptr_->available() << std::endl;
    if( ascii_length != computedRspLength )
        return -1;

    int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    // std::cout << "readRegisters y : " << rsp_length << std::endl;
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function || rsp_buff_[2] != 2*nb )
        return -1;

    // std::cout << "readRegisters z : " << std::endl;
    int dest_cnt = 0;
    for(int i = 0; i < rsp_buff_[2]; i+=2)
        dest[dest_cnt++] = rsp_buff_[3+i] << 8 | rsp_buff_[4+i];

    // std::cout << "readRegisters : " << std::endl;
    if( dest_cnt != nb )
        return -1;

    // std::cout << "readRegisters : " << std::endl;
    return nb;
}

int HgrModbusAscii::writeSingle(int function, int addr, int value)
{
    if( !serial_ptr_ || !serial_ptr_->isOpen() )
        return -1;

    int req_length = buildRequestBasis(function, addr, value, req_buff_);

    int computedRspLength = computeRspLength(function, 0);
    if( computedRspLength < 0 )
        return -1;

    int rc = sendMessage(req_buff_, req_length);
    if (rc == -1)
        return -1;

    int ascii_length = serial_ptr_->read(ascii_buff_, computedRspLength);
    if( ascii_length != computedRspLength )
        return -1;

   int rsp_length = receiveMessage(rsp_buff_, ascii_buff_, ascii_length);
    if( rsp_length < 0 || rsp_buff_[0] != slave_address_ || rsp_buff_[1] != function )
        return -1;

    int rsp_address = rsp_buff_[2] << 8 | rsp_buff_[3];
    if( rsp_address != addr )
        return -1;

    int rsp_value = rsp_buff_[4] << 8 | rsp_buff_[5];
    if( rsp_value != value )
        return -1;

    return 1;
}
