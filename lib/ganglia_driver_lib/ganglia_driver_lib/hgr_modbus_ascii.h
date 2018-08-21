/*
 * Copyright © 2001-2013 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef _HGR_MODBUS_ASCII_H_
#define _HGR_MODBUS_ASCII_H_

#include <unistd.h>
#include <sys/param.h>
#include <stdint.h>
#include <serial/serial.h>

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MODBUS_FC_REPORT_SLAVE_ID           0x11
#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17

#define MODBUS_BROADCAST_ADDRESS    0

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
 * Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
 * (chapter 6 section 11 page 29)
 * Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
 */
#define MODBUS_MAX_READ_BITS                2000
#define MODBUS_MAX_WRITE_BITS               1968

/* The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
 * address (1 byte) - CRC (2 bytes) = 253 bytes.
 *
 * Consequently:
 * - RS232 / RS485 ADU = 253 bytes + Server address (1 byte) + CRC (2 bytes) =
 *   256 bytes.
 * - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes.
 */
#define MODBUS_ASCII_MAX_ADU_LENGTH         256

/* Random number to avoid errno conflicts */
#define MODBUS_ENOBASE 112345678

/* Protocol exceptions */
enum {
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE,
    MODBUS_EXCEPTION_ACKNOWLEDGE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY,
    MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE,
    MODBUS_EXCEPTION_MEMORY_PARITY,
    MODBUS_EXCEPTION_NOT_DEFINED,
    MODBUS_EXCEPTION_GATEWAY_PATH,
    MODBUS_EXCEPTION_GATEWAY_TARGET,
    MODBUS_EXCEPTION_MAX
};

#define EMBXILFUN  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_FUNCTION)
#define EMBXILADD  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS)
#define EMBXILVAL  (MODBUS_ENOBASE + MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE)
#define EMBXSFAIL  (MODBUS_ENOBASE + MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE)
#define EMBXACK    (MODBUS_ENOBASE + MODBUS_EXCEPTION_ACKNOWLEDGE)
#define EMBXSBUSY  (MODBUS_ENOBASE + MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY)
#define EMBXNACK   (MODBUS_ENOBASE + MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE)
#define EMBXMEMPAR (MODBUS_ENOBASE + MODBUS_EXCEPTION_MEMORY_PARITY)
#define EMBXGPATH  (MODBUS_ENOBASE + MODBUS_EXCEPTION_GATEWAY_PATH)
#define EMBXGTAR   (MODBUS_ENOBASE + MODBUS_EXCEPTION_GATEWAY_TARGET)

/* Native libmodbus error codes */
#define EMBBADCRC  (EMBXGTAR + 1)
#define EMBBADDATA (EMBXGTAR + 2)
#define EMBBADEXC  (EMBXGTAR + 3)
#define EMBUNKEXC  (EMBXGTAR + 4)
#define EMBMDATA   (EMBXGTAR + 5)
#define EMBBADSLAVE (EMBXGTAR + 6)

class HgrModbusAscii
{
public:
    HgrModbusAscii();

    int setSlaveAddress(int slave_address);
    int setSerial(serial::Serial  *serial_ptr);

    int readCoils(int addr, int nb, uint8_t *dest);
    int readDiscreateInputs(int addr, int nb, uint8_t *dest);
    int readHoldingRegisters(int addr, int nb, uint16_t *dest);
    int readInputRegisters(int addr, int nb, uint16_t *dest);
    int writeSingleCoil(int addr, int value);
    int writeSingleRegister(int addr, int value);
    int writeMultipleCoils(int addr, int nb, const uint8_t *src);
    int writeMultipleRegisters(int addr, int nb, const uint16_t *src);
    int maskWriteRegisters(int addr, uint16_t and_mask, uint16_t or_mask);
    int writeAndReadRegisters(int write_addr, int write_nb, const uint16_t *src, int read_addr, int read_nb, uint16_t *dest);

private:
    int8_t ascii_to_binary_[256];
    int8_t binary_to_ascii_[16];

    int slave_address_;
    serial::Serial  *serial_ptr_;    // Serial port from Serial Lib

    uint8_t req_buff_[MODBUS_ASCII_MAX_ADU_LENGTH];
    uint8_t rsp_buff_[MODBUS_ASCII_MAX_ADU_LENGTH];
    uint8_t ascii_buff_[MODBUS_ASCII_MAX_ADU_LENGTH*2];

    const char *modbusStrError(int errnum);
    int buildRequestBasis(int function, int addr, int nb, uint8_t *req);
    int sendMessage(const unsigned char *cmd, int size);
    int computeRspLength(int function, int nb);
    int receiveMessage(uint8_t *rsp_buff, const unsigned char *cmd, int size);

    int readIoStatus(int function, int addr, int nb, uint8_t *dest);
    int readRegisters(int function, int addr, int nb, uint16_t *dest);
    int writeSingle(int function, int addr, int value);
};
#endif // _HGR_MODBUS_ASCII_H_