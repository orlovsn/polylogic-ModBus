/*
 * Copyright (c) 2016. Stanislav Orlov www.polylogic.org orlovsn@live.ru
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * 	 it under the terms of the GNU General Public License as published by
 * 	 the Free Software Foundation, either version 3 of the License, or
 * 	 (at your option) any later version.
 *
 * 	 This program is distributed in the hope that it will be useful,
 * 	 but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	 GNU General Public License for more details.
 *
 * 	 You should have received a copy of the GNU General Public License
 * 	 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package polylogic.org.protocols.ModBus;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import polylogic.org.protocols.ANSI.CRC16ANSI;

/**
 * This library generates typical requests for Modbus protocol. Currently Modbus
 * RTU, Modbus TCP(UDP), Modbus over TCP(UDP) and Modbus ASCII are supported.
 * CRC and LRC calculators and checkers are also implemented. Console (CLI)
 * version is provided as example of library usage.
 *
 * Additionally library can apply or check CRC (CRC-16-ANSI for now, LRC CRC
 * will be added soon) for both Modbus requests and replies.
 *
 * Modbus is a serial communications protocol originally published by Modicon
 * (now Schneider Electric) in 1979 for use with its programmable logic
 * controllers (PLCs). Simple and robust, it has since become a de facto
 * standard communication protocol, and it is now a commonly available means of
 * connecting industrial electronic devices. Modbus organization web site:
 * http://www.modbus.org/ Basic modbus reference can be found here:
 * https://en.wikipedia.org/wiki/Modbus
 *
 * Note that this library is not strict to Modbus standard implementation
 * because, unfortunately, some manufacturers implement addressing in
 * non-standard way, like using 40000 holding register address, 248-255 device
 * addresses and so on, so this library won't check whether device, coil,
 * register etc addresses comply to generic Modbus standard, but will allow any
 * reasonable values as input parameters to allow non-standard implementations
 * work. All offsets of registers addresses must be applied before functions
 * calls, they are described in functions descriptions and Modbus reference
 * manuals.
 *
 * @author Stanislav Orlov www.polylogic.org orlovsn@live.ru
 * @version 1.0
 */
public class ModBus {

    /**
     * Basically this library carries one identificator and increases it with
     * every call to convertToTCP (internally called in every command generator,
     * when Modbus type is set to Modbus-TCP/UDP). So if you have multiple
     * Modbus-TCP/UDP devices, they will share this id (for example if you have
     * 3 devices, that are requested in order, then first packet to device1 will
     * have id 1, packet to device2 will have id 2, packet to device3 will have
     * id 3). In most cases this works ok because device, due to Modbus
     * standard, don't perfom id check and only copies it in reply, but if you
     * want separate ids for each device, you have to implement your own list of
     * device addresses and take care of exact id increment every time you
     * prepare a command for device, then you can force id with setTcpID
     * function (note that tcpID is static, so if you have multiple threads you
     * have to synchronize your double-call transaction - first set id, than
     * generate command) or you can simply overwrite id with your own in
     * generated command.
     */
    public static int tcpID = 0;

    /**
     * Modbus RTU Type with CRC-16-ANSI
     */
    public static final int TYPE_MODBUS_RTU = 1;

    /**
     * Modbus TCP or Modbus UDP Type (TCP and UDP use the same ADU) without CRC
     * in request. Don't mess with Modbus <b>over</b> TCP which requires CRC!
     * This type handles MBAP+PDU, where MBAP Unit Identifier is set as
     * deviceAddress.
     * <b>IMPORTANT</b> MODBUS MESSAGING ON TCP/IP IMPLEMENTATION GUIDE V1.0b
     * says that: <i>"All MODBUS requests and responses are designed in such a
     * way that the recipient can verify that a message is finished. For
     * function codes where the MODBUS PDU has a fixed length, the function code
     * alone is <b>sufficient</b>. For function codes carrying a variable amount
     * of data in the request or response, the data field includes a byte
     * count."</i> This means that writing single coil or holding register must not contain length field.
     */
    public static final int TYPE_MODBUS_TCP_UDP = 2;

    /**
     * Modbus over TCP or Modbus <b>over</b> UDP Type with CRC-16-ANSI. Don't
     * mess with Modbus TCP which doesn't require CRC!
     */
    public static final int TYPE_MODBUS_OVER_TCP = 3;

    /**
     * Modbus ASCII with LRC CRC
     */
    public static final int TYPE_MODBUS_ASCII = 4;    

    /**
     * Function code received in the query is not recognized or allowed by slave
     */
    public static final int RESPONSE_Illegal_Function = 1;
    /**
     * Data address of some or all the required entities are not allowed or do
     * not exist in slave
     */
    public static final int RESPONSE_Illegal_Data_Address = 2;
    /**
     * Value is not accepted by slave
     */
    public static final int RESPONSE_Illegal_Data_Value = 3;
    /**
     * Unrecoverable error occurred while slave was attempting to perform
     * requested action
     */
    public static final int RESPONSE_Slave_Device_Failure = 4;
    /**
     * Slave has accepted request and is processing it, but a long duration of
     * time is required. This response is returned to prevent a timeout error
     * from occurring in the master. Master can next issue a Poll Program
     * Complete message to determine if processing is completed
     */
    public static final int RESPONSE_Acknowledge = 5;
    /**
     * Slave is engaged in processing a long-duration command. Master should
     * retry later
     */
    public static final int RESPONSE_Slave_Device_Busy = 6;
    /**
     * Slave cannot perform the programming functions. Master should request
     * diagnostic or error information from slave
     */
    public static final int RESPONSE_Negative_Acknowledge = 7;
    /**
     * Slave detected a parity error in memory. Master can retry the request,
     * but service may be required on the slave device
     */
    public static final int RESPONSE_Memory_Parity_Error = 8;
    /**
     * Specialized for Modbus gateways. Indicates a misconfigured gateway
     */
    public static final int RESPONSE_Gateway_Path_Unavailable = 10;
    /**
     * Specialized for Modbus gateways. Sent when slave fails to respond
     */
    public static final int RESPONSE_Gateway_Target_Device_Failed_to_Respond = 11;
    /**
     * CRC in reply was faulty - probably data was broken on the way
     */
    public static final int RESPONSE_CRC_ERROR = 100;
    /**
     * Reply don't have data at all or data in insufficient
     */
    public static final int RESPONSE_NO_DATA = 101;

    /**
     * <p>
     * Read coils values (Modbus function code: 1)
     * </p>
     * <p>
     * For example to read value of 3 coils starting from coil 2 in device with
     * address 10 in RTU format write: readValueOfCoils(10, 2, 3,
     * TYPE_MODBUS_RTU);
     * </p>
     *
     * @param deviceAddress Address of device where command will be executed
     * (slave device)
     * @param firstCoilAddress First coil address
     * @param numberOfCoils Number of coils to read
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented. <br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] readValueOfCoils(byte deviceAddress, int firstCoilAddress, int numberOfCoils, int modbusType) {
        byte[] command = prepareRTU6ByteCommand(deviceAddress, (byte) 1, firstCoilAddress, numberOfCoils);
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Read inputs values (Modbus function code: 2)
     * </p>
     * <p>
     * Note that input address is passed with offset of 10001 - so to point to
     * input 10003 you need to point to address 10003-10001=2 To point to 10001
     * address you need to point to 0 address (zero offset).
     * </p>
     * <p>
     * For example to read value of 3 inputs starting from input 10002 in device
     * with address 10 in RTU format write: readValueOfInputs(10, 1, 3,
     * TYPE_MODBUS_RTU);
     * </p>
     *
     * @param deviceAddress Address of device where command will be executed
     * (slave device)
     * @param firstInputAddress First coil address
     * @param numberOfInputs Number of coils to read
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented.<br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] readValueOfInputs(byte deviceAddress, int firstInputAddress, int numberOfInputs,
            int modbusType) {
        byte[] command = prepareRTU6ByteCommand(deviceAddress, (byte) 2, firstInputAddress, numberOfInputs);
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Read values of holding registers (Modbus function code: 3)
     * </p>
     * <p>
     * Note that holding register address is passed with offset of 40001 - so to
     * point to holding register 40003 you need to point to address
     * 40003-40001=2 To point to 40001 address you need to point to 0 address
     * (zero offset).
     * </p>
     * <p>
     * For example to read value of 3 holding registers starting from register
     * 40002 in device with address 10 in RTU format write:
     * readValueOfHoldingRegisters(10, 1, 3, TYPE_MODBUS_RTU);
     * </p>
     *
     * @param deviceAddress Address of device where command will be executed
     * (slave device)
     * @param firstRegisterAddress First register address
     * @param numberOfRegisters Number of registers to read
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented. <br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] readValueOfHoldingRegisters(byte deviceAddress, int firstRegisterAddress,
            int numberOfRegisters, int modbusType) {
        byte[] command = prepareRTU6ByteCommand(deviceAddress, (byte) 3, firstRegisterAddress, numberOfRegisters);
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Read values of input registers (Modbus function code: 4)
     * </p>
     * <p>
     * Note that input register address is passed with offset of 30001 - so to
     * point to register 30003 you need to point to address 30003-30001=2 To
     * point to 30001 address you need to point to 0 address (zero offset).
     * </p>
     * <p>
     * For example to read value of 3 input registers starting from register
     * 30002 in device with address 10 in RTU format write:
     * readValueOfInputRegisters(10, 1, 3, TYPE_MODBUS_RTU);
     * </p>
     *
     * @param deviceAddress Address of device where command will be executed
     * (slave device)
     * @param firstRegisterAddress First register address
     * @param numberOfRegisters Number of registers to read
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented. <br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] readValueOfInputRegisters(byte deviceAddress, int firstRegisterAddress, int numberOfRegisters,
            int modbusType) {
        byte[] command = prepareRTU6ByteCommand(deviceAddress, (byte) 4, firstRegisterAddress, numberOfRegisters);
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Writes value of single coil (Modbus function code: 5)
     * </p>
     * <p>
     * For example to write value 70 in coil 2 of device with address 10 in RTU
     * format write: writeValueOfSingleCoil(10, 2, 70, TYPE_MODBUS_RTU);
     * </p>
     *
     * @param deviceAddress Address of Device where command will be executed
     * (slave device)
     * @param coilAddress Address of Coil where new value should be written to
     * @param newCoilValue New Coil value. Due to Modbus standard, coil values
     * must be 0 (disable) or 1 (enable), but as we are not strict any value
     * will be accepted but values diffe7++++++++++++
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented. <br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] writeValueOfSingleCoil(byte deviceAddress, int coilAddress, int newCoilValue, int modbusType) {
        byte[] command = prepareRTU6ByteCommand(deviceAddress, (byte) 5, coilAddress, newCoilValue);
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Writes value of single holding register (Modbus function code: 6)
     * </p>
     * <p>
     * Note that holding register address is passed with offset of 40001 - so to
     * point to holding register 40003 you need to point to address
     * 40003-40001=2 To point to 40001 address you need to point to 0 address
     * (zero offset).
     * </p>
     * <p>
     * For example to write value 70 in holding register 40002 of device with
     * address 10 in RTU format write: writeValueOfSingleHoldingRegister(10, 1,
     * 70, TYPE_MODBUS_RTU);
     * </p>
     *
     * @param deviceAddress Address of Device where command will be executed
     * (slave device)
     * @param registerAddress Address of Register where new value should be
     * written to
     * @param newRegisterValue New Register value
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented. <br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] writeValueOfSingleHoldingRegister(byte deviceAddress, int registerAddress,
            int newRegisterValue, int modbusType) {
        byte[] command = prepareRTU6ByteCommand(deviceAddress, (byte) 6, registerAddress, newRegisterValue);
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Writes values of multiple holding registers (Modbus function code: 16)
     * </p>
     * <p>
     * Note that holding register address is passed with offset of 40001 - so to
     * point to holding register 40003 you need to point to address
     * 40003-40001=2 To point to 40001 address you need to point to 0 address
     * (zero offset).
     * </p>
     *
     * @param deviceAddress Address of Device where command will be executed
     * (slave device)
     * @param firstRegisterAddress Address of first Register where new values
     * should be written to
     * @param newRegistersValues List of new Registers values. Number of
     * registers to write data to will be calculated from this List's size.
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented.<br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] writeValueOfMultipleHoldingRegisters(byte deviceAddress, int firstRegisterAddress,
            List<Integer> newRegistersValues, int modbusType) {
        byte[] command = new byte[7 + (newRegistersValues.size() * 2)];
        // 7 = 1 byte device address + 1 byte function code + 2 bytes of first
        // register address + 2 bytes number of registers to write + 1 byte
        // number of data bytes.
        // Each register value is passed in two bytes. CRC will be applied later
        // if required by Modbus ADU type.
        command[0] = deviceAddress;
        command[1] = (byte) 16;
        command[2] = (byte) ((firstRegisterAddress >> 8) & 0xFF);
        command[3] = (byte) (firstRegisterAddress & 0xFF);
        command[4] = (byte) ((newRegistersValues.size() >> 8) & 0xFF);
        command[5] = (byte) (newRegistersValues.size() & 0xFF);
        command[6] = (byte) (newRegistersValues.size() * 2);
        for (int regNum = 0; regNum < newRegistersValues.size(); regNum++) {
            command[7 + (regNum * 2)] = (byte) ((newRegistersValues.get(regNum) >> 8) & 0xFF);
            command[7 + (regNum * 2) + 1] = (byte) (newRegistersValues.get(regNum) & 0xFF);
        }
        return convertCommandToModbusType(command, modbusType);
    }

    
    public static byte[] writeValueOfMultipleHoldingRegisters(byte deviceAddress, int firstRegisterAddress,
            byte[] newRegistersValues, int modbusType){
        byte[] command = new byte[7 + (newRegistersValues.length)];
        command[0] = deviceAddress;
        command[1] = (byte) 16;
        command[2] = (byte) ((firstRegisterAddress >> 8) & 0xFF);
        command[3] = (byte) (firstRegisterAddress & 0xFF);
        command[4] = (byte) (((newRegistersValues.length /2) >> 8) & 0xFF);
        command[5] = (byte) (newRegistersValues.length /2 );
        command[6] = (byte) (newRegistersValues.length & 0xFF);
        System.arraycopy(newRegistersValues, 0, command, 7, newRegistersValues.length);
        return convertCommandToModbusType(command, modbusType);
    }
    /**
     * <p>
     * Writes values of multiple coils (Modbus function code: 15)
     * </p>
     *
     * @param deviceAddress Address of Device where command will be executed
     * (slave device)
     * @param firstCoilAddress Address of first Coil where new values should be
     * written to
     * @param newCoilsValues List of new Coils values. Any int above 0 will be
     * parsed as 1 (enable), 0 or negative ints will be parsed as 0 (disable).
     * Number of coils to write data to will be calculated from this List's
     * size.
     * @param modbusType Defines output data format. Currently only RTU, TCP/UDP
     * and over TCP/UDP types implemented.<br>
     * If RTU or Modbus <b>over</b> TCP is selected output will be binary with
     * CRC-16-ANSI. <br>
     * If TCP (for UDP use TCP mode - they are the same) is selected output will
     * be binary without TCP, ASCII will return byte-formated ASCII with LRC
     * CRC. <br>
     * Note that Modbus <b>over</b> TCP/IP and Modbus TCP are different - don't
     * mess them! Modbus TCP relies on TCP's correction mechanisms and don't use
     * CRC in Modbus commands - some devices operating Modbus TCP or Modbus UDP
     * will fail to communicate with CRC applied.
     * @return Prepared to execute command.
     */
    public static byte[] writeValueOfMultipleCoils(byte deviceAddress, int firstCoilAddress,
            List<Integer> newCoilsValues, int modbusType) {
        int dataBytesNum = newCoilsValues.size() / 8 + 1;
        // Each data byte in this packet contains values of 8 coils so to write
        // 10 coils we need 2 bytes.
        byte[] command = new byte[7 + dataBytesNum];
        // 7 = 1 byte device address + 1 byte function code + 2 bytes of first
        // coil address + 2 bytes number of coil to write + 1 byte number of
        // data bytes. Each data byte contains values of 8 coils. CRC will be
        // applied later if required by Modbus ADU type.
        command[0] = deviceAddress;
        command[1] = (byte) 15;
        command[2] = (byte) ((firstCoilAddress >> 8) & 0xFF);
        command[3] = (byte) (firstCoilAddress & 0xFF);
        command[4] = (byte) ((newCoilsValues.size() >> 8) & 0xFF);
        command[5] = (byte) (newCoilsValues.size() & 0xFF);
        command[6] = (byte) dataBytesNum;
        for (int dataByteNum = 0; dataByteNum < dataBytesNum; dataByteNum++) {
            for (int coilNum = 0; coilNum < 8; coilNum++) {
                if ((dataByteNum * 8) + coilNum < newCoilsValues.size()) {
                    if (newCoilsValues.get((dataByteNum * 8) + coilNum) > 0) {
                        command[7 + dataByteNum] ^= 1 << coilNum;
                    }
                } else {
                    break;
                }
            }
        }
        return convertCommandToModbusType(command, modbusType);
    }

    /**
     * <p>
     * Creates skeleton of simple 6-byte Modbus RTU command
     * </p>
     * <p>
     * For example to read value of 3 coils starting from coil 2 in device with
     * address 10 write: prepareRTU6ByteCommand(10,1,2,3); where 1 is read coil
     * status modbus command
     * </p>
     *
     * @param deviceAddress Address of device where command will be executed
     * (slave device)
     * @param command Modbus command number (see Modbus reference)
     * @param address First command argument, usually address of register or
     * coil
     * @param value Second command argument, usually number of registers or
     * coils
     * @return Skeleton of command with filled arguments without CRC (Address +
     * PDU block)
     */
    public static byte[] prepareRTU6ByteCommand(byte deviceAddress, byte command, int address, int value) {
        byte[] simpleCommand = new byte[6];
        simpleCommand[0] = deviceAddress;
        simpleCommand[1] = command;
        simpleCommand[2] = (byte) ((address >> 8) & 0xFF);
        simpleCommand[3] = (byte) (address & 0xFF);
        simpleCommand[4] = (byte) ((value >> 8) & 0xFF);
        simpleCommand[5] = (byte) (value & 0xFF);
        return simpleCommand;
    }

    /**
     * <p>
     * Applies CRC-16-ANSI to given command
     * </p>
     * <p>
     * Command must contain PDU (function code and function value) and device
     * address
     * </p>
     *
     * @param preparedCommand device address and PDU
     *
     * @return byte[] command with CRC-16-ANSI applied in last two bytes.
     */
    public static byte[] applyCRC(byte[] preparedCommand) {
        return CRC16ANSI.applyCRC(preparedCommand);
    }

    /**
     * <p>
     * Applies ASCII LRC to given command
     * </p>
     * <p>
     * Command must contain PDU (function code and function value) and device
     * address
     * </p>
     *
     * @param preparedCommand device address and PDU
     *
     * @return byte[] command with ASCII LRC applied in last byte.
     */
    public static byte[] applyLRC(byte[] preparedCommand) {
        byte[] commandWithLRC = new byte[preparedCommand.length + 1];
        int sum = 0;
        for (int i = 0; i < preparedCommand.length; i++) {
            sum = sum + preparedCommand[i]; // To calculate LRC we need to sum
            // all command's bytes
            commandWithLRC[i] = preparedCommand[i];
        }
        sum *= -1; // and then make it negative
        commandWithLRC[commandWithLRC.length - 1] = (byte) sum;
        return commandWithLRC;
    }

    /**
     * Checks provided device reply for errors: overall validity, exception
     * codes and CRC/LRC errors
     *
     * @param reply whole reply from device
     * @param modbusType defines Modbus ADU type, device is operating on.
     * @return 0 if no errors where found, or error response code.
     */
    public static int checkReply(byte[] reply, int modbusType) {
        if (reply == null || reply.length < 3
                || ((modbusType == TYPE_MODBUS_RTU || modbusType == TYPE_MODBUS_OVER_TCP) && reply.length < 4)
                || (modbusType == TYPE_MODBUS_ASCII && reply.length < 6)) {
            // minimal content is addr+value+crc1+crc2
            return RESPONSE_NO_DATA;
        } else if ((reply[1] & 0xFF) > 128) { // If reply's 2nd byte > 128 (128+function code)
            return reply[2]; // then reply's 3rd byte contains exception
            // code
        } else if (modbusType == TYPE_MODBUS_RTU || modbusType == TYPE_MODBUS_OVER_TCP) {
            if (checkCRC(reply)) {
                return 0;
            } else {
                return RESPONSE_CRC_ERROR;
            }
        } else if (modbusType == TYPE_MODBUS_ASCII) {
            if (reply[0] == 0x3a) {
                if (checkLRC(reply)) {
                    return 0;
                } else {
                    return RESPONSE_CRC_ERROR;
                }
            } else {
                return RESPONSE_NO_DATA;
            }
        } else {
            return 0;
        }
    }

    /**
     * Checks whether provided reply has valid length and contains valid CRC
     *
     * @param reply device reply to check
     *
     * @return true if CRC is valid, false if CRC is not valid
     */
    public static boolean checkCRC(byte[] reply) {
        if (reply == null || reply.length < 4) {
            // minimal content is addr+value+crc1+crc2
            return false;
        } else {
            return CRC16ANSI.checkCRC(reply);
        }
    }

    /**
     * Checks whether provided ASCII reply contains valid LRC
     *
     * @param reply device reply to check
     *
     * @return true if LRC is valid, false if LRC is not valid
     */
    public static boolean checkLRC(byte[] reply) {
        if (reply == null || reply.length < 6) {
            // minimal content is ':'+addr+value+lrc+cr+lf
            return false;
        } else {
            byte[] replyCore = Arrays.copyOfRange(reply, 1, reply.length - 3); //cut ':', LRC, cr and lf
            byte[] replyWithGoodLRC = applyLRC(replyCore);
            return Arrays.equals(reply, replyWithGoodLRC);
        }
    }

    /**
     * <p>
     * Converts given command to ModBus ASCII format
     * </p>
     * <p>
     * Command must contain PDU (function code and function value) and device
     * address
     * </p>
     *
     * @param preparedCommand device address and PDU
     *
     * @return byte[] ModBus ASCII command.
     */
    public static byte[] convertToASCII(byte[] preparedCommand) {
        byte[] commandWithLRC = applyLRC(preparedCommand);
        byte[] commandASCII = new byte[commandWithLRC.length * 2 + 3];
        commandASCII[0] = 0x3a;
        for (int i = 0; i < commandWithLRC.length; i++) {
            String s = String.format("%02X", commandWithLRC[i]);
            commandASCII[i * 2 + 1] = s.getBytes()[0];
            commandASCII[i * 2 + 2] = s.getBytes()[1];
        }
        commandASCII[commandASCII.length - 2] = 0x0d;
        commandASCII[commandASCII.length - 1] = 0x0a;
        return commandASCII;
    }

    /**
     * Adds MBAP Header to provided prepared command. Unit Identifier is taken
     * from deviceAddress field of command. Note that this function allways sets
     * CONTENT LENGTH field in MBAP. If you don't need this field for
     * fixed-length commands, then use convertToTCPNoFixedLength instead. For
     * more information see descriptions of TYPE_MODBUS_TCP_UDP and
     * TYPE_MODBUS_TCP_UDP_NO_FIXED_LENGTHS
     *
     * @param preparedCommand command to add MBAP to
     * @return MBAP header + command
     */
    public static byte[] convertToTCP(byte[] preparedCommand) {
        tcpID++;
        byte[] packetN = new byte[2];
        packetN[1] = (byte) (tcpID & 0xFF);
        packetN[0] = (byte) ((tcpID >> 8) & 0xFF);
        int lengthInt = preparedCommand.length;
        byte[] length = new byte[2];
        length[1] = (byte) (lengthInt & 0xFF);
        length[0] = (byte) ((lengthInt >> 8) & 0xFF);
        byte[] commandWithMBAP = new byte[preparedCommand.length + 6];
        commandWithMBAP[0] = packetN[0]; //Set transaction ID
        commandWithMBAP[1] = packetN[1];
        commandWithMBAP[2] = 0x00; //Set Modbus proto ID=0
        commandWithMBAP[3] = 0x00;
        commandWithMBAP[4] = length[0]; //Set command length
        commandWithMBAP[5] = length[1];
        System.arraycopy(preparedCommand, 0, commandWithMBAP, 6, preparedCommand.length);
        return commandWithMBAP;
    }

    /**
     * Adds MBAP Header to provided prepared command. Unit Identifier is taken
     * from deviceAddress field of command. Note that this function sets CONTENT
     * LENGTH field in MBAP only for variable-length commands or commands with
     * variable length responses (Actually all but 5 and 6 function codes). If
     * you need this field for every command, then use convertToTCP instead. For
     * more information see descriptions of TYPE_MODBUS_TCP_UDP and
     * TYPE_MODBUS_TCP_UDP_NO_FIXED_LENGTHS
     *
     * @param preparedCommand command to add MBAP to
     * @return MBAP header + command
     */
    public static byte[] convertToTCPNoFixedLength(byte[] preparedCommand) {
        if (preparedCommand[1] == 5 || preparedCommand[1] == 6) {
            tcpID++;
            byte[] packetN = new byte[2];
            packetN[1] = (byte) (tcpID & 0xFF);
            packetN[0] = (byte) ((tcpID >> 8) & 0xFF);
            byte[] commandWithMBAP = new byte[preparedCommand.length + 4];
            commandWithMBAP[0] = packetN[0]; //Set transaction ID
            commandWithMBAP[1] = packetN[1];
            commandWithMBAP[2] = 0x00; //Set Modbus proto ID=0
            commandWithMBAP[3] = 0x00;
            System.arraycopy(preparedCommand, 0, commandWithMBAP, 4, preparedCommand.length);
            return commandWithMBAP;
        } else {
            return convertToTCP(preparedCommand);
        }

    }

    /**
     * Strips device reply to data only, removing device address, function code,
     * CRC and TCP/UDP or ASCII overhead. NOTE Null will be returned if there
     * was an error code or reply was too short.
     *
     * @param reply Full reply from device
     * @param typeModbus Type of Modbus ADU
     * @return Only data (without device address, function code, CRC and TCP/UDP
     * or ASCII overhead) OR null if there was an error code or reply was too
     * short
     */
    public static byte[] stripReplyToDATA(byte[] reply, int typeModbus) {
        byte[] strippedPDU = stripReplyToPDU(reply, typeModbus);
        if (strippedPDU != null && strippedPDU.length > 1 && (strippedPDU[1] & 0xff) < 128) {
            if (strippedPDU[1]==5 || strippedPDU[1] ==6){
                byte[] strippedData = Arrays.copyOfRange(strippedPDU, 2, strippedPDU.length);   
                return strippedData;
            } else if (strippedPDU.length>3){
                byte[] strippedData = Arrays.copyOfRange(strippedPDU, 3, strippedPDU.length);   
                return strippedData;
            } else {
                return null;
            }           
        } else {
            return null;
        }
    }

    /**
     * Strips device reply to address + PDU, removing CRC and TCP/UDP or ASCII
     * overhead.
     *
     * @param reply Full reply from device
     * @param typeModbus Type of Modbus ADU
     * @return address + PDU without CRC and TCP/UDP or ASCII overhead.
     */
    public static byte[] stripReplyToPDU(byte[] reply, int typeModbus) {
        if (typeModbus == TYPE_MODBUS_TCP_UDP && reply != null && reply.length > 6) {
//            if (reply.length > 7 && (reply[7] == 1 || reply[7] == 2 || reply[7] == 3 || reply[7] == 4 || reply[7] == 15 || reply[7] == 16)) {
//                byte[] stripped = Arrays.copyOfRange(reply, 6, reply.length);
//                return stripped;
//            } else if (reply.length > 5 && (reply[5] == 5 || reply[5] == 6)) {
//                byte[] stripped = Arrays.copyOfRange(reply, 4, reply.length);
//                return stripped;
//            } else {
//                return null;
//            }
            
                byte[] stripped = Arrays.copyOfRange(reply, 6, reply.length);
                return stripped;
                   
        } else if (typeModbus == TYPE_MODBUS_RTU && reply != null && reply.length > 2) {
            byte[] stripped = Arrays.copyOfRange(reply, 0, reply.length - 2);
            return stripped;
        } else if (typeModbus == TYPE_MODBUS_OVER_TCP && reply != null && reply.length > 8) {
            byte[] stripped = Arrays.copyOfRange(reply, 6, reply.length - 2);
            return stripped;
        } else if (typeModbus == TYPE_MODBUS_ASCII && reply != null && reply.length > 4) {
            byte[] stripped = Arrays.copyOfRange(reply, 1, reply.length - 3);
            return stripped;
        } else {
            return null;
        }
    }

    //NOT IMPLEMENTED YET
//    public static List<Integer> getValuesFromReply(byte[] reply, int bytesInReg) {
//        if (reply == null || reply.length < 5 || (reply[1] & 0xFF) > 128) {
//            return null;
//        } else {
//            
//            List<Integer> values = new ArrayList<>();
//            return values;
//        }
//    }
    
    public static Integer getFromReplyUINT16(byte[] reply, int typeModbus){
        byte[] strippedToData = stripReplyToDATA(reply, typeModbus);
        if (strippedToData != null && strippedToData.length>1){
            return ((strippedToData[0] & 0xff) << 8) | (strippedToData[1] & 0xff);
        } else {
            return null;
        }
    }

    /**
     * @return Current Identification of a MODBUS Request transaction.
     */
    public static int getTcpID() {
        return tcpID;
    }

    /**
     * Basically this library carries one identificator and increases it with
     * every call to convertToTCP (internally called in every command generator,
     * when Modbus type is set to Modbus-TCP/UDP). So if you have multiple
     * Modbus-TCP/UDP devices, they will share this id (for example if you have
     * 3 devices, that are requested in order, then first packet to device1 will
     * have id 1, packet to device2 will have id 2, packet to device3 will have
     * id 3). In most cases this works ok because device, due to Modbus
     * standard, don't perfom id check and only copies it in reply, but if you
     * want separate ids for each device, you have to implement your own list of
     * device addresses and take care of exact id increment every time you
     * prepare a command for device, then you can force id with this function
     * (note that it is static, so if you have multiple threads you have to
     * synchronize your double-call transaction - first set id, than generate
     * command) or you can simply overwrite id with your own in generated
     * command.
     *
     * @param tcpID New Identification of a MODBUS Request transaction for next
     * generated command.
     */
    public static void setTcpID(int tcpID) {
        ModBus.tcpID = tcpID;
    }

    public static byte[] convertCommandToModbusType(byte[] command, int modbusType) {
        switch (modbusType) {
            case TYPE_MODBUS_RTU:
                return applyCRC(command);            
            case TYPE_MODBUS_TCP_UDP:
                return convertToTCP(command);
            case TYPE_MODBUS_OVER_TCP:
                return convertToTCP(applyCRC(command));
            case TYPE_MODBUS_ASCII:
                return convertToASCII(command);
            default:
                throw new UnsupportedOperationException("Not implemented yet");
        }
    }

    public static void main(String[] args) {
        if (args != null && args.length > 3) {
            byte deviceAddress = 1;
            byte command = 3;
            int address = 2;
            List<Integer> values = new ArrayList<>();

            try {
                deviceAddress = Byte.parseByte(args[0]);
            } catch (NumberFormatException e) {
                System.out.println("Device address must be byte! Setting default value 1");
            }
            try {
                command = Byte.parseByte(args[1]);
            } catch (NumberFormatException e) {
                System.out.println("Command must be byte! Setting default value 3");
            }
            try {
                address = Integer.parseInt(args[2]);
            } catch (NumberFormatException e) {
                System.out.println("PDU address must be int! Setting default value 2");
            }
            try {
                for (int i = 3; i < args.length; i++) {
                    int value = Integer.parseInt(args[i]);
                    values.add(value);
                }
            } catch (NumberFormatException e) {
                System.out.println("PDU data value must be int! Setting default value 4");
                values.add(4);
            }

            byte[] result = null;
            if (command == 15) {
                result = writeValueOfMultipleCoils(deviceAddress, address, values, TYPE_MODBUS_RTU);
                result = stripReplyToPDU(result, TYPE_MODBUS_RTU);
            } else if (command == 16) {
                result = writeValueOfMultipleHoldingRegisters(deviceAddress, address, values, TYPE_MODBUS_RTU);
                result = stripReplyToPDU(result, TYPE_MODBUS_RTU);
            } else {
                result = prepareRTU6ByteCommand(deviceAddress, command, address, values.get(0));
            }
            StringBuilder sbHexTCP = new StringBuilder("TCP HEX: ");
            StringBuilder sbDecTCP = new StringBuilder("TCP DEC: ");
            byte[] tcp = convertToTCP(result);
            for (byte b : tcp) {
                sbHexTCP.append(String.format("%02X ", b & 0xff));
                sbDecTCP.append(String.format("%02d ", b & 0xff));
            }
            byte[] ascii = convertToASCII(result);
            StringBuilder sbHexASCII = new StringBuilder("ASCII HEX: ");
            StringBuilder sbStringASCII = new StringBuilder("ASCII String: ");
            for (byte b : ascii) {
                sbHexASCII.append(String.format("%02X ", b & 0xff));
            }
            byte[] asciiWithoutCRLF = new byte[ascii.length - 2];
            for (int i = 0; i < ascii.length - 2; i++) {
                asciiWithoutCRLF[i] = ascii[i];
            }
            sbStringASCII.append(new String(asciiWithoutCRLF));
            sbStringASCII.append("<CR>");
            sbStringASCII.append("<LF>");
            result = applyCRC(result);
            StringBuilder sbHexRTU = new StringBuilder("RTU HEX: ");
            StringBuilder sbDecRTU = new StringBuilder("RTU DEC: ");
            for (byte b : result) {
                sbHexRTU.append(String.format("%02X ", b & 0xff));
                sbDecRTU.append(String.format("%02d ", b & 0xff));
            }
            System.out.println(sbHexRTU.toString());
            System.out.println(sbDecRTU.toString());
            System.out.println(sbHexTCP.toString());
            System.out.println(sbDecTCP.toString());
            System.out.println(sbHexASCII.toString());
            System.out.println(sbStringASCII.toString());
        } else {
            printHelp();
        }
    }

    private static void printHelp() {
        System.out.println("Usage: java -jar ModBus.jar A C a v [v2 v3 ...]");
        System.out.println("Where:");
        System.out.println("A (byte) - slave device address");
        System.out.println("C (byte) - command - ModBus function code:");
        System.out.println("  1 - Read coils values");
        System.out.println("  2 - Read inputs values");
        System.out.println("  3 - Read values of holding registers");
        System.out.println("  4 - Read values of input registers");
        System.out.println("  5 - Writes value of single coil");
        System.out.println("  6 - Writes value of single holding register");
        System.out.println("  15 - Writes values of multiple coils");
        System.out.println("  16 - Writes values of multiple holding registers");
        System.out.println("a (int) - PDU address value, for example address of first register to write");
        System.out.println("v (int) - PDU data value, for example amount of registers to write");
        System.out.println("For functions 15 and 16 you can specify several values separated by space");
        System.out.println("---------------------------------");
        System.out.println("Example: java -jar ModBus.jar 10 2 6 3");
        System.out.println("read inputs values from inputs 6,7,8 from slave device 10");
        System.out.println("Example: java -jar ModBus.jar 1 16 2 10 20 30 40 2");
        System.out.println(
                "set values of holding registers in device 1. Register 2 gets value 10, register 3 gets value 20, register 4 gets value 30, register 5 gets value 40");
    }

    /**
     * CRC-16-ANSI Table for CRC calculations
     */
    private static final int[] CRCTable = {0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601,
        0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1,
        0xce81, 0x0e40, 0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980,
        0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
        0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100,
        0xd1c1, 0xd081, 0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240, 0x3600, 0xf6c1,
        0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80,
        0xfe41, 0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981, 0x2940,
        0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401,
        0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640, 0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0,
        0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240, 0x6600, 0xa6c1, 0xa781,
        0x6740, 0xa501, 0x65c0, 0x6480, 0xa441, 0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
        0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01,
        0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0,
        0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080,
        0xb041, 0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741,
        0x5500, 0x95c1, 0x9481, 0x5440, 0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 0x5a00,
        0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1,
        0x8a81, 0x4a40, 0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 0x4400, 0x84c1, 0x8581,
        0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040};
}
