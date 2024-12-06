/*
 * SerialServo.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Prahadeesh TN
 */

#ifndef INC_SERIALSERVO_H_
#define INC_SERIALSERVO_H_
#include <stdio.h>
#include <stdbool.h>
#include "main.h"

bool transmitToServo();
bool checkChecksum();

uint8_t getChecksum();

#define HEADER                          0xFF
#define STATUS_PACKET_TIMEOUT           50
#define STATUS_FRAME_BUFFER             10
#define INSTRUCTION_FRAME_BUFFER        15
#define Rx_DATA_SIZE                    50

#define EEPROM_MODEL_NUMBER_L           0x
#define EEPROM_MODEL_NUMBER_H           0x
#define EEPROM_VERSION                  0x
#define EEPROM_ID                       0x05
#define EEPROM_BAUD_RATE                0x06
#define EEPROM_RETURN_DELAY_TIME        0x07
#define EEPROM_RESPONSE_STATUS_LEVEL    0x08
#define EEPROM_ANGLE_LIMIT_MIN          0x09
#define EEPROM_ANGLE_LIMIT_MAX          0x0B
#define EEPROM_LIMIT_TEMPERATURE_MAX    0x0D
#define EEPROM_LOW_LIMIT_VOLTAGE        0x0F
#define EEPROM_HIGN_LIMIT_VOLTAGE       0x0E
#define EEPROM_MAX_TORQUE_H             0x10
#define OPERATION_MODE                  0x21
#define EEPROM_POSITION_CORRECTION      0x1F
// RAM AREA
#define TARGET_LOCATION                 0x2A
#define OPERATION_SPEED                 0x2E
#define OPERATION_TIME                  0x2C
#define RAM_TORQUE_LIMIT                0x30
#define RAM_MOVE_FLAG                   0x42
#define RAM_LockFlag                        0x37




#define RAM_Current_location            0x38




#define id                              0x01
#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_SYNC_WRITE              0x83

//ERRORS
#define VOLTAGE_ERROR                   0x02
typedef struct
{
	uint8_t Header_1;
	uint8_t Header_2;
	uint8_t Packet_ID;
	uint8_t Length;
	uint8_t Instruction;
	uint8_t *Param;
	uint8_t Checksum;
} Instruction_Packet;

typedef struct
{
	uint8_t Header_1;
	uint8_t Header_2;
	uint8_t Packet_ID;
	uint8_t Length;
	uint8_t Error;
	uint8_t *Param;
	uint8_t Checksum;
} Status_Packet;

typedef struct
{
	uint8_t *ID;
	uint8_t *pos;
	uint8_t *time;
	uint8_t *speed ;
} SyncWrite_Packet;
#ifndef AXELFLOW_SERIAL_H
#define AXELFLOW_SERIAL_H

#include "stm32f0xx_hal.h"
//typedef struct
//{
//	uint8_t Header_1;
//	uint8_t Header_2;
//	uint8_t Packet_ID;
//	uint8_t Length;
//	uint8_t Info;
//	uint8_t *Param;
//	uint8_t Checksum;
//} Packet;
Status_Packet AxelFlow_fire(UART_HandleTypeDef *huart,
		Instruction_Packet ip);
bool Ping(int ID);
void ping(Instruction_Packet packet);
Status_Packet arr_to_struct(uint8_t array[]);
void struct_to_arr(Instruction_Packet packet);
void Target_location(int pos);
void Min_Max_Angle(int min,int max);
void Operation_mode(int mode);
void Operation_speed(int speed);
void Set_torque(int value);
uint8_t int_to_hex(int decimalNumber, unsigned char *lowByte, unsigned char *highByte);
void Operation_time(int time);
void POSITION_CORRECTION (int pos);
void  ID_loc_time_speed(int ID,int pos,int time,int speed);
void SetID (int ID);
void MEGA_ACTION(int ID[],int pos[],int time[], int speed[],UART_HandleTypeDef *huart);
int32_t convertHexToInteger(unsigned char hexBytes[], int byteCount);
HAL_StatusTypeDef AxelFlow_debug_println(char *st);
int CurrentLocation();
int LockFlag(int ID);
void Write_LockFlag(int ID,int flag);
#endif
#endif /* INC_SERIALSERVO_H_ */
