/*
 * SerialServo.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Prahadeesh TN
 */
#include "SerialServo.h"
#include <stdio.h>
#include <stdbool.h>
#include "main.h"


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
Instruction_Packet packet;
uint8_t rx_data[Rx_DATA_SIZE];
volatile uint8_t received;
uint8_t info_array[INSTRUCTION_FRAME_BUFFER];
Instruction_Packet packet;
Status_Packet return_packet;

//WRITE FUNCTIONS
bool Ping(int ID){
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x03;
	packet.Instruction = COMMAND_PING;
	packet.Param = 0x00;
	packet.Checksum = getChecksum(packet);
	Status_Packet status =AxelFlow_fire(&huart2, packet);
	if (status.Error
			== 0&& status.Header_1 == HEADER && status.Header_2 == HEADER)
		return true;
	else
		return false;
}

void SetID (int ID){
	uint8_t param_array[2]={EEPROM_ID,ID};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = 2;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}

int findID(){
	int i=0,ID;
	for (i=0;i<=254;i++){
		if (Ping(i)){
			return i;;

		}
	}
}

void ReSetID(int Initial_ID,int New_ID){
	uint8_t param_array[2]={EEPROM_ID,New_ID};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = Initial_ID;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
	Write_LockFlag(Initial_ID,0);
    SetID(New_ID);
    Write_LockFlag(New_ID,1);
}

uint8_t Min_max_voltage(){

}

void Write_LockFlag(int ID,int flag){
	uint8_t param_array[2]={RAM_LockFlag,flag};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}


//void MEGA_ACTION(int ID[],int pos[],int time[], int speed[],UART_HandleTypeDef *huart){
//    unsigned char lowByte_p, highByte_p;
//    unsigned char lowByte_t, highByte_t;
//    unsigned char lowByte_s, highByte_s;
//
//
//	uint8_t param_array[sizeof(ID)+sizeof(pos)+sizeof(time)+sizeof(speed)+2];
//	int i,j=0;
//	param_array[0]=TARGET_LOCATION ;
//	param_array[1]=0x06;
//	for (i=2;i<((sizeof(ID)+sizeof(pos)+sizeof(time)+sizeof(speed))+2);i+=7){
//		param_array[i] = ID[j];
//		int_to_hex(pos[j],&lowByte_p,&highByte_p);
//		param_array[i+1] = lowByte_p;
//		param_array[i+2]= highByte_p;
//		int_to_hex(time[j],&lowByte_t,&highByte_t);
//		param_array[i+3] = lowByte_t;
//		param_array[i+4] = highByte_t;
//		int_to_hex(speed[j],&lowByte_s ,&highByte_s);
//		param_array[i+5] = lowByte_s;
//		param_array[i+6] = highByte_s;
//		j++;
//
//	}
//	packet.Header_1 = HEADER;
//	packet.Header_2 = HEADER;
//	packet.Packet_ID = 0xFE;
//	packet.Length = sizeof(param_array)+2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	packet.Checksum = getChecksum(packet);
//    AxelFlow_fire(&huart2, packet);
//}

//void Sync_write(SyncWrite_Packet write_packet){
//    unsigned char lowByte_p, highByte_p;
//    unsigned char lowByte_t, highByte_t;
//    unsigned char lowByte_s, highByte_s;
//
//
//	uint8_t param_array[sizeof(write_packet.ID)+sizeof(write_packet.pos)+sizeof(write_packet.speed)+sizeof(write_packet.time)+2];
//	int i;
//	param_array[0]=0x2A;
//	param_array[1]=0x06;
//	int count=2;
//	for (i=0;i<(sizeof(write_packet.ID));i++){
//		int_to_hex(write_packet.pos[i],&lowByte_p,&highByte_p);
//		int_to_hex(write_packet.time[i],&lowByte_t,&highByte_t);
//		int_to_hex(write_packet.speed[i],&lowByte_s,&highByte_s);
//		param_array[count++] = write_packet.ID[i];
//		param_array[count++]= lowByte_p;
//		param_array[count++]= highByte_p;
//		param_array[count++]= 0;
//		param_array[count++]= 0;
//		param_array[count++]= lowByte_s;
//		param_array[count++]= highByte_s;
//	}
////	HAL_UART_Transmit(&huart1, param_array, sizeof(param_array),HAL_MAX_DELAY);
//	packet.Header_1 = HEADER;
//	packet.Header_2 = HEADER;
//	packet.Packet_ID = 0xFE;
//	packet.Length = sizeof(param_array)+2;
//	packet.Instruction = COMMAND_SYNC_WRITE;
//	packet.Param = param_array;
//	packet.Checksum = getChecksum(packet);
//    AxelFlow_fire(&huart2, packet);
//}
void Sync_write(SyncWrite_Packet write_packet) {
    unsigned char lowByte_p, highByte_p;
    unsigned char lowByte_t, highByte_t;
    unsigned char lowByte_s, highByte_s;


    int num_servos = sizeof(write_packet.ID)-2;


    int param_length = 2 + num_servos * (7);
    uint8_t param_array[param_length];


    param_array[0] = 0x2A;
    param_array[1] = 0x06;

    int count = 2;
    for (int i = 0; i < num_servos; i++) {

        int_to_hex(write_packet.pos[i], &lowByte_p, &highByte_p);
        int_to_hex(write_packet.time[i], &lowByte_t, &highByte_t);
        int_to_hex(write_packet.speed[i], &lowByte_s, &highByte_s);


        param_array[count++] = write_packet.ID[i];
        param_array[count++] = lowByte_p;
        param_array[count++] = highByte_p;
        param_array[count++] = lowByte_t;
        param_array[count++] = highByte_t;
        param_array[count++] = lowByte_s;
        param_array[count++] = highByte_s;
    }
    packet.Header_1 = 0xFF;
    packet.Header_2 = 0xFF;
    packet.Packet_ID = 0xFE;
    packet.Length = param_length + 2;
    packet.Instruction = COMMAND_SYNC_WRITE;
    packet.Param = param_array;
    packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet); // Send packet via UART
}

//void sync_Read(SyncRead_Packet read_packet){  //READS ONLY CURRENT LOCATION CHANGE PARAM ARRAY[0] TO READ ANY OTHER THING(have to change other stuff related to that too)
//    unsigned char lowByte_p, highByte_p;
//    unsigned char lowByte_t, highByte_t;
//    unsigned char lowByte_s, highByte_s;
//
//    int param_length =sizeof(read_packet.ID);
//    uint8_t param_array[param_length];
//
//
//    int num_servos = sizeof(write_packet.ID)-2;
//    for(i=0;i<param_length;i++){
//    	param_array[i]= read_packet.ID[i];
//    }
//
//
//    param_array[0] = RAM_Current_location;
//    param_array[1] = 0x02;
//
//
//    int count = 2;
//    packet.Header_1 = 0xFF;
//    packet.Header_2 = 0xFF;
//    packet.Packet_ID = 0xFE;
//    packet.Length = param_length + 2;
//    packet.Instruction = COMMAND_SYNC_READ;
//    packet.Param = param_array;
//    packet.Checksum = getChecksum(packet);
//    return_packet = AxelFlow_fire(&huart2, packet);
//}

void  ID_loc_time_speed(int ID,int pos,int time,int speed){
    unsigned char lowByte_p, highByte_p;
	int_to_hex(pos,&lowByte_p,&highByte_p);
    unsigned char lowByte_t, highByte_t;
	int_to_hex(time,&lowByte_t,&highByte_t);
    unsigned char lowByte_s, highByte_s;
	int_to_hex(speed,&lowByte_s ,&highByte_s);
	uint8_t param_array[7]={TARGET_LOCATION,lowByte_p,highByte_p,lowByte_t,highByte_t,lowByte_s,highByte_s};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x09;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}

void Target_location(int pos){
    unsigned char lowByte, highByte;
	int_to_hex(pos,&lowByte,&highByte);
	uint8_t param_array[3]={TARGET_LOCATION,lowByte,highByte};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x05;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}

void Operation_time(int time){
    unsigned char lowByte, highByte;
	int_to_hex(time,&lowByte,&highByte);
	uint8_t param_array[3]={OPERATION_TIME,lowByte,highByte};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x05;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}
void POSITION_CORRECTION (int pos){
    unsigned char lowByte, highByte;
	int_to_hex(pos,&lowByte,&highByte);
	uint8_t param_array[3]={EEPROM_POSITION_CORRECTION,lowByte,highByte};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x05;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}
void Operation_speed(int speed){
    unsigned char lowByte, highByte;
	int_to_hex(speed,&lowByte,&highByte);
	uint8_t param_array[3]={OPERATION_SPEED,lowByte,highByte};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x05;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}

void Operation_mode(int ID,int mode){
	uint8_t param_array[2]={OPERATION_MODE ,mode};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
	Write_LockFlag(ID,0);
    AxelFlow_fire(&huart2, packet);
    Write_LockFlag(ID,1);
}

void Set_torque(int value){
    unsigned char lowByte, highByte;
	int_to_hex(value,&lowByte,&highByte);
	uint8_t param_array[4]={RAM_TORQUE_LIMIT ,lowByte,highByte};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x06;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}

void Min_Max_Angle(int min,int max){
    unsigned char lowByte, highByte;
	int_to_hex(min,&lowByte,&highByte);
	uint8_t param_array[3]={EEPROM_ANGLE_LIMIT_MIN,lowByte,highByte};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x05;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
    unsigned char lowByte1, highByte1;
	int_to_hex(max,&lowByte1,&highByte1);
	uint8_t param_array2[3]={EEPROM_ANGLE_LIMIT_MAX,lowByte1,highByte1};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = id;
	packet.Length = 0x05;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array2;
	packet.Checksum = getChecksum(packet);
    AxelFlow_fire(&huart2, packet);
}

//READ FUNTIONS

int GetMoveFlag(int ID){
	uint8_t param_array[2]={RAM_MOVE_FLAG,0x01};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    return_packet = AxelFlow_fire(&huart2, packet);
    int32_t grip;
	grip = convertHexToInteger(return_packet.Param, 1);
    return grip;
}
int LockFlag(int ID){
	uint8_t param_array[2]={RAM_LockFlag,0x01};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    return_packet = AxelFlow_fire(&huart2, packet);
    int32_t grip;
	grip = convertHexToInteger(return_packet.Param, 1);
    return grip;
}
int CurrentLocation(int ID){
	uint8_t param_array[2]={RAM_Current_location,0x02};
	packet.Header_1 = HEADER;
	packet.Header_2 = HEADER;
	packet.Packet_ID = ID;
	packet.Length = 0x04;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	packet.Checksum = getChecksum(packet);
    return_packet = AxelFlow_fire(&huart2, packet);
    int32_t pos;
	pos = convertHexToInteger(return_packet.Param, 2);
	return pos;
}


void struct_to_arr(Instruction_Packet packet)
{
	info_array[0] = HEADER;
	info_array[1] = HEADER;
	info_array[2] = packet.Packet_ID;
	info_array[3] = packet.Length;
	info_array[4] = packet.Instruction;

	for (uint8_t i = 5; i < packet.Length + 3; i++)
		info_array[i] = packet.Param[i - 5];

	info_array[packet.Length + 3] = packet.Checksum;

	for (uint8_t i = packet.Length + 4; i <= INSTRUCTION_FRAME_BUFFER; i++)
	{
		info_array[i] = 0;
	}
}


int32_t convertHexToInteger(unsigned char hexBytes[], int byteCount) {
	uint16_t result = 0;

    for (int i = 0; i < byteCount; i++) {
        result |= ((int32_t)hexBytes[i] << (i * 8));
    }
    return result;
}
void int_to_hex(int decimalNumber, unsigned char *lowByte, unsigned char *highByte) {
    uint16_t hex_value;

    // If the value is negative, set the 15th bit
    if (decimalNumber < 0) {
        hex_value = (uint16_t)(-decimalNumber);  // Take the absolute value
        hex_value |= 0x8000;                    // Set the 15th bit for negative direction
    } else {
        hex_value = (uint16_t)decimalNumber;
    }

    // Convert to little-endian format
    *lowByte = hex_value & 0xFF;          // Lower byte
    *highByte = (hex_value >> 8) & 0xFF; // Higher byte
}


Status_Packet arr_to_struct(uint8_t array[])
{
	Status_Packet packet;
	packet.Header_1 = array[0];
	packet.Header_2 = array[1];
	packet.Packet_ID = array[2];
	packet.Length = array[3];
	packet.Error = array[4];
	uint8_t prm[packet.Length - 2];
	for (uint8_t i = 5; i < packet.Length + 3; i++)
	{
		prm[i - 5] = array[i];
	}
	packet.Param = prm;
	packet.Checksum = array[packet.Length + 3];

	return packet;
}



Status_Packet AxelFlow_fire(UART_HandleTypeDef *huart, Instruction_Packet ip)
{
	HAL_StatusTypeDef err1, err2;
#ifndef DEBUG_PRINT_COMMUNICATION
	(void) err1, (void) err2; // silence warnings
#endif
	uint8_t Status_array[STATUS_FRAME_BUFFER];
	memset(Status_array, 0, STATUS_FRAME_BUFFER);
	struct_to_arr(ip);

	UART_HandleTypeDef huart2 = *huart;

	err1 = HAL_UART_Transmit(&huart2, info_array, ip.Length + 4, HAL_MAX_DELAY);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Enable receive interrupt after transmission
	err2 = HAL_UART_Receive(&huart2, Status_array, STATUS_FRAME_BUFFER,
	STATUS_PACKET_TIMEOUT);
	uint8_t Start_Index = 0;
	for (uint8_t i = 0; i < STATUS_FRAME_BUFFER - 1; i++)
	{
		if (Status_array[i] == 0xFF && Status_array[i + 1] == 0xFF
				&& Status_array[i + 2] != 0xFF)
		{
			Start_Index = i;
			break;
		}
	}
	uint8_t Status_array_filtered[Status_array[Start_Index + 3] + 4];

	for (uint8_t i = 0; i < sizeof(Status_array_filtered); i++)
	{
		Status_array_filtered[i] = Status_array[Start_Index + i];
	}
	Status_Packet packet = arr_to_struct(Status_array_filtered);
#ifdef DEBUG_PRINT_COMMUNICATION
	if (err1 != HAL_OK || (err2 != HAL_OK && err2 != HAL_TIMEOUT))
	{
		char temp[10];
		AxelFlow_debug_println("Communication Failed :(");
		sprintf(temp, "err1: %u", err1);
		AxelFlow_debug_println(temp);
		sprintf(temp, "err2: %u", err2);
		AxelFlow_debug_println(temp);
	}
#endif
	//HAL_UART_Transmit(&huart1,Status_array_filtered, sizeof(Status_array_filtered), HAL_MAX_DELAY);
//	if(packet.Error==0x00){
//		uint8_t error[10] = "NO ERROR\n";
//		HAL_UART_Transmit(&huart1,error, sizeof(error), HAL_MAX_DELAY);
//	}
//	else if(packet.Error == VOLTAGE_ERROR){
//		uint8_t error[30] = "LOW VOLTAGE ERROR\n";
//		HAL_UART_Transmit(&huart1,error, sizeof(error), HAL_MAX_DELAY);
//	}
//	else{
//		uint8_t error[30] = "OTHERERROR\n";
//		HAL_UART_Transmit(&huart1,error, sizeof(error), HAL_MAX_DELAY);
//	}
	return packet;

	// TODO clean out received data.
}

//Status_Packet AxelFlow_fire_multiple_return(UART_HandleTypeDef *huart, Instruction_Packet ip,int return_number)
//{
//
//	HAL_StatusTypeDef err1, err2;
//#ifndef DEBUG_PRINT_COMMUNICATION
//	(void) err1, (void) err2; // silence warnings
//#endif
//	uint8_t Status_array[STATUS_FRAME_BUFFER];
//	uint8_t Status_array_multiple_reutrn[return_number,STATUS_FRAME_BUFFER];
//	memset(Status_array, 0, STATUS_FRAME_BUFFER);
//	struct_to_arr(ip);
//
//	UART_HandleTypeDef huart2 = *huart;
//
//	err1 = HAL_UART_Transmit(&huart2, info_array, ip.Length + 4, HAL_MAX_DELAY);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Enable receive interrupt after transmission
//
//	//FOR RECIVEING MULTIPLE SERVOS INFORMATION
//	for(i=0;i<return_number;i++){
//	    err2 = HAL_UART_Receive(&huart2, Status_array, STATUS_FRAME_BUFFER,
//		STATUS_PACKET_TIMEOUT);
//	    int x;
//	    for(x=0;x<=sizeof(Status_array);x++){
//	    	 Status_array_multiple_return[i,x]=Status_array[x];
//	    }
//	}
//
//	uint8_t Start_Index = 0;
//	for (uint8_t i = 0; i < STATUS_FRAME_BUFFER - 1; i++)
//	{
//		if (Status_array[i] == 0xFF && Status_array[i + 1] == 0xFF
//				&& Status_array[i + 2] != 0xFF)
//		{
//			Start_Index = i;
//			break;
//		}
//	}
//	uint8_t Status_array_filtered[Status_array[Start_Index + 3] + 4];
//
//	for (uint8_t i = 0; i < sizeof(Status_array_filtered); i++)
//	{
//		Status_array_filtered[i] = Status_array[Start_Index + i];
//	}
//	Status_Packet packet = arr_to_struct(Status_array_filtered);
//#ifdef DEBUG_PRINT_COMMUNICATION
//	if (err1 != HAL_OK || (err2 != HAL_OK && err2 != HAL_TIMEOUT))
//	{
//		char temp[10];
//		AxelFlow_debug_println("Communication Failed :(");
//		sprintf(temp, "err1: %u", err1);
//		AxelFlow_debug_println(temp);
//		sprintf(temp, "err2: %u", err2);
//		AxelFlow_debug_println(temp);
//	}
//#endif
//	//HAL_UART_Transmit(&huart1,Status_array_filtered, sizeof(Status_array_filtered), HAL_MAX_DELAY);
////	if(packet.Error==0x00){
////		uint8_t error[10] = "NO ERROR\n";
////		HAL_UART_Transmit(&huart1,error, sizeof(error), HAL_MAX_DELAY);
////	}
////	else if(packet.Error == VOLTAGE_ERROR){
////		uint8_t error[30] = "LOW VOLTAGE ERROR\n";
////		HAL_UART_Transmit(&huart1,error, sizeof(error), HAL_MAX_DELAY);
////	}
////	else{
////		uint8_t error[30] = "OTHERERROR\n";
////		HAL_UART_Transmit(&huart1,error, sizeof(error), HAL_MAX_DELAY);
////	}
//	return packet;
//
//	// TODO clean out received data.
//}

bool checkChecksum()
{
	if (packet.Checksum == getChecksum())
		return 1;
	else
		return 0;
}
void DegtoInt(int deg){

}

uint8_t getChecksum()
{
	uint8_t sum = packet.Packet_ID + packet.Length + packet.Instruction;

	for (uint8_t i = 0; i < packet.Length - 2; i++)
	{
		sum += packet.Param[i];
	}
	return ~sum;
}
HAL_StatusTypeDef AxelFlow_debug_println(char *st)
{
	char str[200];
	sprintf(str, "%s\n\r", st);
	return HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str),
	HAL_MAX_DELAY);
}
