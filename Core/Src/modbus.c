/*
 * modbus.c
 *
 *  Created on: Oct 18, 2022
 *      Author: faruk.isiker
 */

#include "modbus.h"

#define SLAVE_ADDR 0x2
extern UART_HandleTypeDef huart3;
uint8_t Modbus_Rtu_Receive[8];
uint16_t CRC_kontrol;
uint8_t crc_control[2];
uint8_t slave_to_master[21];
uint8_t modbus_write_message=0;
static uint16_t ModRTU_CRC(uint8_t buf[], int len)
{
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}



void modbus_slave_init(void){
	  HAL_GPIO_WritePin(UART3_DIR_GPIO_Port, UART3_DIR_Pin, 0);
	  HAL_UART_Receive_DMA(&huart3,Modbus_Rtu_Receive,8);
	  HAL_Delay(3);
}
static uint8_t modbus_create_head(uint8_t func_num, uint8_t data[])
{
	int i = 0;
	int k = 0;
	slave_to_master[0] = SLAVE_ADDR;
	slave_to_master[1] = func_num;
	slave_to_master[2] = Modbus_Rtu_Receive[5]*2;
	for (i= 4; i < (slave_to_master[2]+4); i+=2) {
		slave_to_master[i] = data[k++];
	}

	return (slave_to_master[2] + 3);
}

static void modbus_transmit(uint8_t func_num, uint8_t data[])
{
	uint8_t msg_crc_index = modbus_create_head(func_num,data);
	HAL_GPIO_WritePin(UART3_DIR_GPIO_Port, UART3_DIR_Pin, 1);
	CRC_kontrol=ModRTU_CRC(slave_to_master, msg_crc_index);
	slave_to_master[msg_crc_index] = CRC_kontrol & 0xFF;
	slave_to_master[msg_crc_index+1] = (CRC_kontrol>>8) & 0xFF;
	HAL_Delay(3);
 	HAL_UART_Transmit(&huart3, slave_to_master,msg_crc_index + 2 , 100);
	memset((uint8_t*)Modbus_Rtu_Receive,0,sizeof(Modbus_Rtu_Receive));
	memset((uint8_t*)slave_to_master,0,sizeof(slave_to_master));
	HAL_Delay(3);
	HAL_GPIO_WritePin(UART3_DIR_GPIO_Port, UART3_DIR_Pin, 0);
}
void modbus_slave_loop(uint8_t func_num, uint8_t data[]){
	  CRC_kontrol=ModRTU_CRC(Modbus_Rtu_Receive, 6);
	  crc_control[1] = (CRC_kontrol>>8) & 0xFF;
	  crc_control[0] = CRC_kontrol & 0xFF;
	  if(crc_control[0] == Modbus_Rtu_Receive[6])
	  {
		  if(crc_control[1] == Modbus_Rtu_Receive[7])
		  {
			  if(Modbus_Rtu_Receive[1] > 5){//if func code is greater than 5, it is modbus write func.
				  modbus_write_message = Modbus_Rtu_Receive[5];
				  memset((uint8_t*)Modbus_Rtu_Receive,0,sizeof(Modbus_Rtu_Receive));
			  }else{		//if func code is less than 5, it is modbus read func.
				  modbus_transmit(func_num,data);

			  }
		  }
	  }
}
