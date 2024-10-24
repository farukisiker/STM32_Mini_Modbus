/*
 * modbus.h
 *
 *  Created on: Oct 18, 2022
 *      Author: faruk.isiker
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "string.h"

void modbus_slave_init(void);
void modbus_slave_loop(uint8_t func_num, uint8_t data[]);
#endif /* INC_MODBUS_H_ */
