/*
 * main.h
 *
 *  Created on: 22 ago 2022
 *      Author: danie
 */

#ifndef MAIN_H_
#define MAIN_H_

#define FLASH_SECTOR 0x00004000

void main_Application(void);
void main_Bootloader(void);
// void UARTStringPut(uint32_t ui32Base, const char *pcString);

#endif /* MAIN_H_ */
