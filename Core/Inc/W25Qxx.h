/*
 * W25Qxx.h
 *
 *  Created on: Dec 27, 2024
 *      Author: nicaf
 */


#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_



void W25Q_Reset (void);
uint32_t W25Q_ReadID (void);

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);


void W25Q_Erase_Sector (uint16_t numsector);

void W25Q_Write_Clean(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
void W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

void W25Q_Write_Byte (uint32_t Addr, uint8_t data);
uint8_t W25Q_Read_Byte (uint32_t Addr);

float W25Q_Read_NUM (uint32_t page, uint16_t offset);
void W25Q_Write_NUM (uint32_t page, uint16_t offset, float data);

void W25Q_Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
void W25Q_Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);

#endif /* INC_W25QXX_H_ */
