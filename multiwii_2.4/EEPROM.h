#ifndef EEPROM_H_
#define EEPROM_H_
/*  extern void eeprom_read_block (void *buf, const void *addr, size_t n);//读取由指定地址开始的指定长度的EEPROM数据
    extern void eeprom_write_byte (uint8_t *addr, uint8_t val);//向指定地址写入一个字节8bit的EEPROM数据
    extern void eeprom_write_word (uint16_t *addr, uint16_t val);//向指定地址写入一个字16bit的EEPROM数据

*/

#include "config.h"
#include "def.h"
#include "stddef.h"
#define bool u8

//#define E2END STM32_FLASH_BASE+1024*STM32_FLASH_SIZE  //末尾地址
void eeprom_read_block (void *buf, void *addr, size_t n);//读取由指定地址开始的指定长度的EEPROM数据
void eeprom_write_block (void *buf, void *addr, size_t n);
void readGlobalSet();
bool readEEPROM();
void update_constants();
void writeGlobalSet(uint8_t b);
void writeParams(uint8_t b);
void LoadDefaults();
void readPLog(void);
void writePLog(void);

#if defined(GPS)
//EEPROM functions for storing and restoring waypoints

void storeWP(void);     // Stores the WP data in the wp struct in the EEPROM
bool recallWP(uint8_t); // Read the given number of WP from the eeprom, supposedly we can use this during flight.
// Returns true when reading is successfull and returns false if there were some error (for example checksum)
uint8_t getMaxWPNumber(void);  // Returns the maximum WP number that can be stored in the EEPROM, calculated from conf and plog sizes, and the eeprom size

void loadGPSdefaults(void);
void writeGPSconf(void) ;
bool recallGPSconf(void);

void EEPROM_I2C_GPIO_Config(void);
u8 AT24CXX_Check(void);
#endif

#endif /* EEPROM_H_ */
