#include <EEPROM.h>

// Write string to eeprom
void eeprom_write_string(int address, const char* sourceStr);
void eeprom_read_string(int address, char* targetStr, int maxlen);
void eeprom_write_int32(int address, int32_t val);
int32_t eeprom_read_int32(int address);