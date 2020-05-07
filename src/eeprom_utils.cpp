#include "eeprom_utils.h"


void eeprom_write_string(int address, const char* sourceStr) {
    int pos = 0;
    uint8_t val;
    do {
        val = sourceStr[pos++];
        EEPROM.write(address++, val);
    } while (val != 0);
}

void eeprom_read_string(int address, char* targetStr, int maxlen) {
    int pos = 0;
    while (true) {
        targetStr[pos] = EEPROM.read(address+pos);
        if (targetStr[pos] == '\0')
            break;
        pos++;
    }
}

void eeprom_write_int32(int address, int32_t val) {
    uint8_t* p = (uint8_t*) &val;
    EEPROM.write(address, *p);
    EEPROM.write(address + 1, *(p + 1));
    EEPROM.write(address + 2, *(p + 2));
    EEPROM.write(address + 3, *(p + 3));
}
int32_t eeprom_read_int32(int address) {
    int val;
    uint8_t* p = (uint8_t*) &val;
    *p        = EEPROM.read(address);
    *(p + 1)  = EEPROM.read(address + 1);
    *(p + 2)  = EEPROM.read(address + 2);
    *(p + 3)  = EEPROM.read(address + 3);
    return val;
}