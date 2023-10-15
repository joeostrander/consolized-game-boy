// It's not really eeprom, but emulated using flash :)
// The cake is a lie

#ifndef EEPROM_H
#define EEPROM_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

uint8_t EEPROM_read(uint8_t address);
void EEPROM_write(uint8_t address, uint8_t const value);
bool EEPROM_commit(void);
void EEPROM_print_buffer(void);
void EEPROM_clear_all(void);

#endif // EEPROM_H