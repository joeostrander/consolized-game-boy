// This is a modified version of the original
// Converted to C and added some extra stuff (disable interrupts, etc.)
// Joe Ostrander

/*
    EEPROM.cpp - RP2040 EEPROM emulation
    Copyright (c) 2021 Earle F. Philhower III. All rights reserved.

    Based on ESP8266 EEPROM library, which is
    Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "eeprom.h"
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <stdio.h>
#include "flash_utils.h"


// I'm storing data in the last page of the last sector
// A custom linker file ensures that the sector will always be defined and not overwritten by new firmware
// Thanks to Jan Cumps from Element14 post:
// https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pico-c-sdk-reserve-a-flash-memory-block-for-persistent-storage
#define FLASH_ADDRESS_SECTOR        (PICO_FLASH_SIZE_BYTES-FLASH_SECTOR_SIZE)
#define FLASH_ADDRESS_PAGE          (PICO_FLASH_SIZE_BYTES-FLASH_PAGE_SIZE)
#define FLAG_VALID_EEPROM           (0xA5)

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_ADDRESS_PAGE);
static uint8_t _data[FLASH_PAGE_SIZE] = {0};

static bool _dirty = false;
static bool init = false;

// **************************************************************
// PRIVATE FUNCTION PROTOTYPES
// **************************************************************
static void _load(void);
static void _clear_flash();
static void print_buf(const uint8_t *buf, size_t len);

// **************************************************************
// PRIVATE FUNCTIONS
// **************************************************************
static void _load(void)
{
    init = true;

    uint32_t i;

    if (flash_target_contents[FLASH_PAGE_SIZE-1] != FLAG_VALID_EEPROM)
    {
        //print_buf(flash_target_contents, FLASH_PAGE_SIZE);

        _clear_flash();
    }

    for (i = 0; i < FLASH_PAGE_SIZE; i++)
    {
       _data[i] = flash_target_contents[i];
    }
    
}

static void _clear_flash()
{
    uint32_t i;
    for (i = 0; i < FLASH_PAGE_SIZE; i++)
    {
        _data[i] = 0;
    }
    _data[FLASH_PAGE_SIZE-1] = FLAG_VALID_EEPROM;
    _dirty = true;
    EEPROM_commit();
}

// **************************************************************
// PUBLIC FUNCTIONS
// **************************************************************
uint8_t EEPROM_read(uint8_t address) 
{
    if (!init)
        _load();

    if (address >= FLASH_PAGE_SIZE) {
        return 0;
    }

    return _data[address];
}

void EEPROM_write(uint8_t address, uint8_t const value) 
{
    if (!init)
        _load();

    if (address >= FLASH_PAGE_SIZE)
        return;

    // Optimise _dirty. Only flagged if data written is different.
    if (_data[address] != value)
    {
        _data[address] = value;
        _dirty = true;
    }
}

bool EEPROM_commit(void) 
{
    if (!init)
        _load();

    if (!_dirty) {
        return true;
    }

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_ADDRESS_SECTOR, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_ADDRESS_PAGE, _data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    _dirty = false;

    // EEPROM_print_buffer();

    return true;
}

static void print_buf(const uint8_t *buf, size_t len) 
{
    for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");
    }
}

void EEPROM_print_buffer(void)
{
    print_buf(flash_target_contents, FLASH_PAGE_SIZE);
}

void EEPROM_clear_all(void)
{
    _clear_flash();
}