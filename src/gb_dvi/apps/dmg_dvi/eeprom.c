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
#include "pico/multicore.h"
#include "pico/stdio.h"

#define FLAG_VALID_EEPROM           (0xA5)

// Reserve the last flash sector for settings. 
// PICO_FLASH_SIZE_BYTES should be set in CMakeLists.txt (0x200000 for 2MB)
// otherwise it may default to 0x400000 (4MB) using the default linker script.
#define FLASH_OFFSET_SECTOR  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define FLASH_OFFSET_PAGE    (FLASH_OFFSET_SECTOR + FLASH_SECTOR_SIZE - FLASH_PAGE_SIZE) // last page in that sector
const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_OFFSET_PAGE);

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
        // If uninitialized, seed the in-RAM buffer but defer flash writes until first commit.
        for (i = 0; i < FLASH_PAGE_SIZE; i++)
        {
            _data[i] = 0;
        }
        _data[FLASH_PAGE_SIZE-1] = FLAG_VALID_EEPROM;
        _dirty = false;
        return;
    }

    for (i = 0; i < FLASH_PAGE_SIZE; i++)
    {
       _data[i] = flash_target_contents[i];
    }
    
}

static void __no_inline_not_in_flash_func(_clear_flash)()
{
    uint32_t i;
    for (i = 0; i < FLASH_PAGE_SIZE; i++)
    {
        _data[i] = 0;
    }
    _data[FLASH_PAGE_SIZE-1] = FLAG_VALID_EEPROM;
    _dirty = true;
    (void)EEPROM_commit();
}

// **************************************************************
// PUBLIC FUNCTIONS
// **************************************************************
eeprom_result_t EEPROM_read(uint8_t address, uint8_t *value)
{
    if (!init)
        _load();

    if (address >= FLASH_PAGE_SIZE) {
        return EEPROM_FAILURE;
    }

    *value = _data[address];
    return EEPROM_SUCCESS;
}

eeprom_result_t EEPROM_write(uint8_t address, uint8_t const value) 
{
    if (!init)
        _load();

    if (address >= FLASH_PAGE_SIZE)
        return EEPROM_FAILURE;

    // Optimise _dirty. Only flagged if data written is different.
    if (_data[address] != value)
    {
        _data[address] = value;
        _dirty = true;
    }
    return EEPROM_SUCCESS;
}

// Must run from RAM while flash is busy
eeprom_result_t __no_inline_not_in_flash_func(EEPROM_commit)(void) 
{
    if (!init)
        _load();

    if (!_dirty)
        return EEPROM_SUCCESS;

    // Not really necessary in this implementation, but keeping around
    if (!_data)
        return EEPROM_FAILURE;

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_OFFSET_SECTOR, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_OFFSET_PAGE, _data, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    _dirty = false;

    // EEPROM_print_buffer();

    return EEPROM_SUCCESS;
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