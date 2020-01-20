/**************************************************************************//**
 * @file DataFlash.h
 * @brief AT45DBxxxD Atmel Dataflash library for Arduino.
 *
 * @par Copyright: 
 * - Copyright (C) 2010-2011 by Vincent Cruz.
 * - Copyright (C) 2011 by Volker Kuhlmann. @n
 * All rights reserved.
 *
 * @authors
 * - Vincent Cruz @n
 *   cruz.vincent@gmail.com
 * - Volker Kuhlmann @n
 *   http://volker.top.geek.nz/contact.html
 *
 * @par Description:
 * Please refer to @ref DataFlash.cpp for more informations.
 *
 * @par History:
 * - Version 1.x, 2010-2011.
 * - Version 2.0, 30 Aug 2011.
 * - Version 2.2, 29 Dec 2011.
 *
 * @par Licence: GPLv3
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. @n
 * @n
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. @n
 * @n
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef DATAFLASH_H_
#define DATAFLASH_H_

#include <inttypes.h>
#include "DataFlashSizes.h"
#include <SPI.h>

/**
 * @addtogroup AT45DBxxxD
 * @{
 **/

/**
 * @defgroup AT45_CHIP_ERASE_ENABLED Chip erase command prevention.
 * @note Will be removed once chip erase is re-implemented.
 * Datasheets are missing errata, but see AT45DB321D. Basically the
 * silicon is buggy and Atmel suggests to use block erase instead,
 * giving rise to the suspicion that sector erase doesn't work either.
 * @{
 **/
#ifdef AT45_CHIP_ERASE_ENABLED
#undef AT45_CHIP_ERASE_ENABLED
#endif
/**
 * @}
 **/

/**
 * @defgroup AT45_USE_SPI_SPEED_CONTROL SPI transfer speed control.
 * @warning This feature is experimental. Use it at your own risk!
 * %Dataflash supports low and high speed transfers. However the low
 * speed transfers are more than 3x the speed of what an Arduino
 * with ATmega 328P or 1280 can provide, so this makes no sense at
 * all with that hardware. 
 * Leaving in, in case it's useful for other hardware. Tested code.
 * Dropping this saves 86 bytes.
 * @{
 **/
#ifdef AT45_USE_SPI_SPEED_CONTROL
#undef AT45_USE_SPI_SPEED_CONTROL
#endif
/**
 * @}
 **/

/**
 * @defgroup PINOUT Default pin connections.
 * Default pin values for Chip Select (CS), Reset (RS) and
 * Write Protec (WP). 
 * Reset and Write Protect pins are not used by default.
 * @{
 **/
/** Chip select (CS) **/
#define AT45_SS_PIN     10
/** Reset (Reset) **/
#define AT45_RESET_PIN  -1
/** Write protect (WP) **/
#define AT45_WP_PIN     -1
/**
 * @}
 **/

/**
 * @defgroup STATUS_REGISTER_FORMAT Status register format.
 * The status register can be used to determine device state
 * (ready/busy) or to retrieve the result of an operation.
 * @{
 **/
/**
 * Ready/busy status is indicated using bit 7 of the status register.
 * If bit 7 is a 1, then the device is not busy and is ready to accept
 * the next command. If bit 7 is a 0, then the device is in a busy
 * state.
 **/
#define AT45_READY 0x80
/**
 * Result of the most recent Memory Page to Buffer Compare operation.
 * If this bit is equal to 0, then the data in the main memory page
 * matches the data in the buffer. If it's 1 then at least 1 byte in
 * the main memory page does not match the data in the buffer.
 **/
#define AT45_COMPARE 0x40
/**
 * Bit 1 in the Status Register is used to provide information to the
 * user whether or not the sector protection has been enabled or
 * disabled, either by software-controlled method or
 * hardware-controlled method. 1 means that the sector protection has
 * been enabled and 0 that it has been disabled.
 **/
#define AT45_PROTECT 0x02
/**
 * Bit 0 indicates whether the page size of the main memory array is
 * configured for "power of 2" binary page size (512 bytes) (bit=1) or
 * standard %Dataflash page size (528 bytes) (bit=0).
 **/
#define AT45_PAGESIZE_PWR2 0x01
/**
 * Bits 5, 4, 3 and 2 indicates the device density. The decimal value
 * of these four binary bits does not equate to the device density; the
 * four bits represent a combinational code relating to differing
 * densities of %Dataflash devices. The device density is not the same
 * as the density code indicated in the JEDEC device ID information.
 * The device density is provided only for backward compatibility.
 **/
#define AT45_SIZE_CODE 0x2C
/**
 * @}
 **/

 /**
 * @defgroup SPECIFIC_SECTORS Special sectors ID.
 * The following list gives the number of pages per sector (P) for the AT45 family:
 *   - AT45DB011D 128 
 *   - AT45DB021D 128
 *   - AT45DB041D 256
 *   - AT45DB081D 256
 *   - AT45DB161D 256
 *   - AT45DB321D 128
 *   - AT45DB642D 256
 *
 * On every %Dataflash, the first 8 pages belongs to the sectod 0a. It's followed 
 * by sector 0b which holds only (P-8) pages (248 on an AT45DB161D). Then comes N-1 
 * (N is the number of sectors) sectors of size P numbered from 1 to N-1 (included).
 * @see chapter titled "Memory Array" in the corresponding %Dataflash datasheet.
 * @{
 **/
/**
 * Sector 0a id.
 **/
#define AT45_SECTOR_0A  0
/**
 * Sector 0b id.
 **/
#define AT45_SECTOR_0B -1
 /**
  * @}
  **/
 
 
/**
 * AT45DBxxxD Atmel %Dataflash device.
 **/
class DataFlash
{
    public:
        /**
         * @brief ID structure.
         * This structure contains information about the
         * %Dataflash chip being used.
         **/
        struct ID
        {
            uint8_t manufacturer;       /**< Manufacturer id **/
            uint8_t device[2];          /**< Device id **/
            uint8_t extendedInfoLength; /**< Extended device information string length **/
        };

        /**
         * @brief Erase mode.
         * Whether pages are erased automatically before being written, or 
         * whether this is expected to be done explicitly first.
         **/
        enum erasemode
        {
            ERASE_AUTO,             /**< Pages are erased automatically. **/
            ERASE_MANUAL            /**< Pages are erased by the user first. **/
        };

        /** 
         * @brief IO speed.
         * The max SPI SCK frequency an ATmega 328P or 1280 can generate is
         * 10MHz. The limit for low-speed SCK for AT45DBxxxD %Dataflash is 33MHz
         * (66MHz for high-speed). Supporting high-speed for Arduino is a waste of
         * time...
         **/
        enum IOspeed
        {
            SPEED_LOW,              /**< Low speed transfers up to 33MHz **/
            SPEED_HIGH              /**< High speed transfers up to 66MHz **/
        };

    public:
        /** Constructor **/
        DataFlash();

        /** Destructor **/
        ~DataFlash();

        /**
         * Set pin use, with defaults for reset and write-protect if not
         * specified as argument.
         * Set SPI transfer speed to "low" (can be changed with .speed() ).
         * @note This must be called the first time; afterwards .begin() can
         *       be called without arguments.
         * @param csPin Chip select (Slave select) pin.
         * @param resetPin Reset pin, optional (default none).
         * @param wpPin Write protect pin, optional (default none).
         * **/
        void setup(int8_t csPin, int8_t resetPin=AT45_RESET_PIN, int8_t wpPin=AT45_WP_PIN);

        /**
         * Initialise SPI interface for use with the %Dataflash,
         * allowing shared use with other SPI devices (which must however use
         * a different chip select pin).
         * **/
        void begin();

        /**
         * Restore SPI configuration, so it can be used by other SPI devices.
         **/
        void end();

        /**
         * Enable (select) %Dataflash.
         **/
        inline void enable();

        /**
         * Disable (deselect) %Dataflash.
         **/
        inline void disable();

        /**
         * Disable (deselect) %Dataflash, then enable (select) it again.
         **/
        void reEnable();

        /**
         * Set erase mode to automatic (default).
         **/
        void autoErase();
        
        /**
         * Set erase mode to manual.
         * User must erase pages first, using one of the erase commands.
         **/
        void manualErase();
        
#ifdef AT45_USE_SPI_SPEED_CONTROL
        /**
         * Set transfer speed (33MHz = low, 66MHz = high).
         * Note: Arduino supports 20MHz max, so using "high" is actually slower
         * because additional bytes have to be transferred for no benefit.
         **/
        void setTransferSpeed(IOspeed rate);

        /**
         * Get transfer speed.
         **/
        IOspeed getTransferSpeed() const;
#endif // AT45_USE_SPI_SPEED_CONTROL

        /**
         * Return whether the chip has completed the current operation and is
         * ready for the next.
         * Note that in some situations read/write access to one of the buffers
         * is permitted although the chip is busy.
         **/
        uint8_t isReady();

        /**
         * @brief Wait until the chip is ready.
         * Perform a low-to-high transition on the CS pin and then poll
         * the status register until the %Dataflash is ready for the next
         * operation.
         */
        void waitUntilReady();
        
        /**
         * Same as waitUntilReady
         **/
        inline void endAndWait();
        
        /**
         * Read status register.
         * @return The content of the status register.
         * **/
        uint8_t status();

        /**
         * Read Manufacturer and Device ID.
         * @note If id.extendedInfoLength is not equal to zero,
         *       successive calls to SPI.transfer() return
         *       the extended device information bytes.
         * @param id ID structure.
         **/
        void readID(DataFlash::ID &id);

        /**
         * A main memory page read allows the user to read data directly from
         * any one of the pages in the main memory, bypassing both of the
         * data buffers and leaving the contents of the buffers unchanged.
         * Reading past the end of the page wraps around to the beginning of
         * the page.
         * The chip must remain enabled by this function; it is the user's
         * responsibility to disable the chip when finished reading.
         * @param page Page of the main memory to read.
         * @param offset Starting byte address within the page (default value: 0).
         **/
        void pageRead(uint16_t page, uint16_t offset=0);
        
        /**
         * Sequentially read a continuous stream of data at the currently set
         * speed. Reading past the end of the last page wraps around to the
         * beginning of the first page.
         * The chip must remain enabled by this function; it is the user's
         * responsibility to disable the chip when finished reading.
         * @param page Page of the main memory where the sequential read will
         * start.
         * @param offset Starting byte address within the page (default value: 0).
         * @note The legacy mode is not needed and not supported.
         **/
        void arrayRead(uint16_t page, uint16_t offset=0);

        /**
         * Read the content of one of the SRAM data buffer at the currently
         * set speed. Reading past the end of the buffer wraps around to the
         * beginning.
         * The chip must remain enabled by this function; it is the user's
         * responsibility to disable the chip when finished reading.
         * @param bufferNum Buffer to read (0 or 1).
         * @param offset Starting byte within the buffer (default value: 0).
         **/
        void bufferRead(uint8_t bufferNum, uint16_t offset=0);

        /**
         * Write data to one of the SRAM data buffers at the currently set
         * speed. Writing past the end of the buffer wraps around to the
         * beginning.
         * The chip must remain enabled by this function; it is the user's
         * responsibility to disable the chip when finished reading.
         * @param bufferNum Buffer to read (0 or 1).
         * @param offset Starting byte within the buffer (default value: 0).
         **/
        void bufferWrite(uint8_t bufferNum, uint16_t offset);

        /**
         * Transfer data from buffer 0 or 1 to a main memory page, erasing the
         * page first if auto-erase is set. If erase is manual, the page must
         * have been erased previously using one of the erase commands.
         * @param bufferNum Buffer to use (0 or 1).
         * @param page Page to which the content of the buffer is written.
         **/
        void bufferToPage(uint8_t bufferNum, uint16_t page);

        /**
         * Transfer a page of data from main memory to buffer 0 or 1.
         * @param page Main memory page to transfer.
         * @param bufferNum Buffer (0 or 1) to which the data is written.
         **/
        void pageToBuffer(uint16_t page, uint8_t bufferNum);

        /**
         * Erase a page in the main memory array.
         * @param page Page to erase.
         **/
        void pageErase(uint16_t page);

        /**
         * Erase a block of pages in a single operation.
         * @param block Block to erase.
         * @warning UNTESTED
         **/
        void blockErase(uint16_t block);

        /**
         * Erase a sector of blocks in a single operation.
         * @param sector Sector to erase.
         **/
        void sectorErase(int8_t sector);

#ifdef AT45_CHIP_ERASE_ENABLED
        /**
         * Erase the entire chip memory. Sectors protected or locked down will
         * not be erased.
         * @warning UNTESTED
         * @warning MAY DAMAGE CHIP, THEREFORE NOT AVAILABLE.
         *          READ DATASHEET FOR DETAILS.
         **/
        void chipErase();
#endif

        /**
         * This a combination of Buffer Write and Buffer to Page with
         * Built-in Erase.
         * The global erase flag .manual_erase() is ignored.
         * Writing past the end of the page wraps around to the beginning of
         * the page.
         * @note You must call endAndWait in order to start transferring data
         * from buffer to page.
         * @param page Page to which the content of the buffer is written.
         * @param offset Starting byte address within the buffer.
         * @param bufferNum Buffer to use (0 or 1).
         **/
        void beginPageWriteThroughBuffer(uint16_t page, uint16_t offset, uint8_t bufferNum);

        /**
         * Compare a page of data in main memory to the data in buffer 0 or 1.
         * @param page Page to compare.
         * @param bufferNum Buffer number (0 or 1).
         * @return
         *      - true  If the page and the buffer contains the same data.
         *      - false Otherwise.
         **/
        int8_t isPageEqualBuffer(uint16_t page, uint8_t bufferNum);

        /**
         * Put the device into the lowest power consumption mode.
         * Once the device has entered the Deep Power-down mode, all
         * instructions are ignored except the Resume from Deep
         * Power-down command.
         * @warning UNTESTED
         **/
        void deepPowerDown();

        /**
         * Takes the device out of Deep Power-down mode.
         * @warning UNTESTED
         **/
        void resumeFromDeepPowerDown();

        /**
         * Reset device via the reset pin.
         **/
        void hardReset();

        /**
         * Enable write protection.
         **/
        inline void writeProtect();

        /**
         * Disable write protection.
         **/
        inline void readWrite();

        /** Get chip Select (CS) pin **/
        inline int8_t chipSelectPin  () const;
        /** Get reset (RESET) pin **/
        inline int8_t resetPin       () const;
        /** Get write protect (WP) pin **/
        inline int8_t writeProtectPin() const;

    private:
        /**
         * Compute page address hi byte.
         */
        inline uint8_t pageToHiU8(uint16_t page) const;

        /**
         * Compute page address lo byte.
         */
        inline uint8_t pageToLoU8(uint16_t page) const;

    private:
        /**
         * %Dataflash read/write addressing infos.
         * @warning Power of 2 addressing is not supported for the moment!
         **/
        struct AddressingInfos
        {
            uint8_t bufferSize[7]; /**< Size of the buffer address bits. **/
            uint8_t pageSize[7];   /**< Size of the page address bits. **/
            uint8_t sectorSize[7]; /**< Size of the sector address bits (part of the page address). **/
        };

        static const AddressingInfos m_infos; /**< @see AddressingInfos **/
        
        int8_t m_chipSelectPin;    /**< Chip select pin (CS). **/
        int8_t m_resetPin;         /**< Reset pin (RESET). **/
        int8_t m_writeProtectPin;  /**< Write protect pin (WP). **/

        uint8_t m_SPCR;             /**< SPI register backup. **/
        uint8_t m_SPSR;             /**< SPI register backup. **/

        uint8_t m_bufferSize;       /**< Size of the buffer address bits. **/
        uint8_t m_pageSize;         /**< Size of the page address bits. **/
        uint8_t m_sectorSize;       /**< Size of the sector address bits. **/

        enum erasemode m_erase;     /**< Erase mode - auto or manual. **/

#ifdef AT45_USE_SPI_SPEED_CONTROL
        enum IOspeed m_speed;       /**< SPI transfer speed. **/
#endif
};

#include "DataFlashInlines.h"

/**
 * @}
 **/

#endif /* DATAFLASH_H_ */
