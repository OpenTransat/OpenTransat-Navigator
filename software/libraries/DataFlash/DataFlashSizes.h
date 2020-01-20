/**************************************************************************//**
 * @file DataFlashSizes.h
 * @brief Header file containing definitions for the sizes of the flash
 * memories in the Atmel Dataflash AT45DB range.
 *
 * @par Copyright: 
 * - Copyright (C) 2010-2011 by Vincent Cruz.
 * - Copyright (C) 2011 by Volker Kuhlmann. @n
 * All rights reserved.
 *
 * @authors
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
#ifndef DATAFLASH_SIZES_H_
#define DATAFLASH_SIZES_H_

/**
 * @addtogroup AT45DBxxxD
 * @{
 **/
 
/**
 * @defgroup Flash_size Dataflash flash size.
 * For each flash memory IC these definitions give the page size, block size,
 * sector size and total size in bytes, the number of pages, blocks and sectors,
 * and the number of pages in sector 0A (the remaining pages of the first sector
 * are in sector 0b).
 * 
 * The standard page size for %Dataflash has an additional 8 bytes per 256 bytes.
 * These can be turned off permanently, in which case the page size reverts to a
 * binary page size with a power of two. This is indicated in the status register.
 *
 * Sizes below are for the larger (non-binary) page size. To get the binary page
 * size, mask out an appropriate number of lower bits, or multiply by 256/264.
 * @{
 **/

/**
 * @defgroup AT45DB011D_size AT45DB011D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB011_PAGESIZE     264
/** Block size. **/
#define DF_45DB011_BLOCKSIZE    (8 * DF_45DB011_PAGESIZE)
/** Sector size. **/
#define DF_45DB011_SECTORSIZE   (16 * DF_45DB011_BLOCKSIZE)
/** Page count. **/
#define DF_45DB011_PAGES        512
/** Block count. **/
#define DF_45DB011_BLOCKS       64
/** Sector count. */
#define DF_45DB011_SECTORS      4
/** Global size. **/
#define DF_45DB011_SIZE         (DF_45DB011_PAGES * DF_45DB011_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB011_S0A_PAGES    8
/** @} **/

/**
 * @defgroup AT45DB021D_size AT45DB021D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB021_PAGESIZE     264
/** Block size. **/
#define DF_45DB021_BLOCKSIZE    (8 * DF_45DB021_PAGESIZE)
/** Sector size. **/
#define DF_45DB021_SECTORSIZE   (16 * DF_45DB021_BLOCKSIZE)
/** Page count. **/
#define DF_45DB021_PAGES        1024
/** Block count. **/
#define DF_45DB021_BLOCKS       128
/** Sector count. **/
#define DF_45DB021_SECTORS      8
/** Global size. **/
#define DF_45DB021_SIZE         (DF_45DB021_PAGES * DF_45DB021_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB021_S0A_PAGES    8
/** @} **/

/**
 * @defgroup AT45DB041D_size AT45DB041D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB041_PAGESIZE     264
/** Block size. **/
#define DF_45DB041_BLOCKSIZE    (8 * DF_45DB041_PAGESIZE)
/** Sector size. **/
#define DF_45DB041_SECTORSIZE   (32 * DF_45DB041_BLOCKSIZE)
/** Page count. **/
#define DF_45DB041_PAGES        2048
/** Block count. **/
#define DF_45DB041_BLOCKS       256
/** Sector count. **/
#define DF_45DB041_SECTORS      8
/** Global size. **/
#define DF_45DB041_SIZE         (DF_45DB041_PAGES * DF_45DB041_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB041_S0A_PAGES    8
/** @} **/

/**
 * @defgroup AT45DB081D_size AT45DB081D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB081_PAGESIZE     264
/** Block size. **/
#define DF_45DB081_BLOCKSIZE    (8 * DF_45DB081_PAGESIZE)
/** Sector size. **/
#define DF_45DB081_SECTORSIZE   (32 * DF_45DB081_BLOCKSIZE)
/** Page count. **/
#define DF_45DB081_PAGES        4096
/** Block count. **/
#define DF_45DB081_BLOCKS       512
/** Sector count. **/
#define DF_45DB081_SECTORS      16
/** Global size. **/
#define DF_45DB081_SIZE         (DF_45DB081_PAGES * DF_45DB081_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB081_S0A_PAGES    8
/** @} **/

/**
 * @defgroup AT45DB161D_size AT45DB161D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB161_PAGESIZE     528
/** Block size. **/
#define DF_45DB161_BLOCKSIZE    (8 * DF_45DB161_PAGESIZE)
/** Sector size. **/
#define DF_45DB161_SECTORSIZE   (32 * DF_45DB161_BLOCKSIZE)
/** Page count. **/
#define DF_45DB161_PAGES        4096
/** Block count. **/
#define DF_45DB161_BLOCKS       512
/** Sector count. **/
#define DF_45DB161_SECTORS      16
/** Global size. **/
#define DF_45DB161_SIZE         (DF_45DB161_PAGES * DF_45DB161_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB161_S0A_PAGES    8
/** @} **/

/**
 * @defgroup AT45DB321D_size AT45DB321D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB321_PAGESIZE     528
/** Block size. **/
#define DF_45DB321_BLOCKSIZE    (8 * DF_45DB321_PAGESIZE)
/** Sector size. **/
#define DF_45DB321_SECTORSIZE   (16 * DF_45DB321_BLOCKSIZE)
/** Page count. **/
#define DF_45DB321_PAGES        8192
/** Block count. **/
#define DF_45DB321_BLOCKS       1024
/** Sector count. **/
#define DF_45DB321_SECTORS      64
/** Global size. **/
#define DF_45DB321_SIZE         (DF_45DB321_PAGES * DF_45DB321_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB321_S0A_PAGES    8
/** @} **/

/**
 * @defgroup AT45DB642D_size AT45DB642D size specification.
 * @{
 **/
/** Page size. **/
#define DF_45DB642_PAGESIZE     1056
/** Block size. **/
#define DF_45DB642_BLOCKSIZE    (8 * DF_45DB642_PAGESIZE)
/** Sector size. **/
#define DF_45DB642_SECTORSIZE   (32 * DF_45DB642_BLOCKSIZE)
/** Page count. **/
#define DF_45DB642_PAGES        8192
/** Block count. **/
#define DF_45DB642_BLOCKS       1024
/** Sector count. **/
#define DF_45DB642_SECTORS      32
/** Global size. **/
#define DF_45DB642_SIZE         (DF_45DB642_PAGES * DF_45DB642_PAGESIZE)
/** Sector 0x0A page count. */
#define DF_45DB642_S0A_PAGES    8
/** @} **/

/** @} **/
/** @} **/

#endif  // DATAFLASH_SIZES_H_

/******************************************************************************/
