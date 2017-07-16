/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various existing      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "delay.h"
#include "objects.h"

#define SD_SECTOR_SIZE 512

SD_CardInfo cdInfo;


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive number to identify the drive */
)
{
	return sdcard.getCardInfo(&cdInfo) != SD_OK;
}



/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive number to identify the drive */
)
{
	SD_Error initResult = sdcard.init();
	return initResult  != SD_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive number to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT result = RES_OK;
	uint32_t addr;
	for (uint8_t i = 0; i < count; i++) {
		addr = sector * SD_SECTOR_SIZE + (i * SD_SECTOR_SIZE);
		buff += SD_SECTOR_SIZE * i;
		if (sdcard.readBlock(buff, addr, SD_SECTOR_SIZE) != SD_OK) {
			result = RES_ERROR;
		}
	}
	return result;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive number to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT result = RES_OK;
	uint32_t addr;
	for (uint8_t i = 0; i < count; i++) {
		addr = sector * SD_SECTOR_SIZE + (i * SD_SECTOR_SIZE);
		buff += SD_SECTOR_SIZE * i;
		if (sdcard.writeBlock((uint8_t*) buff, addr, SD_SECTOR_SIZE) != SD_OK) {
			result = RES_ERROR;
		}
	}
	return result;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive number (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	switch (cmd) {
	case GET_SECTOR_SIZE:     // Get R/W sector size (WORD) 
		*(WORD *) buff = cdInfo.CardBlockSize;
		break;
	case GET_BLOCK_SIZE:      // Get erase block size in unit of sector (DWORD)
		*(DWORD *) buff = cdInfo.CardBlockSize;
		break;
	case GET_SECTOR_COUNT:      // Get erase block size in unit of sector (DWORD)
		*(DWORD *) buff = cdInfo.CardCapacity / cdInfo.CardBlockSize;
		break;
	case CTRL_SYNC:
	case CTRL_TRIM:
		break;
	}

	return RES_OK;


}
#endif
