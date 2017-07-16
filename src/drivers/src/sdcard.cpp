

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "delay.h"
#include "sdcard.h"

SDCARD::SDCARD() {

}

SDCARD::~SDCARD() {

}

void SDCARD::setupHw(SPI_TypeDef* spi, uint16_t spiPrescaler, GPIO_TypeDef* controlPort, uint16_t csPin) {
	this->spi = spi;
	this->spiPrescaler = spiPrescaler;
	this->controlPort = controlPort;
	this->csPin = csPin;

	/* CS pin of SDCARD */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = csPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(controlPort, &GPIO_InitStructure);
	switchCs(1);
}

void SDCARD::setSPISpeed(uint16_t prescaler) {
	SPI_Cmd(spi, DISABLE);
	spi->CR1 &= ~SPI_CR1_BR; // Clear SPI baud rate bits
	spi->CR1 |= prescaler; // Set SPI baud rate bits
	SPI_Cmd(spi, ENABLE);
}

SD_Error SDCARD::init(void)
{
	uint32_t i = 0;
	setSPISpeed(SPI_BaudRatePrescaler_256);
	switchCs(1);
	/*!< Send dummy byte 0xFF, 10 times with CS high */
	/*!< Rise CS and MOSI for 80 clocks cycles */
	for (i = 0; i <= 9; i++)
	{
		/*!< Send dummy byte 0xFF */
		writeByte(SD_DUMMY_BYTE);
	}
	/*------------Put SD in SPI mode--------------*/
	/*!< SD initialized and set to SPI mode properly */
	SD_Error result = (goIdleState());
	setSPISpeed(this->spiPrescaler);
	return result;
}

uint8_t SDCARD::detect(void)
{
	__IO uint8_t status = SD_PRESENT;
	/*!< Check GPIO to detect SD */
	/*if (GPIO_ReadInputData(SD_DETECT_GPIO_PORT) & SD_DETECT_PIN)
	{
		status = SD_NOT_PRESENT;
	}*/
	return status;
}

SD_Error SDCARD::getCardInfo(SD_CardInfo *cardinfo)
{
	SD_Error status = SD_RESPONSE_FAILURE;
	status = getCSDRegister(&(cardinfo->SD_csd));
	status = getCIDRegister(&(cardinfo->SD_cid));
	cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1);
	cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
	cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
	cardinfo->CardCapacity *= cardinfo->CardBlockSize;
	return status;
}

SD_Error SDCARD::readBlock(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t BlockSize)
{
	uint32_t i = 0;
	SD_Error rvalue = SD_RESPONSE_FAILURE;

	switchCs(0);
	sendCmd(SD_CMD_READ_SINGLE_BLOCK, ReadAddr, 0xFF);
	if (!getResponse(SD_OK))
	{
		if (!getResponse(SD_START_DATA_SINGLE_BLOCK_READ))
		{
			for (i = 0; i < BlockSize; i++)
			{
				*pBuffer = readByte();
				pBuffer++;
			}
			readByte();
			readByte();
			rvalue = SD_OK;
		}
	}
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);
	return rvalue;
}

SD_Error SDCARD::readMultiBlocks(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	uint32_t i = 0;
	SD_Error rvalue = SD_RESPONSE_FAILURE;

	switchCs(0);
	sendCmd(SD_CMD_READ_MULT_BLOCK, ReadAddr, 0xFF);
	if (getResponse(SD_OK))
	{
		return  SD_RESPONSE_FAILURE;
	}
	while (NumberOfBlocks--) {
		while (getResponse(SD_START_DATA_MULTIPLE_BLOCK_READ));
		for (i = 0; i < BlockSize; i++)
		{
			*pBuffer = readByte();
			pBuffer++;
		}
		readByte();
		readByte();
	}
	sendCmd(SD_CMD_STOP_TRANSMISSION, 0, 0xFF);
	if (getResponse(SD_OK))
	{
		return  SD_RESPONSE_FAILURE;
	}
	rvalue = SD_OK;
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);
	return rvalue;
}

SD_Error SDCARD::writeBlock(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize)
{
	uint32_t i = 0;
	SD_Error rvalue = SD_RESPONSE_FAILURE;

	switchCs(0);
	sendCmd(SD_CMD_WRITE_SINGLE_BLOCK, WriteAddr, 0xFF);
	if (!getResponse(SD_OK))
	{
		writeByte(SD_DUMMY_BYTE);
		writeByte(SD_START_DATA_SINGLE_BLOCK_WRITE);
		for (i = 0; i < BlockSize; i++)
		{
			writeByte(*pBuffer);
			pBuffer++;
		}
		readByte();
		readByte();
		if (getDataResponse() == SD_DATA_OK)
		{
			rvalue = SD_OK;
		}
	}
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);
	return rvalue;
}

SD_Error SDCARD::writeMultiBlocks(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	uint32_t i = 0;
	SD_Error rvalue = SD_RESPONSE_FAILURE;

	switchCs(0);
	sendCmd(SD_CMD_WRITE_MULT_BLOCK, WriteAddr, 0xFF);
	if (getResponse(SD_OK))
	{
		return SD_RESPONSE_FAILURE;
	}
	writeByte(SD_DUMMY_BYTE);
	while (NumberOfBlocks--)
	{
		writeByte(SD_START_DATA_MULTIPLE_BLOCK_WRITE);
		for (i = 0; i < BlockSize; i++)
		{
			writeByte(*pBuffer);
			pBuffer++;
		}
		readByte();
		readByte();
		if (getDataResponse() != SD_DATA_OK)
		{
			return SD_RESPONSE_FAILURE;
		}
	}
	sendCmd(SD_STOP_DATA_MULTIPLE_BLOCK_WRITE, 0, 0xFF);
	rvalue = SD_OK;
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);
	return rvalue;
}

uint16_t SDCARD::getStatus(void)
{
	uint16_t Status = 0;

	switchCs(0);
	sendCmd(SD_CMD_SEND_STATUS, 0, 0xFF);
	Status = readByte();
	Status |= (uint16_t) (readByte() << 8);
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);

	return Status;
}

SD_Error SDCARD::goIdleState(void)
{
	switchCs(0);
	sendCmd(SD_CMD_GO_IDLE_STATE, 0, 0x95);
	if (getResponse(SD_IN_IDLE_STATE))
	{
		return SD_RESPONSE_FAILURE;
	}
	/*----------Activates the card initialization process-----------*/
	do
	{
		switchCs(1);
		writeByte(SD_DUMMY_BYTE);
		switchCs(0);
		sendCmd(SD_CMD_SEND_OP_COND, 0, 0xFF);
	} while (getResponse(SD_OK));
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);

	return SD_OK;
}



SD_Error SDCARD::getCSDRegister(SD_CSD* SD_csd)
{
	uint32_t i = 0;
	SD_Error rvalue = SD_RESPONSE_FAILURE;
	uint8_t CSD_Tab[16];

	switchCs(0);
	/*!< Send CMD9 (CSD register) or CMD10(CSD register) */
	sendCmd(SD_CMD_SEND_CSD, 0, 0xFF);
	if (!getResponse(SD_OK))
	{
		if (!getResponse(SD_START_DATA_SINGLE_BLOCK_READ))
		{
			for (i = 0; i < 16; i++)
			{
				CSD_Tab[i] = readByte();
			}
		}
		writeByte(SD_DUMMY_BYTE);
		writeByte(SD_DUMMY_BYTE);
		rvalue = SD_OK;
	}
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);

	/*!< Byte 0 */
	SD_csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
	SD_csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
	SD_csd->Reserved1 = CSD_Tab[0] & 0x03;
	/*!< Byte 1 */
	SD_csd->TAAC = CSD_Tab[1];
	/*!< Byte 2 */
	SD_csd->NSAC = CSD_Tab[2];
	/*!< Byte 3 */
	SD_csd->MaxBusClkFrec = CSD_Tab[3];
	/*!< Byte 4 */
	SD_csd->CardComdClasses = CSD_Tab[4] << 4;
	/*!< Byte 5 */
	SD_csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
	SD_csd->RdBlockLen = CSD_Tab[5] & 0x0F;
	/*!< Byte 6 */
	SD_csd->PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
	SD_csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
	SD_csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
	SD_csd->DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
	SD_csd->Reserved2 = 0; /*!< Reserved */
	SD_csd->DeviceSize = (CSD_Tab[6] & 0x03) << 10;
	/*!< Byte 7 */
	SD_csd->DeviceSize |= (CSD_Tab[7]) << 2;
	/*!< Byte 8 */
	SD_csd->DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
	SD_csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
	SD_csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
	/*!< Byte 9 */
	SD_csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
	SD_csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
	SD_csd->DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
	/*!< Byte 10 */
	SD_csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
	SD_csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
	SD_csd->EraseGrMul = (CSD_Tab[10] & 0x3F) << 1;
	/*!< Byte 11 */
	SD_csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
	SD_csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);
	/*!< Byte 12 */
	SD_csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
	SD_csd->ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
	SD_csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
	SD_csd->MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
	/*!< Byte 13 */
	SD_csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
	SD_csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
	SD_csd->Reserved3 = 0;
	SD_csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);
	/*!< Byte 14 */
	SD_csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
	SD_csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
	SD_csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
	SD_csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
	SD_csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
	SD_csd->ECC = (CSD_Tab[14] & 0x03);
	/*!< Byte 15 */
	SD_csd->CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
	SD_csd->Reserved4 = 1;

	return rvalue;
}

SD_Error SDCARD::getCIDRegister(SD_CID* SD_cid)
{
	uint32_t i = 0;
	SD_Error rvalue = SD_RESPONSE_FAILURE;
	uint8_t CID_Tab[16];

	switchCs(0);
	/*!< Send CMD10 (CID register) */
	sendCmd(SD_CMD_SEND_CID, 0, 0xFF);
	if (!getResponse(SD_OK))
	{
		if (!getResponse(SD_START_DATA_SINGLE_BLOCK_READ))
		{
			for (i = 0; i < 16; i++)
			{
				CID_Tab[i] = readByte();
			}
		}
		writeByte(SD_DUMMY_BYTE);
		writeByte(SD_DUMMY_BYTE);
		rvalue = SD_OK;
	}
	switchCs(1);
	writeByte(SD_DUMMY_BYTE);

	/*!< Byte 0 */
	SD_cid->ManufacturerID = CID_Tab[0];
	/*!< Byte 1 */
	SD_cid->OEM_AppliID = CID_Tab[1] << 8;
	/*!< Byte 2 */
	SD_cid->OEM_AppliID |= CID_Tab[2];
	/*!< Byte 3 */
	SD_cid->ProdName1 = CID_Tab[3] << 24;
	/*!< Byte 4 */
	SD_cid->ProdName1 |= CID_Tab[4] << 16;
	/*!< Byte 5 */
	SD_cid->ProdName1 |= CID_Tab[5] << 8;
	/*!< Byte 6 */
	SD_cid->ProdName1 |= CID_Tab[6];
	/*!< Byte 7 */
	SD_cid->ProdName2 = CID_Tab[7];
	/*!< Byte 8 */
	SD_cid->ProdRev = CID_Tab[8];
	/*!< Byte 9 */
	SD_cid->ProdSN = CID_Tab[9] << 24;
	/*!< Byte 10 */
	SD_cid->ProdSN |= CID_Tab[10] << 16;
	/*!< Byte 11 */
	SD_cid->ProdSN |= CID_Tab[11] << 8;
	/*!< Byte 12 */
	SD_cid->ProdSN |= CID_Tab[12];
	/*!< Byte 13 */
	SD_cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
	SD_cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;
	/*!< Byte 14 */
	SD_cid->ManufactDate |= CID_Tab[14];
	/*!< Byte 15 */
	SD_cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
	SD_cid->Reserved2 = 1;

	return rvalue;
}

void SDCARD::sendCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc)
{
	uint32_t i = 0x00;
	uint8_t Frame[6];

	Frame[0] = (Cmd | 0x40); /*!< Construct byte 1 */
	Frame[1] = (uint8_t) (Arg >> 24); /*!< Construct byte 2 */
	Frame[2] = (uint8_t) (Arg >> 16); /*!< Construct byte 3 */
	Frame[3] = (uint8_t) (Arg >> 8); /*!< Construct byte 4 */
	Frame[4] = (uint8_t) (Arg); /*!< Construct byte 5 */
	Frame[5] = (Crc); /*!< Construct CRC: byte 6 */

	for (i = 0; i < 6; i++)
	{
		writeByte(Frame[i]);
	}
}

uint8_t SDCARD::getDataResponse(void)
{
	uint32_t i = 0;
	uint8_t response, rvalue;

	while (i <= 64)
	{
		response = readByte();
		response &= 0x1F;
		switch (response)
		{
			case SD_DATA_OK:
			{
				rvalue = SD_DATA_OK;
				break;
			}
			case SD_DATA_CRC_ERROR:
				return SD_DATA_CRC_ERROR;
			case SD_DATA_WRITE_ERROR:
				return SD_DATA_WRITE_ERROR;
			default:
			{
				rvalue = SD_DATA_OTHER_ERROR;
				break;
			}
		}
		if (rvalue == SD_DATA_OK)
			break;
		i++;
	}
	while (readByte() == 0);

	return response;
}

SD_Error SDCARD::getResponse(uint8_t Response)
{
	uint32_t Count = 0xFFF;

	while ((readByte() != Response) && Count)
	{
		Count--;
	}
	if (Count == 0)
	{
		return SD_RESPONSE_FAILURE;
	}
	else
	{
		return SD_OK;
	}
}

uint8_t SDCARD::writeByte(uint8_t Data)
{
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET);
	spi->DR = Data;
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET);
	return spi->DR;
}

uint8_t SDCARD::readByte(void)
{
	uint8_t Data = 0;

	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET);
	spi->DR = SD_DUMMY_BYTE;
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET);
	Data = spi->DR;

	return Data;
}

void SDCARD::switchCs(short BitVal)
{
	if (BitVal != Bit_RESET) {
		controlPort->BSRR = csPin;
	}
	else {
		controlPort->BRR = csPin;
	}
}


