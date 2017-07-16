/**
  ******************************************************************************
  * @file    memory.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif 



/* Includes ------------------------------------------------------------------*/

#include "memory.h"
#include "usb_scsi.h"
#include "usb_bot.h"
#include "usb_regs.h"
#include "usb_mem.h"
#include "usb_conf.h"
#include "stm32f10x.h"
#include "usb_lib.h"
#include "objects.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t Block_Read_count = 0;
__IO uint32_t Block_offset;
__IO uint32_t Counter = 0;
uint32_t  Idx;
uint8_t Data_Buffer[512]; /* 512 bytes*/
uint8_t TransferState = TXFR_IDLE;

/* Extern variables ----------------------------------------------------------*/
extern uint8_t Bulk_Data_Buff[BULK_MAX_PACKET_SIZE];  /* data buffer*/
extern uint16_t Data_Len;
extern uint8_t Bot_State;
extern Bulk_Only_CBW CBW;
extern Bulk_Only_CSW CSW;

SD_CardInfo cardInfo;

//emfat_t emfat;
uint32_t Mass_Memory_Size[2];
uint32_t Mass_Block_Size[2];
uint32_t Mass_Block_Count[2];

/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Init_Memory() {
	sdcard.init();
}

uint8_t MemoryGetStatus(uint8_t lun) {
	return sdcard.getCardInfo(&cardInfo) != SD_OK;
}

uint8_t MemoryGetCapacity(uint8_t lun)
{
	Mass_Block_Size[lun] = cardInfo.CardBlockSize;
	Mass_Block_Count[lun] = cardInfo.CardCapacity / cardInfo.CardBlockSize;
	Mass_Memory_Size[lun] = cardInfo.CardCapacity;
	return 1;
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Read_Memory(uint8_t lun, uint32_t LBA, uint32_t BlockNbr)
{
  static uint32_t OffsetBytes, LengthBytes;

  if (TransferState == TXFR_IDLE )
  {
	OffsetBytes = LBA * Mass_Block_Size[lun];
	LengthBytes = BlockNbr * Mass_Block_Size[lun];
    TransferState = TXFR_ONGOING;
  }

  if (TransferState == TXFR_ONGOING )
  {
    if (!Block_Read_count)
    {
		sdcard.readBlock(Data_Buffer, OffsetBytes, Mass_Block_Size[lun]);

		USB_SIL_Write(EP1_IN, Data_Buffer, BULK_MAX_PACKET_SIZE);

		Block_Read_count = Mass_Block_Size[lun] - BULK_MAX_PACKET_SIZE;
		Block_offset = BULK_MAX_PACKET_SIZE;
    }
    else
    {
      USB_SIL_Write(EP1_IN, (uint8_t *)Data_Buffer + Block_offset, BULK_MAX_PACKET_SIZE);

      Block_Read_count -= BULK_MAX_PACKET_SIZE;
      Block_offset += BULK_MAX_PACKET_SIZE;
    }

    SetEPTxCount(ENDP1, BULK_MAX_PACKET_SIZE);
    SetEPTxStatus(ENDP1, EP_TX_VALID);  
	OffsetBytes += BULK_MAX_PACKET_SIZE;
	LengthBytes -= BULK_MAX_PACKET_SIZE;

    CSW.dDataResidue -= BULK_MAX_PACKET_SIZE;

	Led_READ(1);
  }
  if (LengthBytes == 0)
  {
    Block_Read_count = 0;
    Block_offset = 0;
	OffsetBytes = 0;
    Bot_State = BOT_DATA_IN_LAST;
    TransferState = TXFR_IDLE;
	Led_READ(0);
  }
}

/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Write_Memory (uint8_t lun, uint32_t LBA, uint32_t BlockNbr)
{

  static uint32_t W_Offset, W_Length;

  uint32_t temp =  Counter + 64;

  if (TransferState == TXFR_IDLE )
  {
    W_Offset = LBA * Mass_Block_Size[lun];
    W_Length = BlockNbr * Mass_Block_Size[lun];
    TransferState = TXFR_ONGOING;
  }

  if (TransferState == TXFR_ONGOING )
  {

    for (Idx = 0 ; Counter < temp; Counter++)
    {
      *((uint8_t *)Data_Buffer + Counter) = Bulk_Data_Buff[Idx++];
    }

    W_Offset += Data_Len;
    W_Length -= Data_Len;

    if (!(W_Length % Mass_Block_Size[lun]))
    {
      Counter = 0;
	  sdcard.writeBlock(Data_Buffer, W_Offset, W_Length);
    }

    CSW.dDataResidue -= Data_Len;
    SetEPRxStatus(ENDP2, EP_RX_VALID); /* enable the next transaction*/   
	//Led_WRITE(1);
  }

  if ((W_Length == 0) || (Bot_State == BOT_CSW_Send))
  {
    Counter = 0;
    Set_CSW (CSW_CMD_PASSED, SEND_CSW_ENABLE);
    TransferState = TXFR_IDLE;
	//Led_WRITE(0);
  }
}

void Format_Memory() {
	//Led_WRITE(1);
	//Led_WRITE(0);
}

#ifdef __cplusplus
}
#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

