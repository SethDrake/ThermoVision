
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_exti.h"
/*#include "usb_lib.h"
#include "usb_pwr.h"
#include "memory.h"*/
#include "ov7670.h"
#include "ili9341.h"
#include "sdcard.h"
#include "ff.h"
#include "sg90_servo.h"
#include "delay.h"
#include "misc.h"
#include "mlx90614.h"
#include "periph_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


typedef struct
{
	float temp;
	int16_t xpos;
	int16_t ypos;
} TEMP_withCoord;

typedef struct  
{
	char signature[4] = "THM";
	uint8_t version;
	uint16_t width;
	uint16_t height;
	uint8_t isContainsTemperature;
	uint8_t isContainsThermalMap;
	uint8_t isContainsPhoto;
} THM_HEADER;

const uint16_t xmax = 320;
const uint16_t ymax = 200;

float tline[xmax];
uint16_t line[xmax];

volatile float mit, mat;
volatile uint8_t isCardMounted = 0;
volatile uint16_t cameraId = 0;

MLX90614 irSensor;
SG90Servo horizServo;
SG90Servo vertServo;
OV7670 camera;
ILI9341 display;
SDCARD sdcard;

FATFS fatfs;

void RCC_Configuration(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5); // set usb clock to 72MHz/1.5 = 48MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//clock DMA1 controller
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    SysTick_Config(SystemCoreClock / 1000);
}
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin =  I2C_SCL | I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);

    /* Configure SPI2 pins */
    GPIO_InitStructure.GPIO_Pin = SPI_SCK | SPI_MOSI | SPI_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	/* SET PA11, PA12 for USB: USB_DM, USB_DP */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //disable JTAG

	/* setup display pins */
	display.setupHw(SPI2, SPI_BaudRatePrescaler_2, ILI9341_PORT, ILI9341_PIN_RS, ILI9341_PIN_RESET, ILI9341_PIN_CS);
	
	/* setup sdcard pins */
	sdcard.setupHw(SPI2, SPI_BaudRatePrescaler_2, SD_PORT, SD_PIN_CS);

	/* setup servo pins*/
	horizServo.setupPort(SERVO_PORT, SERVO_PWM1);
	vertServo.setupPort(SERVO_PORT, SERVO_PWM2);

	/* setup camera pins */
	camera.setupSCCB(OV7670_SCCB_PORT, OV7670_SCCB_CLK_PIN, OV7670_SCCB_DATA_PIN);
	camera.setupHw(OV7670_DATA_PORT, OV7670_CLCK_PORT, OV7670_VSYNC_PIN, OV7670_RCLK_PIN, OV7670_WREN_PIN, OV7670_WRRST_PIN);

}
void I2C_Configuration(void)
{
    I2C_InitTypeDef  I2C_InitStructure;

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C1, ENABLE);
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C1, &I2C_InitStructure);
}
void SPI_Configuration(void)
{
	SPI_InitTypeDef spiStruct;
	SPI_Cmd(SPI2, DISABLE);
	spiStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spiStruct.SPI_Mode = SPI_Mode_Master;
	spiStruct.SPI_DataSize = SPI_DataSize_8b;
	spiStruct.SPI_CPOL = SPI_CPOL_Low;
	spiStruct.SPI_CPHA = SPI_CPHA_1Edge;
	spiStruct.SPI_NSS = SPI_NSS_Soft;
	spiStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; /* PCLK2/2 = 72Mhz/2/2 = 18MHz SPI */
	spiStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	spiStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &spiStruct);
	SPI_Cmd(SPI2, ENABLE);
	SPI_CalculateCRC(SPI2, DISABLE);
}
void EXTI_Configuration()
{
	/*EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_ClearITPendingBit(EXTI_Line18);
	EXTI_InitStructure.EXTI_Line = EXTI_Line18;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);*/
}
void NVIC_Configuration(void) {

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);*/
}

void checkMinMaxTemp(int16_t xpos, int16_t ypos, float temperature, TEMP_withCoord* minTemp, TEMP_withCoord* maxTemp) {
	if (temperature < minTemp->temp) {
		minTemp->temp = temperature;
		minTemp->xpos = xpos;
		minTemp->ypos = ypos;
	}
	if (temperature > maxTemp->temp) {
		maxTemp->temp = temperature;
		maxTemp->xpos = xpos;
		maxTemp->ypos = ypos;
	}
}
void prepareIRScan(TEMP_withCoord* minTemp, TEMP_withCoord* maxTemp) {
	int16_t x;
	uint8_t i, j;
	const uint16_t xmax = 320;

	minTemp->temp = 200;
	maxTemp->temp = -50;

	for (x = 0; x < xmax; x++) {
		vertServo.setServoAngle(100 - (x / 4));
		horizServo.setServoAngle(120 - (x / 4));
		vTaskDelay(10);
		float temperature = irSensor.readTemp(0);
		checkMinMaxTemp(0, 0, temperature, minTemp, maxTemp);
	}
}
uint16_t scanIRPoint(int16_t xpos, int16_t ypos, TEMP_withCoord* minTemp, TEMP_withCoord* maxTemp, float* temp) {
	 float temperature = irSensor.readTemp(0);
	*temp = temperature;
	checkMinMaxTemp(xpos, ypos, temperature, minTemp, maxTemp);
	return irSensor.temperatureToRGB565(temperature, mit, mat);
}
void scanIRToBuffer(TEMP_withCoord* minTemp, TEMP_withCoord* maxTemp, uint8_t stepX, uint8_t stepY) {
	volatile uint16_t x, y, s, t, p;
	volatile uint8_t i;
	float ptmp;
	UINT writtenBytes;
	THM_HEADER fileHeader;
	char filename[12];
	FIL fp;

	
	if (isCardMounted) {
		sprintf(filename, "%d.thm", xTaskGetTickCount());
		fileHeader.version = 1;
		fileHeader.width = xmax;
		fileHeader.height = ymax;
		fileHeader.isContainsTemperature = 1;
		fileHeader.isContainsThermalMap = 1;
		fileHeader.isContainsPhoto = (cameraId != 0);

		FRESULT fr = f_open(&fp, (const TCHAR*) filename, FA_WRITE | FA_OPEN_ALWAYS);
		if (fr == FR_OK)
		{
			fr = f_lseek(&fp, f_size(&fp));
			if (fr == FR_OK) {
				f_write(&fp, &fileHeader, sizeof(fileHeader), &writtenBytes);
			}
			f_close(&fp);
		}
		else {
			isCardMounted = 0;
		}
	}

	if (isCardMounted && fileHeader.isContainsPhoto) {
		camera.captureFrame();
		for (uint16_t i = 0; i < ymax; i++) {
			camera.readLine(line, xmax);
			display.lineDraw(i, line, xmax);
			while (display.IsDataSending()); //wait until all data was sended to display

			FRESULT fr = f_open(&fp, (const TCHAR*) filename, FA_WRITE | FA_OPEN_ALWAYS);
			if (fr == FR_OK)
			{
				fr = f_lseek(&fp, f_size(&fp));
				if (fr == FR_OK) {
					f_write(&fp, line, xmax * 2, &writtenBytes);
				}
				f_close(&fp);
			}
			else {
				isCardMounted = 0;
			}
		}
		camera.resetCounters();
	}
	vTaskDelay(500);

	t = 0;
	for (y = 0; y < (ymax / stepY); y++) {
		vertServo.setServoAngle(100 - (t / 4));

		if ((y % 2) == 0) {
			s = 0;
			for (x = 0; x < xmax / stepX; x++) {
				horizServo.setServoAngle(120 - (s / 4));
				vTaskDelay(20 + (20 * (stepX - 1)));
				p = scanIRPoint(s, t, minTemp, maxTemp, &ptmp);
				for (i = 0; i < stepX; i++) { //fill line buffer
					tline[s] = ptmp;
					line[s] = p;
					s++;
				}
			}
		}
		else {
			for (x = (xmax / stepX); x--;) {
				horizServo.setServoAngle(120 - (s / 4));
				vTaskDelay(20 + (20 * (stepX - 1)));
				p = scanIRPoint(s, t, minTemp, maxTemp, &ptmp);
				for (i = 0; i < stepX; i++) {
					tline[s - 1] = ptmp;
					line[s-1] = p;
					s--;
				}
			}
		}
		for (i = 0; i < stepY; i++) {
			display.lineDraw(t, line, xmax); //draw line
			t++;
		}
		if (isCardMounted && (fileHeader.isContainsTemperature || fileHeader.isContainsThermalMap)) {
			while (display.IsDataSending()); //wait until all data was sended to display

			for (i = 0; i < stepY; i++) {
				FRESULT fr = f_open(&fp, (const TCHAR*) filename, FA_WRITE | FA_OPEN_ALWAYS);
				if (fr == FR_OK)
				{
					fr = f_lseek(&fp, f_size(&fp));
					if (fr == FR_OK) {
						if (fileHeader.isContainsTemperature) {
							f_write(&fp, tline, xmax * 4, &writtenBytes);
						}
						if (fileHeader.isContainsThermalMap) {
							f_write(&fp, line, xmax * 2, &writtenBytes);
						}
					}
					f_close(&fp);
				}
				else {
					isCardMounted = 0;
				}
			}
		}
	}
}

/*void Enter_LowPowerMode(void)
{
	bDeviceState = SUSPENDED;
}

void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	if (pInfo->Current_Configuration != 0)
	{
		bDeviceState = CONFIGURED;
	}
	else
	{
		bDeviceState = ATTACHED;
	}

}*/

void vMountSDCard(void *pvParameters)
{
	while (1) {
		if (!isCardMounted) {
			FRESULT result = f_mount(&fatfs, (const TCHAR*)"0", 1);
			isCardMounted = (result == FR_OK);
			if (isCardMounted) {
				display.printf(260, 220, WHITE, BLACK, "SD-CARD");
			}
			else {
				display.printf(260, 220, RED, BLACK, "NO CARD");
			}
		}
		vTaskDelay(3000);
	}
}

void vThermalScan(void *pvParameters)
{
	TEMP_withCoord minTemp, maxTemp;

	prepareIRScan(&minTemp, &maxTemp);
	mit = minTemp.temp - 5;
	mat = maxTemp.temp + 5;

	while (1)
	{
		display.fillScreen(TFT_MIN_X, TFT_MIN_Y, TFT_MAX_X, 200, BLACK);
		scanIRToBuffer(&minTemp, &maxTemp, 1, 1);
		mit = minTemp.temp - 2;
		mat = maxTemp.temp + 2;

		if (minTemp.xpos > 280) {
			minTemp.xpos = 280;
		}
		if (minTemp.ypos > 180) {
			minTemp.xpos = 180;
		}
		if (maxTemp.xpos > 280) {
			maxTemp.xpos = 280;
		}
		if (maxTemp.ypos > 180) {
			maxTemp.xpos = 180;
		}

		display.printf(minTemp.xpos, minTemp.ypos, BLUE, BLACK, "%02.02f", minTemp.temp);
		display.printf(maxTemp.xpos, maxTemp.ypos, RED, BLACK, "%02.02f", maxTemp.temp);

		vTaskDelay(5000);
	}
}

void vTestServos(void *pvParameters)
{
	while (1)
	{
		for (uint8_t i = 0; i <= 180; i++) {
			horizServo.setServoAngle(i);
			vTaskDelay(50);
		}
		for (uint8_t i = 180; i--;) {
			horizServo.setServoAngle(i);
			vTaskDelay(50);
		}
		vTaskDelay(5000);
	}
}


int main(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	I2C_Configuration();
	SPI_Configuration();
	EXTI_Configuration();
	NVIC_Configuration();

	//USB_Init();

	horizServo.setup(TIM4, HORIZ_SERVO_CH);
	DelayManager::DelayMs(100);
	vertServo.setup(TIM4, VERT_SERVO_CH);
	DelayManager::DelayMs(100);

	irSensor.setup(I2C1);
	display.init();

	display.clear(BLACK);
	display.printf(20, 220, YELLOW, BLACK, "ThV-IR Project version 0.1e");
	
	display.printf(20, 200, "CAM INIT");
	cameraId = camera.init();
	if (cameraId) {
		display.printf(20, 200, "CAM ID:0x%x", cameraId);
	}
	else {
		display.printf(20, 200, "CAM NOT FOUND");
	}

	xTaskCreate(vMountSDCard, "MountSDCard", 256, NULL, tskIDLE_PRIORITY + 5, NULL);
	//xTaskCreate(vThermalScan, "ThermalScan", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vTestServos, "TestServos", 128, NULL, tskIDLE_PRIORITY + 1, NULL);

	vTaskStartScheduler();
}
