
/*******************************************************************************
 * File Name          : main.c
 * Author             :
 * Version            :
 * Date               :
 * Description        :
 *******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_x_device.h"
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "hal_radio.h"
#include "osal.h"
#include "fifo.h"
#include "sleep.h"
#include "ihc.h"
#include "HDC2080.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	I2C_OP_FAILED = 0,
	I2C_OP_SUCCEED
}
I2C_AppOpStatus;

typedef struct {
	uint8_t rx_buff[7];
	uint8_t tx_buff[7];
	uint8_t ptr_addr;
	uint8_t dev_addr;
	uint8_t rx_size;
	uint8_t tx_size;
}i2c_data_t;

typedef enum {
	state_init,
	state_ready,
	state_off,
	state_err
}app_state_t;

typedef struct {
	app_state_t state;
}app_data_t;

/* Private define ------------------------------------------------------------*/

/* I2C clock frequency */
#define SDK_EVAL_I2C_CLK_SPEED  (5000)//(10000)

/* Private macro -------------------------------------------------------------*/
#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x8E89BED6)
#define STARTING_CHANNEL        (uint8_t)(24)    // RF channel 22
#define HS_STARTUP_TIME         (uint16_t)(1)  /* High Speed start up time min value */

#define CHAT_RECEIVE_RELATIVETIME       5000
#define CHAT_TRANSMIT_RELATIVETIME      5000
#define CHAT_RECEIVE_TIMEOUT            100000  /* 100 ms */

#define MAX_CHAT_PACKET_LEN 3
#define MAX_RETRIES 3

#define CHAT_FREQUENCYCHANNEL           STARTING_CHANNEL

#ifdef PKT_ENCRYPTION
#define	MIC_FIELD_LEN	MIC_FIELD_LENGTH
#else
#define	MIC_FIELD_LEN	0
#endif

/* Private variables ---------------------------------------------------------*/

/* Blue FIFO */
circular_fifo_t blueRec_fifo;
uint8_t blueRec_buffer[MAX_PACKET_LENGTH*2];

uint32_t interval;
uint8_t uart_buffer[MAX_PACKET_LENGTH];
uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t sendAckData[2] = {0xAE, 0}; /* 0xAE ACK value, length = 0 */
uint8_t receivedData[MAX_PACKET_LENGTH];
uint8_t receivedAckData[MAX_PACKET_LENGTH];
static uint8_t retries = 0;

uint8_t flag_SendingPacket = FALSE;
uint8_t SendingPacketFailed = FALSE;

uint8_t app_termo_buff[IHC_CFG_RADIO_PACKET_SIZE];

i2c_data_t i2c_data;
app_data_t app_data;
ADC_InitType xADC_InitType;
volatile FlagStatus xUartDataReady = RESET;
volatile uint8_t ihc_rx_flag = 0;
uint8_t ihc_rx_buff[IHC_CFG_RADIO_PACKET_SIZE];
uint8_t ihc_off_ms = 0;
uint32_t sleep_cnt = 0;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void ADC_Configuration(void);
void MFT_Configuration(void);
void I2C_ConfigurationMaster(void);
I2C_AppOpStatus I2C_Read_Reg(i2c_data_t * data);
I2C_AppOpStatus I2C_Write_Reg(i2c_data_t * data);
void processCommand(void);
uint8_t IhcChecksum(uint8_t * data, uint8_t size);
uint8_t IhcChecksumValidate(uint8_t * data, uint8_t size);
void IhcAppendChecksum(uint8_t * data, uint8_t size);
void HDC2080Init(void);
void HDC2080Read(void);
void AdcRead(void);
uint8_t TxCallback(ActionPacket* p, ActionPacket* next);
uint8_t RxCallback(ActionPacket* p, ActionPacket* next);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  This routine is called when a receive event is complete.
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t RxCallback(ActionPacket* p, ActionPacket* next)
{
//	uint8_t idx;

	/* received a packet */
	if( (p->status & BIT_TX_MODE) == 0)
	{
		if((p->status & IRQ_RCV_OK) != 0)
		{
			fifo_put(&blueRec_fifo, p->data[1]+2, &p->data[0]);

			if( (p->status & IRQ_ERR_ENC) != 0)
			{
				printf("\r\n Rx - err \r\n");
			}
			else
			{
//				if(IHC_CFG_RADIO_PACKET_SIZE == p->data[1])
//				{
//					ihc_rx_flag = 1;	/* data ready, length ok */
//
//					for(idx = 0; idx < IHC_CFG_RADIO_PACKET_SIZE; idx++)
//					{
//						ihc_rx_buff[idx] = p->data[idx + 2];
//					}
//				}
//				else
//				{
//					printf("wrong size");
//				}
			}
		}
		else if((p->status & IRQ_TIMEOUT) != 0)
		{
			//      SdkEvalLedToggle(LED1);
			if(flag_SendingPacket == FALSE)
			{
				HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);
			}
			else
			{
				HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, TxCallback);
			}
		}
		else if((p->status & IRQ_CRC_ERR) != 0)
		{
			if(flag_SendingPacket == FALSE)
			{
				HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);
			}
			else
			{
				HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, TxCallback);
			}
		}
	}
	else /* Transmit complete */
	{
		//    SdkEvalLedToggle(LED3);
		if(flag_SendingPacket == FALSE)
		{
			HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);
		}
		else
		{
			HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL,CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, TxCallback);
		}
	}
	return TRUE;
}

/**
* @brief  This routine is called when a transmit event is complete.
* @param  p: Current action packet which its transaction has been completed.
* @param  next: Next action packet which is going to be scheduled.
* @retval return value: TRUE
*/
uint8_t TxCallback(ActionPacket* p, ActionPacket* next)
{
  /* received a packet */
  if((p->status & BIT_TX_MODE) == 0) {

    if((p->status & IRQ_RCV_OK) != 0) {
      flag_SendingPacket = FALSE;
      HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);
    }
    else if((p->status & IRQ_TIMEOUT) != 0) {
      if (retries > 0) {
        HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL,CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, TxCallback);
        retries--;
      }
      else {
        flag_SendingPacket = FALSE;
        SendingPacketFailed = TRUE;
        HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);
      }
    }
    else if((p->status & IRQ_CRC_ERR) != 0) {
      if (retries > 0) {
        HAL_RADIO_SendPacketWithAck(CHAT_FREQUENCYCHANNEL,CHAT_TRANSMIT_RELATIVETIME, sendData, receivedAckData, CHAT_RECEIVE_TIMEOUT, TxCallback);
        retries--;
      }
      else {
        flag_SendingPacket = FALSE;
        SendingPacketFailed = TRUE;
        HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);
      }
    }
  }
  /* Transmit complete */
  else {
  }
//  printf("tx");
  return TRUE;
}

/* UART echo and radio trigger */
void IOProcessInputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  static uint8_t cUartDataSize = 0;
  uint8_t i;

  for (i = 0; i < Nb_bytes; i++) {
    uart_buffer[cUartDataSize] = data_buffer[i];
    SdkEvalComIOSendData(data_buffer[i]);
    cUartDataSize++;

    if((uart_buffer[cUartDataSize-1] == '\r') || (cUartDataSize == (MAX_CHAT_PACKET_LEN - MIC_FIELD_LEN))) {
      xUartDataReady = SET;
      Osal_MemCpy(&sendData[2], uart_buffer, cUartDataSize);
			sendData[1] = cUartDataSize + MIC_FIELD_LEN;
      cUartDataSize = 0;
    }
  }
}

int main(void)
{
	uint8_t temp[MAX_PACKET_LENGTH];
	uint8_t length;

	/* System initialization function */
	SystemInit();

	GPIO_Configuration();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* UART initialization */
//	SdkEvalComUartInit(UART_BAUDRATE);
	SdkEvalComIOConfig(IOProcessInputData);


	ADC_Configuration();
	I2C_ConfigurationMaster();
//	MFT_Configuration();

	/* Start conversion */
	ADC_Cmd(ENABLE);

	/* Radio configuration - HS_STARTUP_TIME 642 us, external LS clock, NULL, whitening enabled */
#if LS_SOURCE==LS_SOURCE_INTERNAL_RO
	RADIO_Init(HS_STARTUP_TIME, 1, NULL, ENABLE);
#else
	RADIO_Init(HS_STARTUP_TIME, 0, NULL, ENABLE);
#endif

	/* Create the blueRec FIFO */
	fifo_init(&blueRec_fifo, MAX_PACKET_LENGTH*2, blueRec_buffer, 1);

	/* Set the Network ID */
	HAL_RADIO_SetNetworkID(BLE_ADV_ACCESS_ADDRESS);

	/* Configures the transmit power level */
	RADIO_SetTxPower(MAX_OUTPUT_RF_POWER);

#ifdef PKT_ENCRYPTION
	/* Set the encryption parameter */
	uint8_t RxCounter[5]    = {0,0,0,0,0};
	uint8_t TxCounter[5]    = {0,0,0,0,0};
	uint8_t encKey[16]      = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0xFF,0xFF};
	uint8_t encInitVector[8]= {0,0,0,0,0,0,0,0};

	RADIO_SetEncryptionCount(STATE_MACHINE_0, TxCounter, RxCounter);
	RADIO_SetEncryptionAttributes(STATE_MACHINE_0, encInitVector, encKey);
	RADIO_SetEncryptFlags(STATE_MACHINE_0, ENABLE, ENABLE);
#endif

	/* Receives a packet. Then sends a packet as an acknowledgment. */
	HAL_RADIO_ReceivePacketWithAck(CHAT_FREQUENCYCHANNEL, CHAT_RECEIVE_RELATIVETIME, receivedData, sendAckData, CHAT_RECEIVE_TIMEOUT, RxCallback);



	IhcInit();	/* indoor heating control initialization */

	HDC2080Read();
	AdcRead();
	IhcAppendChecksum(app_termo_buff, IHC_CFG_RADIO_PACKET_SIZE - 1);

    xUartDataReady = SET;
    Osal_MemCpy(&sendData[2], app_termo_buff, IHC_CFG_RADIO_PACKET_SIZE);
	sendData[1] = IHC_CFG_RADIO_PACKET_SIZE + MIC_FIELD_LEN;

	printf("START: satellite project\r\n");

	while(1) {
		/* Perform calibration procedure */
		RADIO_CrystalCheck();

		if(Bit_RESET != GPIO_ReadBit(GPIO_Pin_12))	/* HDC2080 measurement ready? */
		{
			HDC2080Read();
			AdcRead();
			IhcAppendChecksum(app_termo_buff, IHC_CFG_RADIO_PACKET_SIZE - 1);

		    xUartDataReady = SET;
		    Osal_MemCpy(&sendData[2], app_termo_buff, IHC_CFG_RADIO_PACKET_SIZE);
			sendData[1] = IHC_CFG_RADIO_PACKET_SIZE + MIC_FIELD_LEN;

//			BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO12, WAKEUP_IOx_HIGH << WAKEUP_IO12_SHIFT_MASK);
			sleep_cnt = 0;
		}
		else
		{
			if(100000 < sleep_cnt)
			{
				/* Cortex will enter in deep sleep mode when WFI command is executed */
//				SystemSleepCmd(ENABLE);
//				__WFI();
				BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO12, WAKEUP_IOx_HIGH << WAKEUP_IO12_SHIFT_MASK);
			}
			else
			{
				sleep_cnt++;
			}
		}

		/* SendingPacket == FALSE */
		if(flag_SendingPacket == FALSE) {

			if(xUartDataReady == SET) {
				/* enable sending */
				sendData[0] = 0x02;
				xUartDataReady = RESET;
				retries = MAX_RETRIES;
				flag_SendingPacket = TRUE;
			}
		}

		if(fifo_size(&blueRec_fifo) !=0) {
			fifo_get(&blueRec_fifo, HEADER_LENGTH, temp);
			length = temp[1];
			fifo_get(&blueRec_fifo, length, &temp[2]);

			/* -4 because the MIC field */
			for(uint8_t i= 2; i<(length+HEADER_LENGTH-MIC_FIELD_LEN); i++) {
//				printf("%c", temp[i]);
			}
		}
		else if(SendingPacketFailed == TRUE) {
			SendingPacketFailed = FALSE;
			printf("\n\r NACK \r\n");
			/* todo */
		}
//		BlueNRG_Sleep(SLEEPMODE_WAKETIMER, WAKEUP_IO12, WAKEUP_IOx_HIGH << WAKEUP_IO12_SHIFT_MASK);
	}

}


//void processCommand(void)
//{
//	uint8_t command;
//	uint32_t temp_var;
//	float temp;
//
//	/* Infinite loop */
//	while(1)
//	{
//		while (UART_GetFlagStatus(UART_FLAG_RXFE) == SET); /* Loop until the UART Receive Data Register is not empty */
//		command = UART_ReceiveData(); /* Store the received byte in RxBuffer */
//
//		switch (command & 0x0F) /* Parse the command */
//		{
//		case 0x01:
//		{
//
//			i2c_data.ptr_addr = 0xFC;
//			i2c_data.rx_size = 4;
//
//			I2C_Read_Reg(&i2c_data);
//			printf("MAN_ID_L: %d \r\n", i2c_data.rx_buff[0]);
//			printf("MAN_ID_H: %d \r\n", i2c_data.rx_buff[1]);
//			printf("DEV_ID_L: %d \r\n", i2c_data.rx_buff[2]);
//			printf("DEV_ID_H: %d \r\n", i2c_data.rx_buff[3]);
//
//			break;
//		}
//		case 0x02:
//		{
//			i2c_data.ptr_addr = 0x00;
//			i2c_data.rx_size = 4;
//			I2C_Read_Reg(&i2c_data);
//
//			temp_var = (i2c_data.rx_buff[1] << 8) + i2c_data.rx_buff[0];
//			temp = ((float)temp_var / 65536) * 165 - 40;
////			printf("TEMP_L: %u \r\n", i2c_data.rx_buff[0]);
////			printf("TEMP_H: %u \r\n", i2c_data.rx_buff[1]);
////			printf("RH_L: %u \r\n", i2c_data.rx_buff[2]);
////			printf("RH_H: %u \r\n", i2c_data.rx_buff[3]);
////			printf("TEMP_L: %ld \r\n", temp_var);
//			printf("HDC2080 temp: %.2f \r\n", temp);
//
//			break;
//		}
//		case 0x03:
//		{
//			i2c_data.ptr_addr = 0x07;
//			i2c_data.tx_size = 1;
//			i2c_data.tx_buff[0] = 0x80;
//			I2C_Write_Reg(&i2c_data);
//
//			i2c_data.ptr_addr = 0x0E;
//			i2c_data.tx_size = 2;
//			i2c_data.tx_buff[0] = 0x46;
//			i2c_data.tx_buff[1] = 0x01;
//			I2C_Write_Reg(&i2c_data);
//
//			printf("write 0x0E reg\r\n");
//			break;
//		}
//		case 0x04:
//		{
//			i2c_data.ptr_addr = 0x0E;
//			i2c_data.rx_size = 2;
//			I2C_Read_Reg(&i2c_data);
//			printf("0x0E reg: %d \r\n", i2c_data.rx_buff[0]);
//			printf("0x0F reg: %d \r\n", i2c_data.rx_buff[1]);
//			break;
//		}
//		default:
//			break;
//		}
//	}
//}

void GPIO_Configuration(void)
{
	GPIO_InitType GPIO_InitStructure;

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/* Put the outputs off */
	GPIO_WriteBit(GPIO_Pin_0, Bit_SET);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = ENABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_WriteBit(GPIO_Pin_0, Bit_SET);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Input;
	GPIO_Init(&GPIO_InitStructure);

	/* Set the GPIO interrupt priority and enable it */
	NVIC_InitType NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configures EXTI line */
	GPIO_EXTIConfigType GPIO_EXTIStructure;
	GPIO_EXTIStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
	GPIO_EXTIStructure.GPIO_Event = IRQ_ON_RISING_EDGE;
	GPIO_EXTIConfig(&GPIO_EXTIStructure);

	/* Clear pending interrupt */
	GPIO_ClearITPendingBit(GPIO_Pin_12);

	/* Enable the interrupt */
	GPIO_EXTICmd(GPIO_Pin_12, ENABLE);
}

/**
 * @brief  ADC_Configuration.
 * @param  None
 * @retval None
 */
void ADC_Configuration(void)
{
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);

	/* Configure ADC */
	xADC_InitType.ADC_OSR = ADC_OSR_200;
	//ADC_Input_BattSensor; //ADC_Input_TempSensor;// ADC_Input_AdcPin1 // ADC_Input_AdcPin12 // ADC_Input_AdcPin2
	xADC_InitType.ADC_Input = ADC_Input_BattSensor;
	xADC_InitType.ADC_ConversionMode = ADC_ConversionMode_Single;
	xADC_InitType.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
	xADC_InitType.ADC_Attenuation = ADC_Attenuation_9dB54;

	ADC_Init(&xADC_InitType);

	/* Enable auto offset correction */
	ADC_AutoOffsetUpdate(ENABLE);
	ADC_Calibration(ENABLE);
}

/**
 * @brief  MFT_Configuration.
 * @param  None
 * @retval None
 */
void MFT_Configuration(void)
{
	NVIC_InitType NVIC_InitStructure;

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX2, ENABLE);

	MFT2->TNMCTRL_b.TNEN = 0;		//disable timer
	MFT2->TNPRSC = 31;				//sysclk / (TNPRSC + 1) -> 1us
	MFT2->TNCKC_b.TNC2CSEL = 1; 	//system clock with prescaler
	MFT2->TNCRB = 10000;			//period = 1/(32MHz / ((TNPRSC + 1) * TNCRB))
	MFT2->TNMCTRL_b.TNMDSEL = 2;	//mode 3
	MFT2->TNICTRL_b.TNDIEN = 1;		//enable interrupt;
	MFT2->TNMCTRL_b.TNEN = 1;		//enable timer

	/* Enable MFT Interrupts */
	NVIC_InitStructure.NVIC_IRQChannel = MFT2B_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void I2C_ConfigurationMaster(void)
{
	GPIO_InitType GPIO_InitStructure;
	I2C_InitType I2C_InitStruct;

	/* Enable I2C and GPIO clocks */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C2 | CLOCK_PERIPH_GPIO, ENABLE);

	/* Configure I2C pins */
	GPIO_InitStructure.GPIO_Pin = SDK_EVAL_I2C_CLK_PIN ;
	GPIO_InitStructure.GPIO_Mode = SDK_EVAL_I2C_DATA_MODE;
	GPIO_InitStructure.GPIO_Pull = ENABLE;
	GPIO_InitStructure.GPIO_HighPwr = DISABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SDK_EVAL_I2C_DATA_PIN;
	GPIO_InitStructure.GPIO_Mode = SDK_EVAL_I2C_DATA_MODE;
	GPIO_Init(&GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Input;
//	GPIO_InitStructure.GPIO_Pull = DISABLE;
//	GPIO_InitStructure.GPIO_HighPwr = DISABLE;
//	GPIO_Init(&GPIO_InitStructure);

	/* Configure I2C in master mode */
	I2C_StructInit(&I2C_InitStruct);
	I2C_InitStruct.I2C_OperatingMode = I2C_OperatingMode_Master;
	I2C_InitStruct.I2C_ClockSpeed = SDK_EVAL_I2C_CLK_SPEED;
	I2C_Init((I2C_Type*)SDK_EVAL_I2C, &I2C_InitStruct);

	/* Clear all I2C pending interrupts */
	I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MSK);

}

I2C_AppOpStatus I2C_Read_Reg(i2c_data_t * data)
{
	I2C_TransactionType t;
	uint8_t idx;

	/* Write I2C device address address */
	t.Operation = I2C_Operation_Write;
	t.Address = data->dev_addr;
	t.StartByte = I2C_StartByte_Disable;
	t.AddressType = I2C_AddressType_7Bit;
	t.StopCondition = I2C_StopCondition_Disable;
	t.Length = 1;

	I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);

	/* wait until Flush finished */
	while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);

	/* Write I2C device address address and put the send_val in TX FIFO */
	I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
	I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, data->ptr_addr);

	/* Check write */
	do
	{
		if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
			return I2C_OP_FAILED;

	} while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS) == RESET);

	I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS);

	/* Prepare to read data */
	t.Operation = I2C_Operation_Read;
	t.Address = data->dev_addr;
	t.StartByte = I2C_StartByte_Disable;
	t.AddressType = I2C_AddressType_7Bit;
	t.StopCondition = I2C_StopCondition_Enable;
	t.Length = data->rx_size;

	I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);

	/* Check read */
	do
	{
		if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
			return I2C_OP_FAILED;

	} while (RESET == I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD));

	I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);

	/* Get data */
	for(idx = 0; idx < data->rx_size; idx++)
	{
		data->rx_buff[idx] = I2C_ReceiveData((I2C_Type*)SDK_EVAL_I2C);
	}

	return I2C_OP_SUCCEED;
}

I2C_AppOpStatus I2C_Write_Reg(i2c_data_t * data)
{
	I2C_TransactionType t;
	uint8_t idx;

	/* Write I2C device address address */
	t.Operation = I2C_Operation_Write;
	t.Address = data->dev_addr;
	t.StartByte = I2C_StartByte_Disable;
	t.AddressType = I2C_AddressType_7Bit;
	t.StopCondition = I2C_StopCondition_Enable;
	t.Length = data->tx_size + 1;

	I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);

	/* wait until Flush finished */
	while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);

	/* Write I2C device address address and put the send_val in TX FIFO */
	I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);

	I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, data->ptr_addr);

	for(idx = 0; idx < data->tx_size; idx++)
	{
		I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, data->tx_buff[idx]);
	}

	/* Check write */
	do
	{
		if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
			return I2C_OP_FAILED;

	} while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD) == RESET);

	I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);

	return I2C_OP_SUCCEED;
}

void HDC2080Init(void)
{
	uint16_t temp_val;

	i2c_data.dev_addr = HDC2080_ADDRESS;

	i2c_data.ptr_addr = 0xFC;
	i2c_data.rx_size = 4;

	I2C_Read_Reg(&i2c_data);

//	printf("MAN_ID_L: %d \r\n", i2c_data.rx_buff[0]);
//	printf("MAN_ID_H: %d \r\n", i2c_data.rx_buff[1]);
//	printf("DEV_ID_L: %d \r\n", i2c_data.rx_buff[2]);
//	printf("DEV_ID_H: %d \r\n", i2c_data.rx_buff[3]);

	temp_val = (((uint16_t)i2c_data.rx_buff[1]) << 8) + i2c_data.rx_buff[0];
	if(HDC2080_MANUFACTURER_ID != temp_val)
	{
		printf("HDC2080 wrong manufacturer ID\r\n");
		while(1);
	}
	temp_val = (((uint16_t)i2c_data.rx_buff[3]) << 8) + i2c_data.rx_buff[2];
	if(HDC2080_DEVICE_ID != temp_val)
	{
		printf("HDC2080 wrong device ID\r\n");
		while(1);
	}

	i2c_data.ptr_addr = 0x07;	//interrupt configuration register
	i2c_data.tx_size = 2;
	i2c_data.tx_buff[0] = 0x80;	//data ready interrupt enable
	/*
	 * 	1. Programming TEMP_OFFSET_ADJUST to 00000001 adjusts the reported temperature by +0.16°C.
		2. Programming TEMP_OFFSET_ADJUST to 00000111 adjusts the reported temperature by +1.12°C.
		3. Programming TEMP_OFFSET_ADJUST to 00001101 adjusts the reported temperature by +2.08°C.
		4. Programming TEMP_OFFSET_ADJUST to 11111111 adjusts the reported temperature by -0.16°C.
		5. Programming TEMP_OFFSET_ADJUST to 11111001 adjusts the reported temperature by -1.12°C.
		6. Programming TEMP_OFFSET_ADJUST to 11110011 adjusts the reported temperature by -2.08°C.
	 * */
	i2c_data.tx_buff[1] = 0xF8;	//temperature offset: +- 0.16'C / bit
	I2C_Write_Reg(&i2c_data);

	i2c_data.ptr_addr = 0x0E;
	i2c_data.tx_size = 2;
	i2c_data.tx_buff[0] = 0x36;	//configuration register
	i2c_data.tx_buff[1] = 0x01;	//measurement configuration register
	I2C_Write_Reg(&i2c_data);

	printf("HDC2080 OK!!!\r\n");
}
void HDC2080Read(void)
{
	uint32_t temp_var;
	float temp;
	uint16_t temp1;

	i2c_data.ptr_addr = 0x00;
	i2c_data.rx_size = 4;
	I2C_Read_Reg(&i2c_data);

	temp_var = (i2c_data.rx_buff[1] << 8) + i2c_data.rx_buff[0];
	temp = ((float)temp_var / 65536) * 165 - 40;	//hdc2080 formula
	if(5 <= (((uint16_t)(temp * 100)) % 10))
	{
		temp1 = (uint16_t)(temp*10) + 1;
	}
	else
	{
		temp1 = (uint16_t)(temp*10);
	}

//	printf("Tx test: %d ['C]\r\n", temp1);

//	printf("Tx temp: %.4f ['C]\r\n", temp);

	app_termo_buff[1] = (uint8_t)temp1;
	app_termo_buff[2] = (uint8_t)(temp1 >> 8);
	temp_var = (i2c_data.rx_buff[3] << 8) + i2c_data.rx_buff[2];
	temp = ((float)temp_var / 65536) * 100;	//hdc2080 formula
	temp1 = (uint16_t)temp;

//	printf("Tx rh: %.4f [%%]\r\n", temp);

	app_termo_buff[3] = (uint8_t)temp1;
	app_termo_buff[4] = (uint8_t)(temp1 >> 8);
}

void IhcInit(void)
{
//	if(Bit_RESET != GPIO_ReadBit(GPIO_Pin_12))	/* HDC2080 measurement ready? */
//	{
//		app_data.state = state_ready;
//	}
//	else
	{
		app_data.state = state_init;
	}

	app_termo_buff[0] = IHC_CFG_RADIO_PACKET_SIZE;

	HDC2080Init();
}

void AdcRead(void)
{
	float adc_value;
	uint16_t temp;

	/* Polling of End-Of-Conversion flag */
	if(ADC_GetFlagStatus(ADC_FLAG_EOC)) {

		/* Read converted data */
		adc_value = ADC_GetConvertedData(xADC_InitType.ADC_Input, xADC_InitType.ADC_ReferenceVoltage);

		/* Print the ADC value converted */
		if(xADC_InitType.ADC_Input == ADC_Input_TempSensor) {
			printf("BlueNRG-1 temp: %.1f ['C]\r\n", adc_value);
		}
		else {
//			printf("Battery: %.3f [V]\r\n", adc_value);//*1000.0);
		}
		temp = (uint16_t)(adc_value * 1000);
		app_termo_buff[5] = (uint8_t)temp;
		app_termo_buff[6] = (uint8_t)(temp >> 8);

//		printf("Batt: %d [mV]\r\n", temp);

		ADC_Cmd(ENABLE); /* Start new conversion - todo */
	}
}

void IhcMain(void)
{
	if(state_init == app_data.state)
	{
		HDC2080Init();
		app_data.state = state_ready;
	}
	else if(state_ready == app_data.state)
	{
		if(Bit_RESET != GPIO_ReadBit(GPIO_Pin_12))	/* HDC2080 measurement ready? */
		{
			HDC2080Read();
			AdcRead();
			IhcAppendChecksum(app_termo_buff, IHC_CFG_RADIO_PACKET_SIZE - 1);

		    xUartDataReady = SET;
		    Osal_MemCpy(&sendData[2], app_termo_buff, IHC_CFG_RADIO_PACKET_SIZE);
			sendData[1] = IHC_CFG_RADIO_PACKET_SIZE + MIC_FIELD_LEN;

//			BlueNRG_Sleep(SLEEPMODE_WAKETIMER, WAKEUP_IO12, WAKEUP_IOx_HIGH << WAKEUP_IO12_SHIFT_MASK);
//			app_data.state = state_off;
//			ihc_off_ms = 0;
		}
	}
	else if(state_off == app_data.state)
	{
		if(90 < ihc_off_ms)
		{
//			GPIO_WriteBit(GPIO_Pin_0, Bit_RESET);
			app_data.state = state_ready;
			BlueNRG_Sleep(SLEEPMODE_WAKETIMER, WAKEUP_IO12, WAKEUP_IOx_HIGH << WAKEUP_IO12_SHIFT_MASK);
		}
		else
		{
			ihc_off_ms += 10;
		}
	}
}

inline uint8_t IhcChecksum(uint8_t * data, uint8_t size)
{
	uint8_t idx, checksum = 0;

	for(idx = 0; idx < size; idx++)
	{
		checksum = checksum + *(data + idx);
	}

	return checksum;
}

void IhcAppendChecksum(uint8_t * data, uint8_t size)
{
	*(data + size) = IhcChecksum(data, size);
}

uint8_t IhcChecksumValidate(uint8_t * data, uint8_t size)
{
	uint8_t retval, checksum;

	checksum = IhcChecksum(data, size);

	if(*(data + size) == checksum)
	{
		retval = 1;
	}
	else
	{
		retval = 0;
	}

	return retval;
}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
