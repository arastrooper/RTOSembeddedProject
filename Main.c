/**************************************************************************//**
 *
 * @file        Main.c
 * @brief       FreeRTOS Examples
 * @author      Geoffrey Daniels
 * @author		Jez Dalton and Sam Walder
 * @version     1.21 (GW)
 * @date        17/02/2015
 *
 * Copyright(C) 2012, Geoffrey Daniels, GPDaniels.com
 * Copyright(C) 2015, Jeremy Dalton, jd0185@my.bristol.ac.uk
 * All rights reserved.
 *
******************************************************************************/
/******************************************************************************
 * FreeRTOS includes.
 *****************************************************************************/
#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_Task.h"
#include "FreeRTOS_Queue.h"
#include "FreeRTOS_Timers.h"
#include "FreeRTOS_Semaphore.h"

/******************************************************************************
 * Library includes.
 *****************************************************************************/
#include "stdio.h"
#include "LPC17xx.h"
#include "LPC17xx_GPIO.h"
#include "Queee.h"
#include "Font5x7.h"


/******************************************************************************
 * Defines and typedefs
 *****************************************************************************/
#define SOFTWARE_TIMER_PERIOD_MS (1000 / portTICK_RATE_MS)	// The timer period (1 second)
#define WAVPLAYER_INCLUDE_SAMPLESONGS						// Include the sample in WavPlayer_Sample.h
//#define PutStringOLED PutStringOLED1						// Select which to use
#define PutStringOLED PutStringOLED2						// Select which to use
//#define PutStringOLED PutStringOLED3
//#define PutStringOLED PutStringOLED4
//#define PutStringOLED PutStringOLED5
// PCADC / PCAD
#define ADC_POWERON (1 << 12)  //12th bit
#define PCLK_ADC 24
#define PCLK_ADC_MASK (3 << 24) //3 followed by 24 zeroes
// AD0.0 - P0.23, PINSEL1 [15:14] = 01
#define SELECT_ADC0 (0x1<<14) //just a shift
// ADOCR constants
#define START_ADC (1<<24)
#define OPERATIONAL_ADC (1 << 21)
#define SEL_AD0 (1 <<0)
#define ADC_DONE_BIT	(1 << 31)
/******************************************************************************
 * Library includes.
 *****************************************************************************/


#include "dfrobot.h"
#include "pca9532.h"
#include "joystick.h"
#include "OLED.h"
#include "WavPlayer.h"

/******************************************************************************
 * Global variables
 *****************************************************************************/
// Variable defining the SPI port, used by the OLED and 7 segment display
Peripheral_Descriptor_t SPIPort;

// Fixed Seven segment values. Encoded to be upside down.
static const uint8_t SevenSegmentDecoder[] = {0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x30};

// Variables associated with the software timer
static xTimerHandle SoftwareTimer = NULL;
uint8_t Seconds, Minutes, Hours;

// Variables associated with the WEEE navigation
int dx = 0, dy = 0, cx = 0, cy = 0,OldCurrentY=0,OldCurrentX=0;

//wheelcount
uint32_t LeftWheel = 0, Play =0 , Pause = 0;
uint32_t RightWheel = 0;
volatile uint32_t n = 0;
volatile uint32_t k = 0;
volatile uint32_t o = 0;
volatile uint32_t p = 0;
volatile uint32_t i = 0;
uint32_t Distance = 1;
int DestX = 0, TurnX = 0, DistanceX = 0, DistanceY =0;
int DestY = 0, TurnY = 0,  WheelCounts = 0, WheelCounts2 =0;
int s0 = 0,s1 = 0,s2 = 0, s3 =0, pat1 = 1, pat2 = 0, pat3 = 0;; //drive states


//OLED semaphores
xSemaphoreHandle DisplayEnabler= 0;

//SPI vs OLED semaphore
xSemaphoreHandle SPI_OLED = 0;

uint8_t Gear = 1; //byte
uint32_t L =10, R = 10; //byte
uint16_t ADCval; //half word

char bufferD[32];
char bufferC[32];

/******************************************************************************
 * Local functions
 *****************************************************************************/
static uint8_t const  Font_Mask[8] = {0x80, 0x40, 0x20, 0X10, 0X08, 0X04, 0X02, 0X01};


/******************************************************************************
 * Task Defintions
 *****************************************************************************/
/******************************************************************************
 * Description:	The callback function assigned to the SoftwareTimer.
 *
 *****************************************************************************/
static void SoftwareTimerCallback(xTimerHandle xTimer)
{
    (void)xTimer;

	// Increment timers, inside critical so that they can't be accessed while updating them
	taskENTER_CRITICAL();
		++Seconds;
		if (Seconds == 60) { Seconds = 0; ++Minutes; }
		if (Minutes == 60) { Minutes = 0; ++Hours; }
	taskEXIT_CRITICAL();
}


/******************************************************************************
 * Description:	OLED helper writing functions. Put out entire string
 *				in one critical section.
 *****************************************************************************/
void PutStringOLED1(uint8_t* String, uint8_t Line)
{
	uint8_t X = 2;
	uint8_t Ret = 1;
	while(1)
	{
		if ((*String)=='\0')
			break;

		//taskENTER_CRITICAL();
			Ret = OLED_Char(X, ((Line)%7)*9 + 1, *String++, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		//taskEXIT_CRITICAL();

		if (Ret == 0)
			break;
		X += 6;
	}
}


/******************************************************************************
 * Description:	Put out characters one by one, each in a critical section
 *
 *****************************************************************************/
void PutStringOLED2(uint8_t* String, uint8_t Line)
{
	//uint8_t rx_char;
	//int rx_flag = 0;
	///const portTickType TaskPeriodms = 10UL / portTICK_RATE_MS;
	//if(xSemaphoreTake(SPI_OLED, TaskPeriodms) == pdTRUE) {
	//vSemaphoreCreateBinary(DisplayControl);
	//if(xSemaphoreTake(SPI_OLED, 500UL/portTICK_RATE_MS ) == pdTRUE) {
	//if (xQueueReceive(QueueChar, &rx_char, 10)  && xQueueReceive(QueueLine, &rx_line, 10))  {
	//if (xQueueReceive(QueueFlag, &rx_flag, 10)) {
		//if (rx_flag != 1) {
			//taskENTER_CRITICAL();
	OLED_String(2,  ((Line)%7)*9 + 1, String, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			//taskEXIT_CRITICAL();
			//rx_flag = 0;
		//}
	//}
	//}
	//xSemaphoreGive(DisplayControl);
	//xSemaphoreGive(SPI_OLED);	//}

	//vTaskDelay(TaskPeriodms);
}



/******************************************************************************
 * Description:	This task counts seconds and shows the number on the seven
 *				segment display
 *****************************************************************************/
static void SevenSegmentTask(void *pvParameters)
{
	const portTickType TaskPeriodms = 900UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	uint8_t i = 0;
	(void)pvParameters;

	// Initialise LastExecutionTime prior to the first call to vTaskDelayUntil().
	// This only needs to be done once, as after this call, LastExectionTime is updated inside vTaskDelayUntil.
	LastExecutionTime = xTaskGetTickCount();
	for(;;)
	{
		if(xSemaphoreTake(SPI_OLED, 500UL/portTICK_RATE_MS ) == pdTRUE) {
			for(i = 0; i < 10; ++i)
			{
				// Critical section here so that we don't use the SPI at the same time as the OLED
				//taskENTER_CRITICAL();
					board7SEG_ASSERT_CS();
						FreeRTOS_write(SPIPort, &(SevenSegmentDecoder[i]), sizeof(uint8_t));
					board7SEG_DEASSERT_CS();
				//taskEXIT_CRITICAL();

				// Delay until it is time to update the display with a new digit.
				xSemaphoreGive(SPI_OLED);
				vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
			}
		} else {
			//vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
		}
	}
}

/******************************************************************************
 * Description:	This task makes the top four lines of the OLED black boxes
 *
 *****************************************************************************/
static void OLEDTask1(void *pvParameters)
{
	const portTickType TaskPeriodms = 1000UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();
	DisplayEnabler = xSemaphoreCreateMutex();
	uint8_t Char;
	int Line;
	for(;;)
	{
		if(xSemaphoreTake(DisplayEnabler, TaskPeriodms) == pdTRUE && xSemaphoreTake(SPI_OLED, TaskPeriodms) == pdTRUE) {
			PutStringOLED((uint8_t*)"", 0);
			vTaskDelay((portTickType)100);
			PutStringOLED((uint8_t*)"", 1);
			vTaskDelay((portTickType)100);
			PutStringOLED((uint8_t*)"", 2);
			xSemaphoreGive(DisplayEnabler);
			xSemaphoreGive(SPI_OLED);
		}
		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task makes the top four lines of the OLED empty
 *
 *****************************************************************************/
static void OLEDTask2(void *pvParameters)
{
	const portTickType TaskPeriodms = 1000UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();
	for(;;)
	{
		if(xSemaphoreTake(DisplayEnabler, TaskPeriodms) == pdTRUE && xSemaphoreTake(SPI_OLED, TaskPeriodms) == pdTRUE ) {
			PutStringOLED((uint8_t*)"                ", 0);
			vTaskDelay((portTickType)100);
			PutStringOLED((uint8_t*)"                ", 1);
			vTaskDelay((portTickType)100);
			PutStringOLED((uint8_t*)"                ", 2);
			xSemaphoreGive(DisplayEnabler);
			xSemaphoreGive(SPI_OLED);
		}
		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task makes the top four lines of the OLED a char
 *
 *****************************************************************************/
static void OLEDTask3(void *pvParameters)
{
	const portTickType TaskPeriodms = 1000UL / portTICK_RATE_MS;
	portTickType LastExecutionTime;
	(void)pvParameters;
	LastExecutionTime = xTaskGetTickCount();
	for(;;)
	{
		if(xSemaphoreTake(DisplayEnabler, TaskPeriodms) == pdTRUE && xSemaphoreTake(SPI_OLED, TaskPeriodms) == pdTRUE) {
			PutStringOLED((uint8_t*)"<<<<<<<<<<<<<<< ", 0);
			vTaskDelay((portTickType)100);
			PutStringOLED((uint8_t*)" >>>>>>>>>>>>>>>", 1);
			vTaskDelay((portTickType)100);
			PutStringOLED((uint8_t*)"<<<<<<<<<<<<<<< ", 2);
			xSemaphoreGive(DisplayEnabler);
			xSemaphoreGive(SPI_OLED);
		}
		vTaskDelayUntil(&LastExecutionTime, TaskPeriodms);
	}
}


/******************************************************************************
 * Description:	This task displays a moving + on a bar of -  LINE3
 *
 *****************************************************************************/
static void OLEDTask4(void *pvParameters)
{
	const portTickType TaskPeriodms = 100UL / portTICK_RATE_MS;
	char Buffer[17] = "----------------";
	uint8_t Up = 1;
	uint8_t ID = 0;
	(void)pvParameters;
	SPI_OLED = xSemaphoreCreateMutex();
	for(;;)
	{
		if ( xSemaphoreTake(SPI_OLED, TaskPeriodms) == pdTRUE) {
			if (Up)
				Buffer[ID] = '+';
			else
				Buffer[ID] = '-';

			if (ID == 15) { ID = 0; Up = !Up; }
			else { ++ID; }

			PutStringOLED((uint8_t*)Buffer, 3);
			xSemaphoreGive(SPI_OLED);
		}
		vTaskDelay(TaskPeriodms);
	}
}

/******************************************************************************
 * Description: Read User Input
 *
 *****************************************************************************/
static void WEEEInputTask(void *pvParameters)
{
	const portTickType TaskPeriodms =100UL / portTICK_RATE_MS;
	(void)pvParameters;

	long test;

	for(;;)
	{

		vTaskDelay(TaskPeriodms);
	}
}


/******************************************************************************
 * Description: Write Input to Display LINES 5 and 6
 *
 *****************************************************************************/
static void WEEEDisplayTask(void *pvParameters)
{
	const portTickType TaskPeriodms =200UL / portTICK_RATE_MS;
	(void)pvParameters;

	long test;

	for(;;)
	{
		if ( xSemaphoreTake(SPI_OLED, TaskPeriodms) == pdTRUE) {
			if ( Play == 1 && Pause == 0) {
				PutStringOLED((uint8_t*)" Tune: Playing  ", 4);
			} else if (Play == 0) {
				PutStringOLED((uint8_t*)" Tune: Stopped  ", 4);
			} else if (Play == 1 && Pause == 1) {
				PutStringOLED((uint8_t*)" Tune: Paused  ", 4);
			}
			// Show Destination X and Y | Current X and Y
			sprintf(bufferD, "Des:%d,%dCur:%d,%d ", dx, dy,cx,cy);
			PutStringOLED(bufferD,5);
			xSemaphoreGive(SPI_OLED);
		}
		vTaskDelay(TaskPeriodms);
	}
}


/******************************************************************************
 * Description: Move UP AND DOWN
 *
 *****************************************************************************/
static void WEEEOutputTask(void *pvParameters)
{
	const portTickType TaskPeriodms =50UL / portTICK_RATE_MS;

	(void)pvParameters;
	long test;

	for(;;)
	{
		//move forward
		if(OldCurrentY < DestY && s0 == 1) {
			 if(WheelCounts<DistanceY) {
				DFR_DriveForward(100);
				// wheel power throttling
				if (n == k) {
					DFR_SetLeftDrive (DFR_FORWARD, 100);
					DFR_SetRightDrive (DFR_FORWARD, 100);
				} else if (n >= k) {
					DFR_SetLeftDrive (DFR_FORWARD, 100);
					DFR_SetRightDrive (DFR_FORWARD, 95);
				} else if (k>=n) {
					DFR_SetLeftDrive (DFR_FORWARD, 95);
					DFR_SetRightDrive (DFR_FORWARD, 100);
				}
			 } else {
				 DistanceY = 0;
				 OldCurrentY = DistanceY;
				 s0 = 0;
				 s1 = 1; //enable the next step
				 L =4, R =4; //set wheel counts to 5 as next step is turning
				 WheelCounts = 0;
			 }
		}


		//move backward
		else if(OldCurrentY > DestY && s0 == 1) {
			if(WheelCounts<DistanceY) {
				if (n == k) {
					DFR_SetLeftDrive (DFR_REVERSE, 100);
					DFR_SetRightDrive (DFR_REVERSE, 100);
				} else if (n >= k) {
					DFR_SetLeftDrive (DFR_REVERSE, 100);
					DFR_SetRightDrive (DFR_REVERSE, 95);
				} else if (k>=n) {
					DFR_SetLeftDrive (DFR_REVERSE, 95);
					DFR_SetRightDrive (DFR_REVERSE, 100);
				}
			} else {
				DistanceY = 0;
				OldCurrentY = DistanceY;
				s0 = 0;
				s1 = 1;
				L =3, R =3;
				WheelCounts = 0;
			}
		}

		//turn left - three sequential states
		if(OldCurrentX > TurnX) {

			if (s1 ==1)  {
				if(WheelCounts<1) {
					DFR_DriveLeft(100);
				} else {
					WheelCounts = 0;
					s2 = 1; //enable the next state
					s1 = 0;
					L =10, R =10; //set wheel count to 10 for the next step
					WheelCounts = 0;
				}
			}

			if (s2 == 1) {
				if(WheelCounts<DistanceX) {
					DFR_DriveForward(100);
				 } else {

					 s2 =0;
					 s3 =1; //enable the next step
					 L = 3, R =3;
					 WheelCounts = 0;
					 DistanceX = 0;
				 }

			}

			if (s3 == 1) {
				if(WheelCounts<1) {
					DFR_DriveRight(63);
				} else {
					WheelCounts = 0;
					s3 = 0;
					L = 10, R =10; // end of movement
					OldCurrentX = DistanceX;
					TurnX = OldCurrentX;
				}
			}

		}

		//turn right
		if(OldCurrentX < TurnX) {
			if (s1 ==1)  {
				if(WheelCounts<1) {
					DFR_DriveRight(100);
				} else {
					WheelCounts = 0;
					s2 = 1; //enable the next state
					s1 = 0;
					L =10, R =10;
				}
			}

			if (s2 == 1) {
				if(WheelCounts<DistanceX) {
					DFR_DriveForward(100);
				 } else {
					 WheelCounts = 0;
					 s2 =0;
					 s3 =1; //enable the next step
					 L = 3, R =3;
				 }

			}

			if (s3 == 1) {
				if(WheelCounts<1) {
					DFR_DriveLeft(70);
				} else {
					WheelCounts = 0;
					s3 = 0;
					L = 10, R =10; // end of movement
					DistanceX = 0;
					OldCurrentX = DistanceX;
					TurnX = OldCurrentX;
				}
			}

		}
		vTaskDelay(TaskPeriodms);
	}
}



/******************************************************************************
 * Description:
 *
 *****************************************************************************/
int main(void)
{

	// Globally disable interrupts
	__disable_irq();

	// Globally enable interrupts
	__enable_irq();

	 WheelCounts =0;
	// The examples assume that all priority bits are assigned as preemption priority bits.
    NVIC_SetPriorityGrouping(0UL);

    // Init SPI...
    SPIPort = FreeRTOS_open(board_SSP_PORT, (uint32_t)((void*)0));

    // Init 7seg
    GPIO_SetDir(board7SEG_CS_PORT, board7SEG_CS_PIN, boardGPIO_OUTPUT );
    board7SEG_DEASSERT_CS();

    // Init OLED
    OLED_Init(SPIPort);
    OLED_ClearScreen(OLED_COLOR_WHITE);

   	// Init wav player
   	WavPlayer_Init();

   	// Joystick Init
   	joystick_init();

   	// LED Banks Init
   	pca9532_init();

   	// Init Chassis Driver
   	DFR_RobotInit();

   	// Enable GPIO Interrupts
   	GPIO_IntCmd(0, 1<<4| 1 << 15 | 1 << 16 | 1 << 17, 0);
   	GPIO_IntCmd(2,1 << 3 | 1 << 4 | 1<< 11 | 1<< 12, 0);
   	NVIC_SetPriority(EINT3_IRQn, ((0x01<<3)|0x01)); //less than timer0
   	NVIC_EnableIRQ(EINT3_IRQn);


   	/*
    // Create a software timer
   	SoftwareTimer = xTimerCreate((const int8_t*)"TIMER",   // Just a text name to associate with the timer, useful for debugging, but not used by the kernel.
                                 SOFTWARE_TIMER_PERIOD_MS, // The period of the timer.
                                 pdTRUE,                   // This timer will autoreload, so uxAutoReload is set to pdTRUE.
                                 NULL,                     // The timer ID is not used, so can be set to NULL.
                                 SoftwareTimerCallback);   // The callback function executed each time the timer expires.
    xTimerStart(SoftwareTimer, portMAX_DELAY);*/

    // Create the Seven Segment task
    xTaskCreate(SevenSegmentTask,               // The task that uses the SPI peripheral and seven segment display.
                (const int8_t* const)"7SEG",    // Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself.
                configMINIMAL_STACK_SIZE*2,     // The size of the stack allocated to the task.
                NULL,                           // The parameter is not used, so NULL is passed.
                1U,                             // The priority allocated to the task.
                NULL);                          // A handle to the task being created is not required, so just pass in NULL.

    // Create the tasks
    xTaskCreate(OLEDTask1, 			(const int8_t* const)"OLED1", 		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(OLEDTask2, 			(const int8_t* const)"OLED2", 		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(OLEDTask3, 			(const int8_t* const)"OLED3", 		configMINIMAL_STACK_SIZE*2, NULL, 0U, NULL);
    xTaskCreate(OLEDTask4, 			(const int8_t* const)"plusminus", 	configMINIMAL_STACK_SIZE*2, NULL, 3U, NULL);
    xTaskCreate(WEEEDisplayTask,	(const int8_t* const)"Display",		configMINIMAL_STACK_SIZE*2, NULL, 2U, NULL);
    xTaskCreate(WEEEOutputTask,		(const int8_t* const)"move",		configMINIMAL_STACK_SIZE*4, NULL, 4U, NULL);

	// Start the FreeRTOS scheduler.
	vTaskStartScheduler();
	//xQueueCreate( 10, 10);
	// The following line should never execute.
	// If it does, it means there was insufficient FreeRTOS heap memory available to create the Idle and/or timer tasks.
    for(;;);
}


/******************************************************************************
 * Interrupt Service Routines
 *****************************************************************************/
void EINT3_IRQHandler (void)
{
	if ( (((LPC_GPIOINT->IO2IntStatR) >> 11)& 0x1) == ENABLE)
	{
		if (LeftWheel < Distance) { //10 interrupts = one wheel rotation
			if (n<L) {
				n++;
			} else {
				LeftWheel++;
				WheelCounts++;
				n = 0;
			}
		} else {
			DFR_DriveStop(); //stop here when travelled Destination
			LeftWheel  =0;
		}
	}
	if ((((LPC_GPIOINT->IO2IntStatR) >> 12)& 0x1) == ENABLE)
	{
		if (RightWheel < Distance) {
			if (k<R) {
				k++;
			} else {
				RightWheel++;
				k = 0;
			}
		} else {
			DFR_DriveStop();
			RightWheel = 0;
		}
	}
	GPIO_ClearInt(2,1 << 12 | 1 << 11);
	if ((((LPC_GPIOINT->IO2IntStatR) >> 3)& 0x1) == ENABLE) // up
	{
		if ((((GPIO_ReadValue(1) >> 31) & 0x01) == 0)) {
			DFR_IncGear();
			Gear = DFR_GetGear();
		} else {
			dy += 1;
		}
	}

	else if ((((LPC_GPIOINT->IO0IntStatR) >> 15)& 0x1) == ENABLE) //down
	{
		if ((((GPIO_ReadValue(1) >> 31) & 0x01)) == 0) {
			DFR_DecGear();
			Gear = DFR_GetGear();
		} else {
			dy -= 1;
		}
	}
	else if ((((LPC_GPIOINT->IO2IntStatR) >> 4)& 0x1) == ENABLE) //left
	{
		dx -= 1;
	} else if ((((LPC_GPIOINT->IO0IntStatR) >> 16)& 0x1) == ENABLE) //right
	{
		dx += 1;
	}
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 17)& 0x1) == ENABLE) //centre
	{
		DFR_DriveStop();
		DestX = dx;
		DestY = dy;
		TurnX = dx;
		TurnY = dy;
		OldCurrentY = cy;
		OldCurrentX = cx;
		cx = DestX;
		cy = DestY;
		dx = 0;
		dy = 0;
		WheelCounts = 0;
		DistanceY = abs(DestY-OldCurrentY); //distance to travel
		DistanceX = abs(DestX-OldCurrentX);
		if(DestY == OldCurrentY) {
			s1 = 1;
			s0=0,s2=0,s3 = 0;
			L = 5, R =5;
		} else {
			s0 = 1;
			s1 =0,s2=0,s3=0;
			L = 10, R =10;
		}
	}
	//button
	else if ((((LPC_GPIOINT->IO0IntStatR) >> 4)& 0x1) == ENABLE) {
		if(((GPIO_ReadValue(1) >> 31) & 0x01) == 0 && Play == 0) {
			WavPlayer_Play(WavPlayer_Sample, WavPlayer_SampleLength);
			Play = 1;
			Pause = 0;
		} else if(((GPIO_ReadValue(1) >> 31) & 0x01) == 0 && Play == 1) {
			WavPlayer_Stop();
			Play = 0;
			Pause = 0;
		} else if (Pause == 0 && Play == 1) {
			Pause = 1;
			WavPlayer_Pause();
		} else if (Pause == 1 && Play == 1) {
			WavPlayer_Pause();
			Pause =0;
		}
	}
	// Clear GPIO Interrupt Flags
	GPIO_ClearInt(0, 1<<4 |  1 << 15| 1 << 16| 1 << 17);
	// Joystick | Joystick | Encoder | Encoder
	GPIO_ClearInt(2,1 << 3 |1 << 4);
}
/******************************************************************************
 * Error Checking Routines
 *****************************************************************************/
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	// Unused variables
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	//taskDISABLE_INTERRUPTS();
    for(;;);
}


void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	//taskDISABLE_INTERRUPTS();
    for(;;);
}
