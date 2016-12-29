/***************************************************************************//**
 *
 * @file		OLED.c
 * @brief		Driver for the OLED Display (SSD1305)
 * @author		Embedded Artists AB
 * @author		Geoffrey Daniels
 * @version		1.0
 * @date		14 March. 2012
 * @warning		Initialize I2C or SPI, and GPIO before calling any functions in
 *              this file.
 *
 * Copyright(C) 2009, Embedded Artists AB
 * All rights reserved.
 *
*******************************************************************************/

#include <string.h>

// Includes
#include "FreeRTOS.h"
#include "FreeRTOS_Task.h"
#include "FreeRTOS_Queue.h"
#include "FreeRTOS_Semaphore.h"
#include "FreeRTOS_IO.h"

#include "LPC17xx_GPIO.h"

#include "OLED.h"
#include "Font5x7.h"

//------------------------------------------------------------------------------

// Defines and typedefs
#define OLED_CS_OFF() GPIO_SetValue(0, (1<<6))
#define OLED_CS_ON()  GPIO_ClearValue(0, (1<<6))
#define OLED_DATA()   GPIO_SetValue(2, (1<<7))
#define OLED_CMD()    GPIO_ClearValue(2, (1<<7))

// The display controller can handle a resolutiom of 132x64. The OLED on the
// base board is 96x64.
#define X_OFFSET 18

#define SHADOW_FB_SIZE (OLED_DISPLAY_WIDTH*OLED_DISPLAY_HEIGHT >> 3)

#define SetAddress(Page, LowerAddress, HigherAddress)\
    WriteCommand(Page);\
    WriteCommand(LowerAddress);\
    WriteCommand(HigherAddress);

//------------------------------------------------------------------------------

// External global variables
//...

//------------------------------------------------------------------------------

// Local variables
static uint8_t const  Font_Mask[8] = {0x80, 0x40, 0x20, 0X10, 0X08, 0X04, 0X02, 0X01};

// The SSD1305 doesn't support reading from the display memory when using serial
// mode (only parallel mode). Since it isn't possible to write only one pixel to
// the display (a minimum of one column, 8 pixels, is always wriiten) a shadow
// framebuffer is needed to keep track of the display data.
static uint8_t ShadowFB[SHADOW_FB_SIZE];

static Peripheral_Descriptor_t SPIPort;

//------------------------------------------------------------------------------

// Local Functions
static void WriteCommand(uint8_t Data)
{
    // Indicate incoming command
    OLED_CMD();

    // Select the OLED
    OLED_CS_ON();

    FreeRTOS_write(SPIPort, &Data, 1);

    // De-select the OLED
    OLED_CS_OFF();
}

static void WriteData(uint8_t Data)
{
    // Indicate incoming data
    OLED_DATA();

    // Select the OLED
    OLED_CS_ON();
    //taskENTER_CRITICAL();
    FreeRTOS_write(SPIPort, &Data, 1);
    //taskEXIT_CRITICAL();

    // De-select the OLED
    OLED_CS_OFF();
}

/// @todo Optimize (at least from a RAM point of view)
static void WriteDataLength(unsigned char Data, unsigned int Length)
{
    uint8_t Buffer[140];
    int i;

    // Fill buffer
    for (i = 0; i < Length; i++)
        Buffer[i] = Data;

    // Indicate incoming data
    OLED_DATA();

    // Select the OLED
    OLED_CS_ON();

	// FIX TO USE FREERTOS and I2CPort
    FreeRTOS_write(SPIPort, Buffer, Length);

    // De-select the OLED
    OLED_CS_OFF();
}

static void RunInitSequence(void)
{
    // Recommended Initial code according to manufacturer

    WriteCommand(0X02); // Set low column address
    WriteCommand(0X12); // Set high column address
    WriteCommand(0x40); // (Display start set)
    WriteCommand(0x2e); // (Stop horzontal scroll)
    WriteCommand(0x81); // (Set contrast control register)
    WriteCommand(0x32); // ?
    WriteCommand(0x82); // (Brightness for color banks)
    WriteCommand(0x80); // (Display on)
    WriteCommand(0xa1); // (Set segment re-map)
    WriteCommand(0xa6); // (Set normal/inverse display)
  //WriteCommand(0xa7); // (Set inverse display)
    WriteCommand(0xa8); // (Set multiplex ratio)
    WriteCommand(0x3F); // ?
    WriteCommand(0xd3); // (Set display offset)
    WriteCommand(0x40); // ?
    WriteCommand(0xad); // (Set dc-dc on/off)
    WriteCommand(0x8E); // ?
    WriteCommand(0xc8); // (Set com output scan direction)
    WriteCommand(0xd5); // (Set display clock divide ratio/oscillator/frequency)
    WriteCommand(0xf0); // ?
    WriteCommand(0xd8); // (Set area color mode on/off & low power display mode )
    WriteCommand(0X05); // ?
    WriteCommand(0xd9); // (Set pre-charge period)
    WriteCommand(0xF1); // ?
    WriteCommand(0xda); // (Set com pins hardware configuration)
    WriteCommand(0X12); // ?
    WriteCommand(0xdb); // (Set vcom deselect level)
    WriteCommand(0x34); // ?
    WriteCommand(0x91); // (Set look up table for area color)
    WriteCommand(0x3f); // ?
    WriteCommand(0x3f); // ?
    WriteCommand(0x3f); // ?
    WriteCommand(0x3f); // ?
    WriteCommand(0xaf); // (Display on)
    WriteCommand(0xa4); // (Display on)
}

// Public Functions
void OLED_Init(Peripheral_Descriptor_t SPIPortIn)
{
    volatile int Delay = 0;

    SPIPort = SPIPortIn;

    //GPIO_SetDir(PORT0, 0, 1);
    GPIO_SetDir(2, (1<<1), 1);
    GPIO_SetDir(2, (1<<7), 1);
    GPIO_SetDir(0, (1<<6), 1);

    // Make sure power is off
    GPIO_ClearValue(2, (1<<1) );

    OLED_CS_OFF();

    // Send the initialization commands to the display
    RunInitSequence();

    // Zero the shadow framebuffer
    memset(ShadowFB, 0, SHADOW_FB_SIZE);

    // Small delay before turning on power
    for (Delay = 0; Delay < 0xffff; Delay++);

    // Power on
    GPIO_SetValue( 2, (1<<1) );
}

void OLED_ClearScreen(OLED_Colour Colour)
{
    uint8_t i;
    uint8_t c = 0;

    if (Colour == OLED_COLOR_WHITE)
        c = 0xff;

    // Go through all 8 pages
    for(i=0xB0; i<0xB8; i++)
    {
        SetAddress(i, 0X00, 0X10);
        WriteDataLength(c, 132);
    }

    // Erase framebuffer
    memset(ShadowFB, c, SHADOW_FB_SIZE);
}

void OLED_Pixel(uint8_t X, uint8_t Y, OLED_Colour Colour)
{
    uint8_t Page;
    uint16_t Add;
    uint8_t LowAddress;
    uint8_t HighAddress;
    uint8_t Mask;
    uint32_t ShadowPos = 0;

    if (X > OLED_DISPLAY_WIDTH)
        return;

    if (Y > OLED_DISPLAY_HEIGHT)
        return;

    // Page address
         if(Y < 8)  Page = 0xB0;
    else if(Y < 16) Page = 0xB1;
    else if(Y < 24) Page = 0xB2;
    else if(Y < 32) Page = 0xB3;
    else if(Y < 40) Page = 0xB4;
    else if(Y < 48) Page = 0xB5;
    else if(Y < 56) Page = 0xB6;
    else            Page = 0xB7;

    Add = X + X_OFFSET;
    LowAddress = 0X0F & Add;        // Low address
    HighAddress = 0X10 | (Add >> 4);// High address

    // Calculate mask from rows basically do a y%8 and remainder is bit position
    Add = Y>>3;         // Divide by 8
    Add <<= 3;          // Multiply by 8
    Add = Y - Add;      // Calculate bit position
    Mask = 1 << Add;    // Left shift 1 by bit position

    // Set the address (sets the page, lower and higher column address pointers)
    taskENTER_CRITICAL();
    SetAddress(Page, LowAddress, HighAddress);
    taskEXIT_CRITICAL();

    ShadowPos = (Page-0xB0) * OLED_DISPLAY_WIDTH + X;

    if(Colour > 0) {
        ShadowFB[ShadowPos] |= Mask;
    } else {
        ShadowFB[ShadowPos] &= ~Mask;
    }

    taskENTER_CRITICAL();
    WriteData(ShadowFB[ShadowPos]);
    taskEXIT_CRITICAL();
}


uint8_t OLED_Char(uint8_t X, uint8_t Y, uint8_t Character, OLED_Colour Forground, OLED_Colour Background)
{
    unsigned char Data = 0;
    unsigned char i = 0, j = 0;

    if ((X >= (OLED_DISPLAY_WIDTH - 8)) || (Y >= (OLED_DISPLAY_HEIGHT - 8)))
        return 0;

    // Unknown character will be set to blank
    if((Character < 0x20) || (Character > 0x7f))
        Character = 0x20;

    Character -= 0x20;
    for (i = 0; i < 8; i++)
    {
        Data = Font5x7[Character][i];
        for(j = 0; j < 6; j++)
        {
            if((Data & Font_Mask[j]) == 0) {
            	//taskENTER_CRITICAL();
                OLED_Pixel(X, Y, Background);
                //taskEXIT_CRITICAL();
            } else {
            	//taskENTER_CRITICAL();
                OLED_Pixel(X, Y, Forground);
               // taskEXIT_CRITICAL();
            }
            X++;
        }
        Y++;
        X -= 6;
    }
    return 1;
}

void OLED_String(uint8_t X, uint8_t Y, uint8_t *String, OLED_Colour Forground, OLED_Colour Background)
{
    while(1)
    {
        if ((*String)=='\0') return;
        if (OLED_Char(X, Y, *String++, Forground, Background) == 0) return;
        X += 6;
    }
}
