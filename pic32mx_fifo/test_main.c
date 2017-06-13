/* Test code for SPI#1 slave SDI interface to RPi  and SPI#2 master interface to ADC
 *
 * External Slave connections
 * Slave #1 select pin: B0 pin 4, B1 pin 5, B3 pin 7
 * Slave #2 select pin: A3
 *
 * Slave1 Inputs structure INBITS
 * SD card eject button is bit 0 'eject'
 * SD card detect switch is bit 5 'card_detect'
 *
 * Slave1 outputs structure OUTBITS
 * SD card eject lamp is bit 2 'eject_led'
 */

// PIC32MX250F128B Configuration Bit Settings

#include <p32xxxx.h>

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multipliere (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2, 48mhz)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = EC             // Primary Oscillator Configuration 
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin
#pragma config FPBDIV = DIV_4           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Enabled)
//#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF              // JTAG 
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#include "mx_test.h"

#define CONFIG          (CN_ON | CN_IDLE_CON)
#define PINS            (CN15_ENABLE)
#define PULLUPS         (CN15_PULLUP_ENABLE)
#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)
#define SYS_FREQ         (50000000L)



/* 
 * File:   test_main.c
 * Author: root
 *
 * Created on September 22, 2017, 5:52 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mx_test_defs.h"
#include "mx_test_types.h"

/*
 * ISR data
 */
volatile struct V_data V;

/*
 * SPI service request interrupt
 */
void __ISR(_EXTERNAL_1_VECTOR, IPL2AUTO) External_Interrupt_1(void)
{
	V.spi_flag = HIGH;
	V.spi_flag0++;
	mINT1ClearIntFlag();
}

/*
 * SPI service request interrupt
 */
void __ISR(_EXTERNAL_2_VECTOR, IPL2AUTO) External_Interrupt_2(void)
{
	V.spi_flag = HIGH;
	V.spi_flag1++;
	mINT2ClearIntFlag();
}

/*
 *  B3 is the slave (de)select bit for the address decoder 74HC138
 *  B0|B1 are the address bits for the slave [0..2] for 4 possible devices on the bus
 */
void ps_deselect(void)
{
	mPORTBSetBits(BIT_3); // deselect
	mPORTBSetBits(BIT_0 | BIT_1); // set address bits high for 7
}

void ps_select(int ps_id)
{
	mPORTASetBits(BIT_3); // deselect move cs signal out of the device range
	mPORTBSetBits(BIT_0 | BIT_1); // set address
	switch (ps_id) {
	case 0:
		mPORTBClearBits(BIT_0 | BIT_1); // set address
		break;
	case 1:
		mPORTBClearBits(BIT_1); // set address
		mPORTBSetBits(BIT_0); // set address
		break;
	case 2:
		mPORTBClearBits(BIT_0); // set address
		mPORTBSetBits(BIT_1); // set address
		break;
	case 3:
		mPORTBSetBits(BIT_0 | BIT_1); // set address
		break;
	default:
		return;
	}
	mPORTBClearBits(BIT_3); // clear enable bit to move cs bit into the device range
}

void SpiInitDevice1(SpiChannel chn, int speed)
{
	SpiOpenFlags oFlags = SPI_OPEN_MODE8 | SPI_OPEN_SLVEN | SPI_OPEN_SSEN | SPI_OPEN_CKE_REV; // SPI open mode
	// Open SPI module, use SPI channel , use flags set above
	SpiChnOpen(chn, oFlags, speed);
}

void SpiInitDevice2(SpiChannel chn, int speed)
{
	SpiOpenFlags oFlags = SPI_OPEN_MODE8 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV; // SPI open mode
	// Open SPI module, use SPI channel , use flags set above, Divide Fpb by X
	SpiChnOpen(chn, oFlags, speed);
}

void init_spi_ports(void)
{
	SpiInitDevice1(SPI_CHANNEL1, 1); // Initialize the SPI channel 1 as slave.

	SpiInitDevice2(SPI_CHANNEL2, 2); // Initialize the SPI channel 2 as master, 2.083 mhz SCK
	ps_select(0);
}

int main(void)
{
	SDEV_TYPE1 S1 = {0};

	// Init remote device data
	S1.obits.out_byte = 0xff;

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//STEP 1. Configure cache, wait states and peripheral bus clock
	// Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
	srand(ReadCoreTimer()); // Seed the pseudo random generator

	// SCLK1 pin 25 - SD pin 5
	PORTSetPinsDigitalIn(IOPORT_B, BIT_14); // Clock input
	// SCLK2 pin 26 - PIC pin 18
	PORTSetPinsDigitalOut(IOPORT_B, BIT_15); // Clock output

	PPSInput(2, SDI1, RPB8); //Assign SDI1 to pin 17 - SD pin 7
	PORTSetPinsDigitalIn(IOPORT_B, BIT_8); // Input for SDI1
	PPSInput(3, SDI2, RPB13); //Assign SDI2 to pin 24
	PORTSetPinsDigitalIn(IOPORT_B, BIT_13); // Input for SDI2
	PPSOutput(2, RPB11, SDO1); // Set 22 pin as output for SDO1 - SD pin 2
	PPSOutput(2, RPB5, SDO2); // Set 14 pin as output for SDO2
	PORTSetPinsDigitalOut(IOPORT_B, BIT_2); // CS line pin 6 - SD pin 1
	mPORTBSetBits(BIT_2); // deselect SDCARD
	PORTSetPinsDigitalOut(IOPORT_B, BIT_3); // select pin enable for SPI slaves bit 2 to 74hc138
	mPORTBSetBits(BIT_3); // deselect Slave1
	PORTSetPinsDigitalIn(IOPORT_A, BIT_0); // for SPI 1 slave select input
	PPSInput(1, SS1, RPA0); // for SPI 1 slave select input
	PORTSetPinsDigitalIn(IOPORT_A, BIT_4); // for SPI 2 slave interrupt input #1
	PPSInput(4, INT1, RPA3); // EXT Interrupt #1 Port A3 chip pin 10
	ConfigINT1(EXT_INT_PRI_2 | FALLING_EDGE_INT | EXT_INT_ENABLE);
	SetSubPriorityINT1(EXT_INT_SUB_PRI_3);
	ConfigINT2(EXT_INT_PRI_2 | FALLING_EDGE_INT | EXT_INT_ENABLE);
	SetSubPriorityINT2(EXT_INT_SUB_PRI_3);
	mINT1ClearIntFlag(); // clear before enable
	mINT2ClearIntFlag(); // clear before enable
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Diag Led pins
	PORTSetPinsDigitalOut(IOPORT_A, BIT_1); // LED
	PORTSetPinsDigitalOut(IOPORT_B, BIT_0 | BIT_1); // programming inputs /device select outputs address for SPI slaves bits [0..1] 74hc138

	// Enable multi-vectored interrupts
	//	INTEnableSystemMultiVectoredInt();

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// STEP 3. initialize the port pin states = outputs low

	mPORTBClearBits(BIT_0 | BIT_1);

	init_spi_ports();

	while (1) { // loop and move data
		mPORTAToggleBits(BIT_1);
	}
}

