//TEJ4M cPotADC.c
/*********************************************************************
Module:
 main()

Author - James Fong
Date - April 6 2014

Explain Operation of Program here:
 * Learn to use the sample and convert analog signals using the ADC (Analog
 to Digital Convert) on the PIC24
 * Learn the basiccs of how to write and use a function in 'C'
 * Use a different delay function

Hardware Notes:
 * PIC used
     * PIC24FV32KA302 operating at 8MHz
 * I/O ports used and hardware attached
     * Output:
           * RB0 to RB9 connected to red LED
     * Input:
           * RA0/AN0 connected to 10K pot

********************************************************************/

/*******************************************************************
	Include Files
********************************************************************/

#include "p24FV32KA302.h"
#include "configBits.h"


/*******************************************************************
	Symbolic Constants used by main()
********************************************************************/

//leave blank for now

/*******************************************************************
	Local Function Prototypes
********************************************************************/

void initialize(void);
void getInput(void);
void delayMs(unsigned int ms);

/*******************************************************************
	Global Variable Declarations
********************************************************************/

// variables declared go here, before main ()
unsigned int potValue;

/*******************************************************************
	main() function
********************************************************************/

int main(void)
    {
        //Initialize all peripherals
        initialize();

        //infinite loop
        while(1) 
        {
            //refresh MSB ADC value from pot
            // and assign MSB of 10-bit ADC Buffer to 10 LEDs
                getInput();
                LATB= potValue;
        }//end while
    }//end mainLEDSequencePot.c

/*******************************************************************
	List of Functions
********************************************************************/
/*******************************************************************************
* Function: void initialize(void)
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
* Side Effects: None
*
* Overview: Initializes the microcontroller, the peripherals
* used in the application and any global variables
* used by multiple functions.
*
* Note: None
******************************************************************************/
void initialize(void)
{
    // PORTB all outputs
    TRISB=0;
    // initialize PORTB
    LATB=0;

    // Initialize Potentiometer Input (RA0/AN0)
    ANSAbits.ANSA0 = 1; // Make AN0 input Analog
    AD1CON1 = 0x0000; // SSRC = 0000 clearing SAMP starts conversion
    // 10-bit ADC mode selected
    AD1CHS = 0x0000; // Connect RA0/AN0 as an analog channel input
    AD1CSSL = 0;
    AD1CON3 = 0x1F02; // Sample time = 31Tad, Tad = internal (2*Tcy)
    AD1CON2 = 0;
    AD1CON1bits.ADON = 1; // Turn on ADC
}
/*******************************************************************************
* Function: void getInput(void)
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
* Side Effects: None
*
* Overview: Gets the binary value that the ADC generates and stores
* in the ADC Buffer Register (ADC1BUF0) after it samples
* and converts the analog signal coming from the pot
*
* Note: None
******************************************************************************/
void getInput(void)
{
    // Sample/save potentiometer connected to RA0/AN0
    AD1CHS = 0x0000; // Connect RA0/AN0 as CH0 input
    AD1CON1bits.SAMP = 1; // Sample potentiometer value
    delayMs(1); // after 1mS start conversion
    AD1CON1bits.SAMP = 0; // Convert potentiometer value
    while(!AD1CON1bits.DONE); // conversion done? (takes 12*Tad)
    potValue = ADC1BUF0; // yes, then save ADC value to variable
}
/*******************************************************************************
* Function: void delayMs(unsigned int ms)
*
* PreCondition: Requires Tcyc=250nS (8 MHz Fosc)
*
* Input: delay in milliseconds (1-65535)
*
* Output: None
*
* Side Effects: Blocking Delay (CPU is blocked from doing other tasks)
* Non-portable (Uses PIC24 Assembly language instructions)
*
* Overview: This function implements an in-line delay of up to 65535ms
*
* Note: None
******************************************************************************/
void delayMs(unsigned int ms)
{
    while(ms--)
    {
        asm("repeat #4000"); // 4000 instruction cycles @ 250nS ea. = 1mS
        asm("nop"); // instruction to be repeated 4000x
    }
}
//end functions

