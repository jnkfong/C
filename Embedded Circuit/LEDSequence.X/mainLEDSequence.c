//TEJ4M cTut4 LEDSequence.c
/*********************************************************************
Module:
 main()

Author - James Fong
Date - April 7, 2014

Explain Operation of Program here:
 * Review decision making structures in ?C? using SPST push buttons

Hardware Notes:
 * PIC used
     PIC24FV32KA302 operating at 8MHz
 * I/O ports used and hardware attached
     * Output:
           * RB0-RB9 connected to red LED
    * Input:
           * RA0-RA1 connected to SPST switches

********************************************************************/

/*******************************************************************
	Include Files
********************************************************************/

#include "p24FV32KA302.h"
#include "configBits.h"
#include "delay.h"

/*******************************************************************
	Symbolic Constants used by main()
********************************************************************/

#define SHORT_DELAY 500
#define MID_DELAY 1000
#define LONG_DELAY 2000

/*******************************************************************
	Local Function Prototypes
********************************************************************/

    void initTImer (void);
    void delay (unsigned long milli);
    void shiftBitL(void);
    void shiftBitR(void);
    void switchSigBit(void);
    void countBit(void);


/*******************************************************************
	Global Variable Declarations
********************************************************************/

// variables declared go here, before main ()
unsigned int i;

/*******************************************************************
	main() function
********************************************************************/

int main (void)
{
    //Initialize time for delay action
    initTimer();
    //Initialize all RB values to be outputs
    TRISB=0;
    //Set all RB output to 0
    LATB=0;
    //Initialize all RA values to be digital and inputs
    TRISA=0xFF;
    ANSA=0;

    while (1)
    {
        if (PORTAbits.RA0==0 && PORTAbits.RA1==1)
        {
            shiftBitL();
        }

        else if (PORTAbits.RA0==1 && PORTAbits.RA1==0)
        {
            shiftBitR();
        }
       
        else if (PORTAbits.RA0==0 && PORTAbits.RA1==0)
        {
            switchSigBit();
        }
        else
        {
            delay(SHORT_DELAY);
            LATB++;
        }
    }

    


} // end mainTemplate.c

/*******************************************************************
	List of Functions
********************************************************************/
//LEDs shift to left quickly
void shiftBitL(void)
{
    LATB= 0x200;
        for(i=0; i<10; i++)
        {
            delay(SHORT_DELAY);
            LATB= LATB>>1;
        }
}
//LEDs shift to right slowly
void shiftBitR (void)
{
    LATB= 0x001;
         for(i=0; i<10; i++)
        {
            delay(MID_DELAY);
            LATB= LATB<<1;
        }
}
//LSb and MSb flash every 2 second in between
void switchSigBit(void)
{
    LATB= 0x1F;
        delay(LONG_DELAY);
        LATB=0xFE0;
        delay(LONG_DELAY);
}

