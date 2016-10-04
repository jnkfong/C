//TEJ4M cTut1 main.c
/*********************************************************************
Module:
 main()

Author - James Fong
Date - April 11 2014

Explain Operation of Program here:

Hardware Notes:
 * PIC used
     * PIC24FV32KA302 operating at 8MHz
 * I/O ports used and hardware attached
     * Output:
           * RB0 to RB9 connected to red LED
     * Input:
           * B0(RA0) connected as input to SPST
           * B1(RA1) connected as input to SPST

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
#define LONG_DELAY 1250

/*******************************************************************
	Local Function Prototypes
********************************************************************/

    void initTImer (void);
    void delay (unsigned long milli);

/*******************************************************************
	Global Variable Declarations
********************************************************************/

unsigned int i; //Variable used as counter in loop

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
        //If button 0 is pressed only
        if (PORTAbits.RA0==0 && PORTAbits.RA1==1)
        {

        }
        //If button 1 was pressed only
        else if (PORTAbits.RA0==1 && PORTAbits.RA1==0)
        {

        }
        //If both buttons are pressed
        else if (PORTAbits.RA0==0 && PORTAbits.RA1==0)
        {

        }
        //If no buttons are pressed
        else
        {

        }
    }

} // end mainTemplate.c

/*******************************************************************
	List of Functions
********************************************************************/



