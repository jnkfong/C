//mainHello.c
/*********************************************************************
Module:
 main()

Author - James Fong
Date - April 6 2014

Explain Operation of Program here:
 * Learn how to write and use arrays in 'C'
 * This program is used to display the word "HELLO" using 10 LEDs.

Hardware Notes:
 * PIC used
     * PIC24FV32KA302 operating at 8MHz
 * I/O ports used and hardware attached
     * Output:
           * RB0 to RB9 connected to red LED
           * RA0 & RA1 connected to SPST switches

********************************************************************/

/*******************************************************************
	Include Files
********************************************************************/

#include "p24FV32KA302.h"
#include "configBits.h"
#include "delay.h"
#include "global.h"

/*******************************************************************
	Symbolic Constants used by main()
********************************************************************/

#define SHORT_DELAY 50
#define LONG_DELAY 550

/*******************************************************************
	Local Function Prototypes
********************************************************************/

//leave blank for now

/*******************************************************************
	Global Variable Declarations
********************************************************************/

unsigned int i;


/*******************************************************************
	main() function
********************************************************************/

int main (void)
{
    //InitializeTimer to be used for delay function
    initTimer();

    //teach all of PORTB to be outputs;
    TRISB=0;

    /*cycle through the array which when moving hand from left to right
     * generates the word "HELLO" in a streak of red */
    for (i=0; i<30; i++)
    {
        //turn on LED
        PORTB = bitmap[i];
        delay(SHORT_DELAY);
    }

        //leave on all LEDs
         PORTB=0xFF;
         delay(LONG_DELAY);

         //turn off all LEDs
         PORTB=0;

    while (1); //reset

} // end mainTemplate.c

/*******************************************************************
	List of Functions
********************************************************************/

//leave blank for now

