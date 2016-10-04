//mainHello.c
/*********************************************************************
Module:
 main()

Author - put your name here
Date - put the date here

Explain Operation of Program here:
eg. this program is designed as a template file
to be used for other 'C' source code files

Hardware Notes:
 * PIC used
     * eg. PIC24FV32KA302 operating at 8MHz
 * I/O ports used and hardware attached
     * eg. Output:
           * RB6 connected to green LED

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

#define SHORT_DELAY 500
#define LONG_DELAY 1250

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

