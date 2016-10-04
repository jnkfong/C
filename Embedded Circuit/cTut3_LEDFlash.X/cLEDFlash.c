    //TEJ4M cTut3_LEDFlash
    /*********************************************************************
    Module:
     main()

    Author - James Fong
    Date - April 2, 2014

    Explain Operation of Program here:
    This program is designed to learned about how the inputs and outputs
     are used for the circuit through simulation of LEDS

    Hardware Notes:
     * PIC used
         * eg. PIC24FV32KA302 operating at 8MHz
     * I/O ports used and hardware attached
         * eg. Output:
               * RB0 - RB9 connected to Red LED

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
    #define SHORT_DELAY 100
    #define LONG_DELAY 1250

    /*******************************************************************
            Local Function Prototypes
    ********************************************************************/

    void initTImer (void);
    void delay (unsigned long milli);

    /*******************************************************************
            Global Variable Declarations
    ********************************************************************/

    //declare variable used in loops
    unsigned int i;

    /*******************************************************************
            main() function
    ********************************************************************/

    int main (void)
    {
        //Initializes Timer to be used for the delay function
        initTimer();
        
        //Teach all PORTB to be outputs
        TRISB=0;
        //Initialize all of PORTB
        LATB=0;

        //Continuously loop
        while (1)
            {
              
         
        }


    } // end mainLEDFlash.c

    /*******************************************************************
            List of Functions
    ********************************************************************/
   
    sample ()
    {
        //quickly flash LSB LED 3 times
                for(i=0;i<3; i++)
                {
                    LATBbits.LATB0=1;
                    delay(SHORT_DELAY);
                    LATBbits.LATB0=0;
                    delay(SHORT_DELAY);
                }
               //quickly flash MSb LED 6 times
                for(i=0;i<6; i++)
                {
                    LATBbits.LATB7=LATBbits.LATB7^1;
                    delay(SHORT_DELAY);
                }
                //slowly flash byte LED 2 times
                for(i=0;i<2; i++)
                {
                    LATB=0xFF;
                    delay(LONG_DELAY);
                    LATB=0x00;
                    delay(LONG_DELAY);
                }
    }
    //Flash all LEDs forever every 2.5 seconds
    answerA()
    {
        //flash LED
        LATB=0xFF;
        delay(SHORT_DELAY);
        //Wait for 2.5 seconds before next flash
        LATB=0x00;
        delay(2500);

    }
    //Toggle 4 LSb and 4 MSb 6 times
    answerB()
    {
        for(i=0; i<6; i++)
        {
            LATB=0x000;
            LATB=0x00F;
            delay(SHORT_DELAY);
            LATB=0x000;
            LATB=0x3C0;
            delay(SHORT_DELAY);
        }
    }
    //Loop forever 2 MSb and middle 2 LEDs
    answerC()
    {
        while (1)
        {
            for(i=0; i<10; i++)
            {
                LATB=0xF00;
                delay(50);
                LATB=0x000;
                delay(50);
            }
           for(i=0; i<4; i++)
            {
                LATB=0x030;
                delay(20);
                LATB=0x000;
                delay(20);
            }
        }
    }


