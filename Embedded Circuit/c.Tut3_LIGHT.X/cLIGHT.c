    //TEJ4M cTut3 LIGHT.c
    /*********************************************************************
    Module:
     main()

    Author - James Fong
    Date - April 2, 2014

    Explain Operation of Program here:
    This program is designed to learned about how the inputs and outputs
     are used for the circuit

    Hardware Notes:
     * PIC used
         * eg. PIC24FV32KA302 operating at 8MHz
     * I/O ports used and hardware attached
         * eg. Output:
               * RB0 - RB9 connected to Green LED

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
    #define SHORTEST_DELAY 50
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
        //Initializes the time in the program
        initTimer();
        //Initialize that all RA values are inputs
        TRISA=0xFF;
        ANSA=0;
        //Initialize that all RB values are ouputs
        TRISB=0;
        //Set all RB values to 0
        LATB=0;

        while (1)
        {
            while(PORTAbits.RA1==0)
            {
                helloLight();

            }

          multiFlash();
        }


    } // end cLIGHT.c

    /*******************************************************************
            List of Functions
    ********************************************************************/
    multiFlash()
    {
                //Flash even LEDS
        int x = 0;
        int array[]={0x2AA,0x155};
        for(x = 0; x<2; x++)
        {
            LATB= array[x];
            delay(SHORT_DELAY);
        }


    }
    helloLight()
            {
                //H symbol
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x030;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                delay(LONG_DELAY);
                //E symbol
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x333;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x333;
                delay(LONG_DELAY);
                LATB=0x000;
                delay(LONG_DELAY);
                //L symbol
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x300;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x300;
                delay(LONG_DELAY);
                LATB=0x000;
                delay(LONG_DELAY);
                 //L symbol
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x300;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x300;
                delay(LONG_DELAY);
                LATB=0x000;
                delay(LONG_DELAY);
                //O symbol
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x303;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0x303;
                delay(LONG_DELAY);
                LATB=0x000;
                LATB=0xFFF;
                delay(LONG_DELAY);
                LATB=0x000;
                delay(LONG_DELAY);


        }


