//global.h
/*******************************************************************************
                    Global Variable and Array Declarations
 ******************************************************************************/

//declare variable used in loops
unsigned int i;

//generate a bitmap array which spells out "HELLO" ysing all 8 LEDs

unsigned char bitmap[30] = { 0b11111111, //H
                             0b00001000,
                             0b00001000,
                             0b11111111,
                             0b00000000,
                             0b00000000,
                             0b11111111, //E
                             0b10001001,
                             0b10001001,
                             0b10001001,
                             0b00000000,
                             0b00000000,
                             0b11111111, //L
                             0b10000000,
                             0b10000000,
                             0b10000000,
                             0b00000000,
                             0b00000000,
                             0b11111111, //L
                             0b10000000,
                             0b10000000,
                             0b10000000,
                             0b00000000,
                             0b00000000,
                             0b11111111, //O
                             0b10000001,
                             0b10000001,
                             0b11111111,
                             0b11111111,
                             0b11111111,
                            };