//TEJ4M cTut2 cOperators.c
/*********************************************************************
Module:
 main()

Author - James Fong
Date - March 30, 2014

Explain Operation of Program here:
This program is designed to learned about the different oeprators in the
 c language through different operation questions. 

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

/*******************************************************************
	Symbolic Constants used by main()
********************************************************************/

//leave blank for now

/*******************************************************************
	Local Function Prototypes
********************************************************************/

//leave blank for now

/*******************************************************************
	Global Variable Declarations
********************************************************************/

// variables declared go here, before main ()
int op1, op2, result;

/*******************************************************************
	main() function
********************************************************************/

int main (void)
{

//initialize variables
 op1=4, op2=9, result=0;

//pg 1..

 result = op1 + op2 ; // result= 13

 result = op1 + op2 * op1 - op2; // result= 31
 
 result = op2 % op1; // result= 1

 result = op1 % op2; // result= 4

 result = op2 * 0x0B; // result= 99

 result = 0b1110/0x0F; // result= 1

 result = op2/op1; // result= 2

 result = 0xFFFF % 16; // result= 15

 result = op1/4 + op2/2 % op1 + 6/op2; // result= 1

 result = -op2 - 20 % 7 + 0x10; // result= 1

 result = op1 * 0b1111 % 0x07 ; // result= 4

 result *= op1; // result= 16

 result = op2 != op1 ; // result= 1

 result = op1 < op2; // result= 1

 result = (op2 <= op1) || (op1 > op2); // result= 0

 result = (op2 >= op2) < (op1 < op2) ; // result= 0

 result = (op2 < op1) < !(op1 == op2); // result= 1

 result = (op2 >= 5) && (op1 * op2 < 10); // result= 0

 result = (32 << 2) % (64 >> 1) ; // result= 0 **

 result = (op1 != op1) && (op2 > op2); // result= 0 **

 result = 0xFF & 0x0F; // result= 15 **
 
 result = 0x0F | 0xF0; // result= 255 **

 result = 0xFF ^ 0xFF; // result= 0 **

 result = op2 ^ 0x000B; // result= 2 **

    while (1); //loop forever
                //this line isn't always necessary

} // end mainTemplate.c

/*******************************************************************
	List of Functions
********************************************************************/

//leave blank for now

