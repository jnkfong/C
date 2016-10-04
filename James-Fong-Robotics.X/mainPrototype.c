/*******************************************************************************
Module:
  main.c

Summary:
 Robot Prototype code

Copyright Notice:
  (Dennis Cecic (d.cecic@ieee.org)

Original Write:
  October 05, 2011

Remarks:
    This code is used by the student to test the circuit construction.
    The project initializes the ADC, Timer and PWM peripherals to drive both
    GM8/9 Motors using a PWM signal. The PWM signal is controlled by the
    Potentiometer connected to RA0/AN0. The 3 indicator LEDs will flash when
    each opto sensor detects a line. Note that the thresholds will require
    some adjustment by the student.

    The main loop is formatted as a structured control loop for ease of
    understanding and modification as the robot control code becomes more
    complex.

    *******************************************************************************/

/*** Include Files ************************************************************/

#include "p24fv32ka302.h"
#include "configBits.h"

/*** Symbolic Constants used by main() ****************************************/

// Optosensor Trip Thresholds

//left opto sensor varies from 123 (W) to 45 (B)
    #define LEFT_THRESHOLD      85

//right opto sensor varies from 45 (W) to 18 (B)
    #define RIGHT_THRESHOLD     50

/*** Local Function Prototypes ************************************************/

void initialize(void);	// initialize peripheral SFRs and global variables
void get_Inputs(void);  // Read Peripheral, PIN and/or variable states
void decide(void);      // Make decisions on the inputs to manipulate global variables
void do_Outputs(void);  // Output data onto the pins or into the peripherals
void timing(void);      // Determines how fast the control loop executes

void delayMs(unsigned int ms);  // Provide a delay of up to 65535 mS


/*** Global Variable Declarations *********************************************/

unsigned int potValue;   // RA0/AN0: Raw Potentiometer setting
unsigned int leftOpto;   // RB4/AN15: Left Opto Sensor Signal
unsigned int rightOpto;  // RA2/AN13: Right Opto Sensor Signal

/*** main() Function **********************************************************/

int main(void)
{

	initialize();

	//infinite loop
	while(1)
	{
		get_Inputs();
		decide();
		do_Outputs();
		timing();
	}

}

/*******************************************************************************
 * Function:        void initialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the microcontroller, the peripherals
 *                  used in the application and any global variables
 *                  used by multiple functions.
 *
 * Note:            None
 ******************************************************************************/

void initialize(void)
{
  // initialize Timer1 for main loop delay of 100mS
  // (Fcyc/2 = 4 MHz)

  T1CONbits.TCS = 0;        // Set T1 Clk Source = Fosc/2
  T1CONbits.TCKPS = 3;      // Set T1 Clk Pre-scale = 1:256
                            // (TMR1 CLK = 15.625kHz)
  PR1 = 400;               // Set period = ~100mS
  TMR1 = 0;                 // Clear T1 counter
  IFS0bits.T1IF = 0;        // Clear T1 overflow/interrupt flag
  T1CONbits.TON = 1;        // Start T1

  // LED D6 (RB8) - "Right Line Detect"
  LATBbits.LATB8 = 0;      // Initial level to place on pin
  TRISBbits.TRISB8 = 0;    // Make the pin a digital output

    // LED D2 (RB6) - "Left Line Detect"
  LATBbits.LATB6 = 0;      // Initial level to place on pin
  TRISBbits.TRISB6 = 0;    // Make the pin a digital output

  // initialize Potentiometer (Speed Control) Input (RA0/AN0)

  ANSAbits.ANSA0 = 1;     // Make RA0 input Analog
  AD1CON1 = 0x0000;        // SSRC = 0000 clearing SAMP starts conversion
                           // 10-bit ADC mode selected
  AD1CHS = 0x0000;         // Connect RA0/AN0 as CH0 input
  AD1CSSL = 0;
  AD1CON3 = 0x1F02;        // Sample time = 31Tad, Tad = internal (2*Tcy)
  AD1CON2 = 0;

  AD1CON1bits.ADON = 1;    // Turn on ADC

  AD1CON1bits.SAMP = 1;     // Sample potentiometer value
  delayMs(1);               // after 1mS go to conversion
  AD1CON1bits.SAMP = 0;     // Convert potentiometer value
  while(!AD1CON1bits.DONE); // conversion done? (takes 12*Tad)
  potValue = ADC1BUF0;   // yes, then save ADC value

  // initialize I/O and Peripherals for Opto Sensors

  // Left
  ANSBbits.ANSB4 = 1;     // Make RB4/AN15 input Analog
  // Middle
  ANSAbits.ANSA3 = 1;     // Make RA3/AN14 input Analog
  // Right
  ANSAbits.ANSA2 = 1;     // Make RA2/AN13 input Analog


  // Initialize I/O and Peripherals to drive "3-4EN" PWM signal

  //    L293DNE Truth Table
  //    3-4EN   3A  4A  Motor Action
  //        1   0   0   Brake (low side)
  //        1   0   1   Forward
  //        1   1   0   Reverse
  //        1   1   1   Brake (high side)
  //        0   x   x   Coast

  // Initialize "3-4EN" PWM control output (RB10/OC3)

  TRISBbits.TRISB10 = 0;    // Make RB10 digital O/P
  LATBbits.LATB10 = 0;		// initialize the pin voltage level

  // initialize "3A" control output (RB15/AN09)

  TRISBbits.TRISB15 = 0;    // Make RB15 digital O/P
  LATBbits.LATB15 = 0;		// initialize the pin voltage level

  // initialize "4A" control output (RB14/AN10)

  TRISBbits.TRISB14 = 0;    // Make RB14 digital O/P
  LATBbits.LATB14 = 1;		// initialize the pin voltage level

  // Initialize Output Compare 3 (OC3) to drive Motor A PWM signal to "3-4EN"
  // We want to create 61.04Hz PWM frequency @ 1024 bits resolution

  OC3CON1bits.OCM = 0b000;       // Disable the OC module
  OC3R = potValue;   // Write the duty cycle for the 1st PWM pulse
  OC3CON1bits.OCTSEL = 0; // Select Timer 2 as the OC time base
  OC3CON1bits.OCM = 0b110;       // Select the OC mode (Edge PWM)


   // Initialize I/O and Peripherals to drive "1-2EN" PWM signal

  //    L293DNE Truth Table
  //    1-2EN   1A  2A  Motor Action
  //        1   0   0   Brake (low side)
  //        1   0   1   Forward
  //        1   1   0   Reverse
  //        1   1   1   Brake (high side)
  //        0   x   x   Coast

  // Initialize "1-2EN" PWM control output (RB11/OC2)

  TRISBbits.TRISB11 = 0;    // Make RB11 digital O/P
  LATBbits.LATB11 = 0;		// initialize the pin voltage level

  // Initialize "1A" control output (RB13/AN11)

  TRISBbits.TRISB13 = 0;    // Make RB13 digital O/P
  LATBbits.LATB13 = 0;		// initialize the pin voltage level

  // Initialize "2A" control output (RB12/AN12)

  TRISBbits.TRISB12= 0;    // Make RB12 digital O/P
  LATBbits.LATB12 = 1;		// initialize the pin voltage level

  // Initialize Output Compare 2 (OC2) to drive Motor B PWM signal to "1-2EN"
  // We want to create 61.04Hz PWM frequency @ 1024 bits resolution

  OC2CON1bits.OCM = 0b000;       // Disable the OC module
  OC2R = potValue;   // Write the duty cycle for the 1st PWM pulse
  OC2CON1bits.OCTSEL = 0; // Select Timer 2 as the OC time base
  OC2CON1bits.OCM = 0b110;       // Select the OC mode (Edge PWM)

  // Initialize and enable Timer 2 to create a 61.04Hz PWM frequency for both PWM channels

  T2CONbits.TON = 0;            // Disable Timer
  T2CONbits.TCS = 0;            // Select internal instruction clock
  T2CONbits.TGATE = 0;          // Disable Gated Timer Mode
  T2CONbits.TCKPS = 0b10;       // Select 1:64 prescale (57.578kHz)
  TMR2 = 0x00;                  // Clear timer register
  PR2 = 1024;                   // Load the period register

  IFS0bits.T2IF = 0;            // Clear Timer 2 interrupt flag
  T2CONbits.TON = 1;            // Start timer (starts PWMs)

}

/*******************************************************************************
 * Function:        void get_Inputs(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Obtains any input information either on-chip
 *                  (from internal registers, etc...) or off-chip
 *                  (pin voltage levels). Uses this information to modify
 *                  or update special data structures used in the control
 *                  function "decide()"
 *
 * Note:            None
 ******************************************************************************/

void get_Inputs(void)
{

  // Sample/save Opto Sensor Levels

  // Left (RB4/AN15)

  AD1CHS = 0x000F;          // Connect RB4/AN15 as CH0 input
  AD1CON1bits.SAMP = 1;     // Sample potentiometer value
  delayMs(5);               // after 5mS start conversion
  AD1CON1bits.SAMP = 0;     // Convert potentiometer value
  while(!AD1CON1bits.DONE); // conversion done? (takes 12*Tad)
  leftOpto = ADC1BUF0;    // yes, then save ADC value

  // Right (RA2/AN13)

  AD1CHS = 0x000D;          // Connect RA2/AN13 as CH0 input
  AD1CON1bits.SAMP = 1;     // Sample potentiometer value
  delayMs(5);               // after 5mS start conversion
  AD1CON1bits.SAMP = 0;     // Convert potentiometer value
  while(!AD1CON1bits.DONE); // conversion done? (takes 12*Tad)
  rightOpto = ADC1BUF0;   // yes, then save ADC value

  // Sample/save potentiometer connected to RA0/AN0

  AD1CHS = 0x0000;          // Connect RA0/AN0 as CH0 input
  AD1CON1bits.SAMP = 1;     // Sample potentiometer value
  delayMs(1);               // after 1mS start conversion
  AD1CON1bits.SAMP = 0;     // Convert potentiometer value
  while(!AD1CON1bits.DONE); // conversion done? (takes 12*Tad)
  potValue = ADC1BUF0;   // yes, then save ADC value

}

/*******************************************************************************
 * Function:        void decide(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Makes decisions based on the input information
 *                  gathered in get_Inputs() function to manipulate
 *                  global output control variables.
 *
 *                  Can implement functional or state-machine based control
 *
 * Note:            None
 ******************************************************************************/

void decide(void)
{

  if(leftOpto < LEFT_THRESHOLD)
  {
      // Left opto detects line
      // Turn on Left LED (RB6)
      LATBbits.LATB6 = 1;

  }
  else
  {
      // Left opto does not detect the line
      // Turn off Left LED (RB6)
      LATBbits.LATB6 = 0;
  }


  if(rightOpto < RIGHT_THRESHOLD)
  {
      // Right opto detects line
      // Turn on Right LED (RB8)
      LATBbits.LATB8 = 1;
  }
  else
  {
      // Right opto does not detect the line
      // Turn off Right LED (RB8)
      LATBbits.LATB8 = 0;
  }

}

/*******************************************************************************
 * Function:        void do_Outputs(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Based on the decisions made in the previous function,
 *                  this function outputs data onto the pins of the
 *                  microcontroller or to registers within the device.
 *
 * Note:            None
 ******************************************************************************/

void do_Outputs(void)
{
    // Update PWM duty cycle registers (motor speed/torque) for both motors
    OC3R = potValue;
    OC2R = potValue;
}

/*******************************************************************************
 * Function:        void timing(void)
 *
 * PreCondition:    Timer1 initialized to provide the appropriate delay
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    No task in the main loop can block the CPU
 *                  Requires cooperative multi-tasking design
 *
 * Overview:        This function determines how fast the control loop runs
 *
 * Note:            None
 ******************************************************************************/

void timing(void)
{
   // wait for T1 overflow
   while(!IFS0bits.T1IF);

   // acknowledge and reset the flag
   IFS0bits.T1IF = 0;
}

/*******************************************************************************
 * Function:        void delayMs(unsigned int ms)
 *
 * PreCondition:    Requires Tcyc=250nS (8 MHz Fosc)
 *
 * Input:           delay in milliseconds (1-65535)
 *
 * Output:          None
 *
 * Side Effects:    Blocking Delay (CPU is blocked from doing other tasks)
 *                  Non-portable (Uses PIC24 Assembly language instructions)
 *
 * Overview:        This function implements an in-line delay of up to 65535ms
 *
 * Note:            None
 ******************************************************************************/

void delayMs(unsigned int ms)
{
   while(ms--)
   {
       asm("repeat #4000"); // 4000 instruction cycles @ 250nS ea. = 1mS
       asm("nop");          // instruction to be repeated 4000x
   }

}

// end main.c

