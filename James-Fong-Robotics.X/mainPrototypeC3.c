/*******************************************************************************
Module:
  mainProtypeC3.c

Author - James Fong 
Date - May 21 2014

Explain Operation of Program here:
        In this challenge, the robot must follow a black line path that ends
 * with a straight horizontal line and a aluminum pail beyond it. When the robot
 * reaches this line, it must pick up the pail at the centre of the course
 * and determine the temperature of the pail either being hot, warm or cold.
 * Once determined, the robot must place the oail into the associated base. After
 * the pail has been placed, the robot must return to its home base and fit within
 * the base's limits. 
Hardware Notes:
 * PIC used
     * PIC24FV32KA302 operating at 8MHz
 * I/O ports used and hardware attached
     * Output:
           * RB6 connected as output to left sensor LED
           * RB7 connected as output to servo motor
           * RB8 connected as output to right sensor LED
           * RB11-RB13 connected as output to control right motor wheels
           * RB10, RB14-RB15 connected as output to control left motor wheels
     * Input:
           * RA0 connected as input to potentiometer
           * RA1 connected as input to thermistor
           * RA2 connected as input to right opto sensor
           * RB4 connected as input to left opto sensor

Certain parts of the code contain codes from Dennis Cecic:
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

//left opto sensor varies from 58 (W) to 16 (B)
    #define LEFT_THRESHOLD      35

//right opto sensor varies from 68 (W) to 18 (B)
    #define RIGHT_THRESHOLD     32

//temperature classified as hot 
    #define HOT_THRESHOLD 327

//temperature classified as cold 
    #define COLD_THRESHOLD 318

//determine which base the pail has been placed on
    #define COLD 0
    #define WARM 1
    #define HOT 2

/*** Local Function Prototypes ************************************************/

void initialize(void);	// initialize peripheral SFRs and global variables
void get_Inputs(void);  // Read Peripheral, PIN and/or variable states
void decide(void);      // Make decisions on the inputs to manipulate global variables
void do_Outputs(void);  // Output data onto the pins or into the peripherals
void timing(void);      // Determines how fast the control loop executes

void delayMs(unsigned int ms);  // Provide a delay of up to 65535 mS

void leftReverse (void);  //reverse robot to the left
void leftForward (void);  //move robot forward to the left
void rightReverse (void); //reverse robot to the right
void rightForward (void); //move robot forward to the right

void right (void);    //turn the robot right
void left (void);     //turn the robot left
void forward (void);  //move the robot forward
void reverse (void);  //move the robot backward
void turn (void);     //turn the robot around
void turnB (void);    //turn the robot around w/ reverse
void brakes (void);   //stop the robot from moving

void liftArm(void);   //lift robot arm
void lowerArm(void);  //lower robot arm

/*** Global Variable Declarations *********************************************/

unsigned int potValue;   // RA0/AN0: Raw Potentiometer setting
unsigned int tempValue;  // RA1/AN1: Raw thermistor setting
unsigned int leftOpto;   // RB4/AN15: Left Opto Sensor Signal
unsigned int rightOpto;  // RA2/AN13: Right Opto Sensor Signal
unsigned int blkDetect=0; //count number of times both sensors detect black
unsigned int tempPlace;  //check which location pail is placed

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

     /***********************SERVO CONTROL*************************************/

  // Initialize Servo PWM control output (RB7/OC1)

  TRISBbits.TRISB7 = 0;    // Make RB10 digital O/P
  LATBbits.LATB7 = 0;      // initialize the pin voltage level

  // Initialize Output Compare 1 (OC1) to drive Servo motor using PWM signal
  // We want to create 50Hz PWM frequency (20ms) @ 10,000 bits resolution
  // The servo moves 180 degrees. The pulse width ranges from .75ms to 2.25ms.
  // Three Servo positions are:
        // 180 degrees: 3.75% duty cycle (.75ms) * 10,000 = 375
        // 90 degrees: 7.5% duty cycle (1.5ms) * 10,000 = 750
        // 0 degrees: 11.25% duty cycle (2.25ms) * 10,000 = 1125

  OC1CON1bits.OCM = 0b000;       // Disable the OC module
  OC1R = 650;           // Write the duty cycle for the 1st PWM pulse at 0 degrees 900 600
  OC1CON1bits.OCTSEL = 0b001;   // Select Timer 3 as the OC time base
  OC1CON1bits.OCM = 0b110;       // Select the OC mode (Edge PWM)
  OC1CON2bits.SYNCSEL = 0b01101; // Synchronize OC1 to Timer3

  // Initialize and enable Timer 3 to create a 50Hz PWM frequency for PWM channel

  T3CONbits.TON = 0;            // Disable Timer
  T3CONbits.TCS = 0;            // Select internal instruction clock
  T3CONbits.TGATE = 0;          // Disable Gated Timer Mode
  T3CONbits.TCKPS = 0b01;       // Select 1:64 prescale (57.578kHz)
  TMR3 = 0x00;                  // Clear timer register
  PR3 = 10000;                   // Load the period register needed for
                                // for 50Hz (20ms) control signal
  IFS0bits.T3IF = 0;            // Clear Timer 3 interrupt flag
  T3CONbits.TON = 1;            // Start timer (starts PWMs)

  /***********************THERMISTOR CONTROL *********************************/

  ANSAbits.ANSA1 = 1;     // Make RA1 input Analog
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
  tempValue = ADC1BUF0;   // yes, then save ADC value
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

  // Sample/save thermistor connected to RA1/AN1

  AD1CHS = 0x0001;          // Connect RA1/AN1 as CH0 input
  AD1CON1bits.SAMP = 1;     // Sample potentiometer value
  delayMs(1);               // after 1mS start conversion
  AD1CON1bits.SAMP = 0;     // Convert potentiometer value
  while(!AD1CON1bits.DONE); // conversion done? (takes 12*Tad)
  tempValue = ADC1BUF0;   // yes, then save ADC value
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
    //If both opto sensors detect black line
     if (leftOpto < LEFT_THRESHOLD && rightOpto < RIGHT_THRESHOLD)
     {
         //If robot detects black line for the first time
          if (blkDetect == 0)
          {
                //While opto detects the line, move pass the black line (starting point)
                 while (leftOpto < LEFT_THRESHOLD && rightOpto < RIGHT_THRESHOLD)
                {
                    forward();
                }
                    //Intialize that the starting line had been passed
                     blkDetect++;
          }
          //Determine and place pail according to temperature
         else if (blkDetect == 1)
          {
              brakes();
              liftArm();
              delayMs(3000); //Delay time to determine temperature of pail
              get_Inputs(); //get input of pail temperature

              //While both sensors detect black, move forward to pass the black square
              while (leftOpto < LEFT_THRESHOLD && rightOpto < RIGHT_THRESHOLD)
                  {
                      forward();
                  }

              //if temperature is detected as cold
              if (tempValue <= COLD_THRESHOLD)
              {
                  tempPlace = COLD; //Initialize that temperature is at cold
                  LATBbits.LATB8 = 1; //Intialize on LED to move right
                  //while both sensors detect white, turn to direction of the cold base
                  while (leftOpto > LEFT_THRESHOLD && rightOpto > RIGHT_THRESHOLD)
                  {
                      right();
                  }
                  //while any sensor detect black, move pass the black forward line
                  while (leftOpto < LEFT_THRESHOLD || rightOpto < RIGHT_THRESHOLD)
                  {
                      right();
                  }
                  //while left sensor detects white, turn until the line towards the cold base is detected
                  while (leftOpto > LEFT_THRESHOLD)
                  {
                      right();
                  }
              }
              //if temperature is classified as cold
              else if (tempValue >= HOT_THRESHOLD)
              {
                  tempPlace = HOT; //Initialize that temperature is at hot
                  LATBbits.LATB6 = 1; //Intialize on LED to move left
                  //while both sensors detect white, turn to direction of the hot base
                  while (leftOpto > LEFT_THRESHOLD && rightOpto > RIGHT_THRESHOLD)
                  {
                      left();
                  }
                  //while any sensor detect black, move pass the black forward line
                  while (leftOpto < LEFT_THRESHOLD || rightOpto < RIGHT_THRESHOLD)
                  {
                      left();
                  }
                  //while left sensor detects white, turn until the line towards the hot base is detected
                  while (rightOpto > RIGHT_THRESHOLD)
                  {
                      left();
                  }
              }
              //if temperature is within room temperature
              else
              {
                  tempPlace = WARM; //Initialize that the temperature is warm
                  forward(); //move forward to follow the line towards the warm base
              }
              blkDetect++; //Initialize that temperature has been detected
          }

          //place pail into designated base
          else if (blkDetect == 2)
          {
              brakes(); //stop to place pail
              lowerArm(); //lower arm to place pail
              turnB(); //turn around after placing pail
              blkDetect++; //Initialize that the pail has been placed
          }

          //shift to direction of home base
          else if (blkDetect == 3)
          {
                //move to end of black square
               while (leftOpto < LEFT_THRESHOLD && rightOpto < RIGHT_THRESHOLD)
                  {
                      forward();
                  }
                //turn back to home base line according to hot temperature
              if (tempPlace == HOT)
              {
                  while (leftOpto > LEFT_THRESHOLD && rightOpto > RIGHT_THRESHOLD)
                  {
                      right();
                  }
                  while (leftOpto < LEFT_THRESHOLD || rightOpto < RIGHT_THRESHOLD)
                  {
                      right();
                  }
                  while (leftOpto > LEFT_THRESHOLD)
                  {
                      right();
                  }
              }
              //turn back to home base line according to cold temperature
              else if (tempPlace == COLD)
              {

                   while (leftOpto > LEFT_THRESHOLD && rightOpto > RIGHT_THRESHOLD)
                  {
                      left();
                  }
                  while (leftOpto < LEFT_THRESHOLD || rightOpto < RIGHT_THRESHOLD)
                  {
                      left();
                  }
                  while (rightOpto > RIGHT_THRESHOLD)
                  {
                      left();
                  }
              }
              //Continues to move forward after detecting black line according to warm temperature
              else if (tempPlace == WARM)
              {
                  while (leftOpto < LEFT_THRESHOLD && rightOpto < RIGHT_THRESHOLD)
                  {
                      forward();
                  }
              }
          blkDetect++; //intialize that the black center square has been passed
          }
          //move back into home base
          else 
          {     
              //moves pass through black line on the home base square
            while (leftOpto < LEFT_THRESHOLD || rightOpto < RIGHT_THRESHOLD)
                  {
                      forward();
                  }
              //moves pass through white space inside of the home base square.
            while (leftOpto > LEFT_THRESHOLD || rightOpto > RIGHT_THRESHOLD)
                  {
                      forward();
                  }
            //Reverse to prevent momentum shifting robot forward
            while (leftOpto < LEFT_THRESHOLD || rightOpto < RIGHT_THRESHOLD)
                  {
                      reverse();
                  }
             //stops at end of black line
            while (1)
                  {
                      brakes();
                  }

          }

    }
    //If right sensor detects black, shift left
     else if(rightOpto < RIGHT_THRESHOLD)
     {   
          left(); //turn left
     }
     //If left sensor detects black, shift right
      else if(leftOpto < LEFT_THRESHOLD)
     {
          right(); //turn right

     }
     //If neither sensors detect black, move forward
      else
      {
          forward(); //move forward
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
// Provide a delay of up to 65535 mS
void delayMs(unsigned int ms)
{
   while(ms--)
   {
       asm("repeat #4000"); // 4000 instruction cycles @ 250nS ea. = 1mS
       asm("nop");          // instruction to be repeated 4000x
   }

}
//Reverse robot to the left
void leftReverse (void)
{
    LATBbits.LATB11 = 1; //1-2EN
    LATBbits.LATB13 = 1; //1A
    LATBbits.LATB12 = 0; //2A
}
//move robot forward to the right
void rightForward (void)
{
   LATBbits.LATB10 = 1; //3-4EN
   LATBbits.LATB15 = 0; //3A
   LATBbits.LATB14 = 1; //4A
}
//move robot forward to the left
void leftForward (void)
{
    LATBbits.LATB11 = 1; //1-2EN
    LATBbits.LATB13 = 0; //1A
    LATBbits.LATB12 = 1; //2A
}
//Reverse robot to the right
void rightReverse(void)
{
    LATBbits.LATB10 = 1; //3-4EN
    LATBbits.LATB15 = 1; //3A
    LATBbits.LATB14 = 0; //4A
}
//move the robot forward
void forward (void)
{
    rightForward(); //right wheel forward
    leftForward(); //left wheel forward
    get_Inputs();
}
//move the robot in reverse
void reverse (void)
{
    rightReverse(); //right wheel reverse
    leftReverse();  //left wheel reverse
    get_Inputs();
}
//turn the robot right
void right (void)
{
    leftReverse(); //left wheel reverse
    rightForward(); //right wheel forward
    get_Inputs();
}
//turn the robot left
void left (void)
{
    rightReverse(); //right wheel reverse
    leftForward(); //left wheel forward
    get_Inputs();
}
//turn the robot
void turn (void)
{
    //turn while both sensors or either sensors detect black
    while (rightOpto < RIGHT_THRESHOLD || leftOpto < LEFT_THRESHOLD)
    {
        left();
    } //Stops loop after both sensors detect white
    //turn until sensor has stop detected white
   while (rightOpto > RIGHT_THRESHOLD)
    {
       left();
    } //stop loop after right sensor has detected black
    //turn until left sensor has detected black

}
//turn the robot with reverse
void turnB (void)
{
    reverse(); //reverse back from black space
    delayMs(500);
    //turn while both sensors detect white space to the left
    while (rightOpto > RIGHT_THRESHOLD && leftOpto > LEFT_THRESHOLD)
    {
        left();
    }
   //turn while both sensors or either sensors detect black
    while (rightOpto < RIGHT_THRESHOLD || leftOpto < LEFT_THRESHOLD)
    {
        left();
    } //Stops loop after both sensors detect white
    //turn until sensor has stop detected white
   while (rightOpto > RIGHT_THRESHOLD)
    {
       left();
    } //stop loop after right sensor has detected black
    //turn until left sensor has detected black
}
//stop the robot from moving
void brakes (void)
{
    LATBbits.LATB11 = 1; //1-2EN
    LATBbits.LATB13 = 1; //1A
    LATBbits.LATB12 = 1; //2A

    LATBbits.LATB10 = 1; //3-4EN
    LATBbits.LATB15 = 1; //3A
    LATBbits.LATB14 = 1; //4A
}
//lift robot arm up
void liftArm(void)
{
    //forward();
    OC1R = 1225; //lift arm up to ## degrees after hooked onto pail 1125 875
    //reverse();
}
//lower robot arm down
void lowerArm(void)
{
    brakes();
    delayMs(500);
    OC1R= 900;
    delayMs(500);
    OC1R= 650; //lower arm to 0 degrees to release pail 900 600
    
}

// end main.c

