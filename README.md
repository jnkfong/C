# Robotics-Competition
## Line Following Robotics Competition for Computer Engineering Teacher's Association

**Challenge 1:**
In this challenge, the robot must follow a black line path that ends with a straight horizontal line. When the robot reaches this line, it must turn around and follow the path back to reach an identical horizontal line. To complete this challenge, the robot must follow this procedure twice.
 
**Challenge 2:**
In this challenge, the robot must follow a black line path that ends with a straight horizontal line and a aluminum pail beyond it. When the robot reaches this line, it must pick up the pail, turn around and follow the path back with the pail to reach an identical horizontal line, where the pail is dropped (beyond the line). To complete this challenge, the robot must follow

**Challenge 3:**
In this challenge, the robot must follow a black line path that ends with a straight horizontal line and a aluminum pail beyond it. When the robot reaches this line, it must pick up the pail at the centre of the course and determine the temperature of the pail either being hot, warm or cold. Once determined, the robot must place the oail into the associated base. After the pail has been placed, the robot must return to its home base and fit within the base's limits. 

## Hardware Notes:
PIC used
  - PIC24FV32KA302 operating at 8MHz

I/O ports used and hardware attached
  - Output:
    - RB6 connected as output to left sensor LED
    - RB7 connected as output to servo motor
    - RB8 connected as output to right sensor LED
    - RB11-RB13 connected as output to control right motor wheels
    - RB10, RB14-RB15 connected as output to control left motor wheels
  - Input:
    - RA0 connected as input to potentiometer
    - RA1 connected as input to thermistor
    - RA2 connected as input to right opto sensor
    - RB4 connected as input to left opto sensor
