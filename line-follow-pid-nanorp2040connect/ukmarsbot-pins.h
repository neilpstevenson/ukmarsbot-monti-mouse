// Input pins:
const int rside = A0; // right side sensor input (start/stop)
const int rfront = A1; //front right line sensor input
const int lfront = A2; //front left line sensor input
const int lside = A3; // left side sensor input (radius)
#define sens1 A4 // unassigned sensor input 1
#define sens2 A5 // unassigned sensor input 2
#define fourwayswitch A6 // input from function switch
#define battery A7 // input for battery measurement
const int Receive = 0; //Receive pin
const int Transmit = 1; //Transmit pin
const int m1encoder1 = 25; // motor 1 encoder 1 input interrupt pin
const int m1encoder2 = 16; // motor 1 encoder 2 input
const int m2encoder1 = 15; // motor 2 encoder 1 input interrupt pin
const int m2encoder2 = 17; // motor 2 encoder 2 input
// Output pins:
const int sensorLED1 = 6; // 1st diagnostic LED on sensor board
const int lmotorDIR = 7; //Left motor dirn input 1
const int rmotorDIR = 9; //8; //Right motor dirn input 3
const int lmotorPWM = 8; //9; //Left motor PWN pin
const int rmotorPWM = 10; //Right motor PWN pin
const int sensorLED2 = 11; // 2nd diagnostic LED on sensor board
const int trigger = 12; // trigger for sensor LEDs
const int LED13 = 13; // ext LED Red

