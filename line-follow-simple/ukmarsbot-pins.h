// Input pins:
const int rside = A0; // left side sensor input
const int rfront = A1; //front left line sensor input
const int lfront = A2; //front right left sensor input
const int lside = A3; // right side sensor input
const int sens1 = A4; // unassigned sensor input 1
const int sens2 = A5; // unassigned sensor input 2
const int fourwayswitch = A6; // input from function switch
const int battery = A7; // input for battery measurement
const int Receive = 0; //Receive pin
const int Transmit = 1; //Transmit pin
const int m1encoder1 = 2; // motor 1 encoder 1 input interrupt pin
const int m1encoder2 = 4; // motor 1 encoder 2 input
const int m2encoder1 = 3; // motor 2 encoder 1 input interrupt pin
const int m2encoder2 = 5; // motor 2 encoder 2 input
// Output pins:
const int sensorLED1 = 6; // 1st diagnostic LED on sensor board
const int lmotorDIR = 7; //Left motor dirn input 1
const int rmotorDIR = 8; //Right motor dirn input 3
const int lmotorPWM = 9; //Left motor PWN pin
const int rmotorPWM = 10; //Right motor PWN pin
const int sensorLED2 = 11; // 2nd diagnostic LED on sensor board
const int trigger = 12; // trigger for sensor LEDs
const int LED13 = 13; // ext LED Red
