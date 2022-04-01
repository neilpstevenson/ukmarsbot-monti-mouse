const int BASE_SPEED = 100;
const int START_STOP_COUNT = 4;
const int SLOWDOWN_SPEED_RATIO = 2; // Half speed
const int SLOWDOWN_TIME = 200000/BASE_SPEED/SLOWDOWN_SPEED_RATIO;

// PID values
const float LOOP_INTERVAL = 0.003;  // 1mS
const float LOOP_FREQUENCY = 1/LOOP_INTERVAL;  // Hz
const float MAX_MOTOR_VOLTS = BASE_SPEED;
const float PID_Kp = 0.6;
const float PID_Ki = 0.0;
const float PID_Kd = 0.15;

// Marker thresholds
const int markerLowThreshold = 100;
const int markerHighThreshold = 300;

const int sensorthreshold = 500; // LED illumination threshold
