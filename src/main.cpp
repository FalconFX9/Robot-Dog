#include <Arduino.h>
#include <Servo.h>
#include <Ramp.h>
#include <PPMReader.h>
#include <interpolation.cpp>

byte ppmPin = 3;
byte numChannels = 10;

PPMReader ppm(ppmPin, numChannels);

#define HIP_SERVO_PIN0 6
#define KNEE_SERVO_PIN0 5

short int ULL = 150;
short int LLL = 150;
double t1, t2, added_angle;
Servo hip_servos[4];
Servo knee_servos[4];
const int hip_offsets[] = {90};
const int knee_offsets[] = {50};

Interpolation interpX;
Interpolation interpY;
int step_state = 0;
double x, y, tar_x, tar_y, ptar_x, ptar_y;

void forward_kinematics(double theta1, double theta2, double *xf, double *yf) {
    *xf = -cos(theta1) * (double)ULL + sin(theta2) * (double)LLL;
    *yf = sin(theta1) * (double)ULL + cos(theta2) * (double)LLL;
}


void get_angles_from_length(double l, double *theta1, double *theta2) {
    *theta1 = PI / 2 - acos((pow(l, 2) + pow(ULL, 2) - pow(LLL, 2)) / (2.0 * l * (double)ULL));
    *theta2 = acos((pow(LLL,2) + pow(l, 2) - pow(ULL, 2)) / (2.0 * (double) LLL * l));
}


void inverse_kinematics(double xf, double yf, double *theta1, double *theta2, double *add_angle) {
    double new_leg_length = sqrt(pow(xf, 2) + pow(yf, 2));
    *add_angle = atan2(xf, yf);
    get_angles_from_length(new_leg_length, theta1, theta2);
    *theta1 += *add_angle;
    *theta2 += *add_angle;
}


void move_leg_to_pos(double xf, double yf, int leg_num){
    inverse_kinematics(xf, yf, &t1, &t2, &added_angle);
    int micros = (int)map((long)((double)hip_offsets[leg_num] + degrees(t1)), 0, 180, 500, 2500);
    hip_servos[leg_num].writeMicroseconds(micros);
    micros = (int)map((int)((double)knee_offsets[leg_num] + degrees(t2)), 0, 180, 500, 2500);
    knee_servos[leg_num].writeMicroseconds(micros);
}


void step_fsm(double cur_x, double cur_y, double *target_x, double *target_y, double *prev_target_x, double *prev_target_y, int* state, const int step_length, const int step_height){
    if ((abs(cur_x - *target_x) < 10) && (abs(cur_y - *target_y) < 10)) {
        if (*state == 0) { // Bring foot to start of step powerstroke
            (*state)++;
            *prev_target_x = *target_x;
            *prev_target_y = *target_y;
            *target_x = (float) step_length / 2;
            *target_y = step_height;
        } else if (*state == 1) { // Slide foot backwards
            (*state)++;
            *prev_target_x = *target_x;
            *prev_target_y = *target_y;
            *target_x = -(float) step_length / 2;
            *target_y = step_height;
        } else if (*state == 2){ // Bring foot forwards and up 5 cm
            (*state)++;
            *prev_target_x = *target_x;
            *prev_target_y = *target_y;
            *target_x = 0;
            *target_y = step_height - 50;
        }else if (*state == 3){ // Bring foot above step positiongit 
            (*state) = 0;
            *prev_target_x = *target_x;
            *prev_target_y = *target_y;
            *target_x = (float) step_length / 2;
        }
    }
}


void move_interpolation(){
    double prev_target_x, prev_target_y, target_x, target_y, cur_x, cur_y;
    int state = 0;

}


void setup() {
    // Get Startup Angles
    double start_x = 0;
    double start_y = 150;
    Serial.begin(115200);
    // Attach Servos
    hip_servos[0].attach(HIP_SERVO_PIN0);
    knee_servos[0].attach(KNEE_SERVO_PIN0);

    // Move to start pos
    move_leg_to_pos(start_x, start_y, 0);
    x = start_x;
    y = start_y;
    delay(10000);
}

void loop() {
    //for (byte channel = 1; channel <= numChannels; ++channel) {
    //        unsigned value = ppm.latestValidChannelValue(channel, 0);
    //}
    //unsigned int interp_speed = map(ppm.latestValidChannelValue(5, 0), 1000, 2000, 500, 100);
    unsigned int interp_speed = 1000;
    step_fsm(x, y, &tar_x, &tar_y, &ptar_x, &ptar_y, &step_state, 150, 150);
    x = interpX.go(tar_x, interp_speed);
    y = interpY.go(tar_y, interp_speed);
    move_leg_to_pos(x, y, 0);
    Serial.println("Current state:");
    Serial.println(step_state);
    Serial.println(x);
    Serial.println(y);
}