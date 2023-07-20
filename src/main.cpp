#include <Arduino.h>
#include <Servo.h>
#include <Ramp.h>

#define HIP_SERVO_PIN0 3
#define KNEE_SERVO_PIN0 5

short int ULL = 150;
short int LLL = 150;
double t1, t2, added_angle;
Servo hip_servos[4];
Servo knee_servos[4];
const int hip_offsets[] = {90};
const int knee_offsets[] = {50};

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


void move_leg_to_pos(double x, double y, int leg_num){
    inverse_kinematics(x, y, &t1, &t2, &added_angle);
    int micros = (int)map((long)((double)hip_offsets[leg_num] + degrees(t1)), 0, 180, 500, 2500);
    hip_servos[leg_num].writeMicroseconds(micros);
    micros = (int)map((int)((double)knee_offsets[leg_num] + degrees(t2)), 0, 180, 500, 2500);
    knee_servos[leg_num].writeMicroseconds(micros);
}


void step_fsm(double cur_x, double cur_y, double *target_x, double *target_y, double *prev_target_x, double *prev_target_y, int* state){
    if (*state == 0){
        if (abs(cur_x - 50) < 2 && abs(cur_y - 150) < 2){
            *state++;
            *prev_target_x = 50;
            *prev_target_y = 150;
            *target_x = -50;
            *target_y = 150;
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
    Serial.begin(9600);
    // Attach Servos
    hip_servos[0].attach(HIP_SERVO_PIN0);
    knee_servos[0].attach(KNEE_SERVO_PIN0);

    // Move to start pos
    move_leg_to_pos(start_x, start_y, 0);
    delay(10000);
}

void loop() {
    double x = 0;
    double y = 100;
    move_leg_to_pos(x, y, 0);
    delay(5000);
    x = 0;
    y = 250;
    move_leg_to_pos(x, y, 0);
    delay(5000);
}