/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use a cubic function applied on the vertical difference to stabilize the robot vertically.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

// global variables declaration
#define TIME_STEP 10
#define PI 3.14159
#define ARRAY_SIZE 1500   // sampling times
static double t0 = 0.0;   // start time
static double t1 = 15.0;  // end time
static double Ts = 0.010; // sampling period

/* reference: [1] https://github.com/cyberbotics/webots/blob/master/projects/robots/dji/mavic/controllers/mavic2pro/mavic2pro.c*/

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

struct _archive{
    double roll_angle_archive[ARRAY_SIZE];              // roll angle of quadcopter
    double pitch_angle_archive[ARRAY_SIZE];             // pitch angle of quadcopter
    double yaw_angle_archive[ARRAY_SIZE];               // yaw angle of quadcopter
    double altitude_archive[ARRAY_SIZE];                // flight altitude of a quadcopter
    double x_coordinate_archive[ARRAY_SIZE];            // x coordinate of the drone
    double y_coordinate_archive[ARRAY_SIZE];            // y coordinate of the drone
    double roll_angular_velocity_archive[ARRAY_SIZE];   // roll angular velocity of the drone
    double pitch_angular_velocity_archive[ARRAY_SIZE];  // pitch angular velocity of the drone
    double yaw_angular_velocity_archive[ARRAY_SIZE];    // yaw angular velocity of the drone
    double control_roll_archive[ARRAY_SIZE];            // control input of roll angle of quadcopter
    double control_pitch_archive[ARRAY_SIZE];           // control input of pitch angle of quadcopter
    double control_yaw_archive[ARRAY_SIZE];             // control input of yaw angle of quadcopter
    double control_altitude_archive[ARRAY_SIZE];        // control input of flight altitude of a quadcopter
    double altitude_difference_archive[ARRAY_SIZE];     // difference of altitude of drone from desired altitude
    double front_left_motor_input_archive[ARRAY_SIZE];  // input value for front left motor of the quadcopter
    double front_right_motor_input_archive[ARRAY_SIZE]; // input value for front right motor of the quadcopter
    double back_left_motor_input_archive[ARRAY_SIZE];   // input value for back left motor of the quadcopter
    double back_right_motor_input_archive[ARRAY_SIZE];  // input value for back right motor of the quadcopter
} archive;

struct _system_state{
    double roll_angle;  // roll angle of quadcopter
    double pitch_angle; // pitch angle of quadcopter
    double yaw_angle;   // yaw angle of quadcopter
    double altitude;    // flight altitude of a quadcopter
} system_state;

struct _controller{
    double k_vertical_thrust;       // minimum thrust required for drone takeoff.
    double k_vertical_offset;       // vertical displacement of gravity center of quadcopter from ground after landing
    double kp_altitude;             // proportional gain of the altitude PID controller
    double kp_roll;                 // proportional gain of the roll angle PID controller
    double kp_pitch;                // proportional gain of the pitch angle PID controller
    double desired_altitude;        // desired altitude, as specified by the user
    bool led_state;                 // state of on-board LEDs
    double x_coordinate;            // x coordinate of the drone
    double y_coordinate;            // y coordinate of the drone
    double roll_angular_velocity;   // roll angular velocity of the drone
    double pitch_angular_velocity;  // pitch angular velocity of the drone
    double yaw_angular_velocity;    // yaw angular velocity of the drone
    double roll_disturbance;        // perturbation of roll angle by keyboard input
    double pitch_disturbance;       // perturbation of pitch angle by keyboard input
    double yaw_disturbance;         // perturbation of yaw angle by keyboard input
    double control_roll;            // control input of roll angle of quadcopter
    double control_pitch;           // control input of pitch angle of quadcopter
    double control_yaw;             // control input of yaw angle of quadcopter
    double control_altitude;        // control input of flight altitude of a quadcopter
    double altitude_difference;     // difference of altitude of drone from desired altitude
    double front_left_motor_input;  // input value for front left motor of the quadcopter
    double front_right_motor_input; // input value for front right motor of the quadcopter
    double back_left_motor_input;   // input value for back left motor of the quadcopter
    double back_right_motor_input;  // input value for back right motor of the quadcopter
} controller;

void CONTROLLER_init(){
    wb_robot_init();

    // obtain four rotor motors and set them to speed mode
    WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
    WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
    WbDeviceTag back_left_motor = wb_robot_get_device("rear left propeller");
    WbDeviceTag back_right_motor = wb_robot_get_device("rear right propeller");
    WbDeviceTag motors[4] = {front_left_motor, front_right_motor, back_left_motor, back_right_motor};
    for (int m = 0; m < 4; ++m){
        wb_motor_set_position(motors[m], INFINITY);
        wb_motor_set_velocity(motors[m], 1.0);
    }

    // display the welcome message
    printf("Start the drone...\n");

    // display manual control message
    printf("You can control the drone with your computer keyboard:\n");
    printf("- 'up': move forward.\n");
    printf("- 'down': move backward.\n");
    printf("- 'right': turn right.\n");
    printf("- 'left': turn left.\n");
    printf("- 'shift + up': increase the target altitude.\n");
    printf("- 'shift + down': decrease the target altitude.\n");
    printf("- 'shift + right': strafe right.\n");
    printf("- 'shift + left': strafe left.\n");

    controller.k_vertical_thrust = 68.5; // minimum thrust required for drone takeoff.
    controller.k_vertical_offset = 0.6;  // vertical displacement of gravity center of quadcopter from ground after landing
    controller.kp_altitude = 3.0;        // proportional gain of the altitude PID controller
    controller.kp_roll = 50.0;           // proportional gain of the roll angle PID controller
    controller.kp_pitch = 30.0;          // proportional gain of the pitch angle PID controller
    controller.desired_altitude = 1.0;   // desired altitude, as specified by the user
}

double CONTROLLER_realize(int i){

    // obtain four rotor motors and set them to speed mode
    WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
    WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
    WbDeviceTag back_left_motor = wb_robot_get_device("rear left propeller");
    WbDeviceTag back_right_motor = wb_robot_get_device("rear right propeller");
    WbDeviceTag motors[4] = {front_left_motor, front_right_motor, back_left_motor, back_right_motor};

    const double time = wb_robot_get_time();  // in seconds.

    // link the front LEDs alternatively with a 1 second rate
    WbDeviceTag front_left_led = wb_robot_get_device("front left led");
    WbDeviceTag front_right_led = wb_robot_get_device("front right led");
    controller.led_state = ((int)time) % 2;
    wb_led_set(front_left_led, controller.led_state);
    wb_led_set(front_right_led, !controller.led_state);

    // get three Euler angles of UAV sing inertial measurement unit sensor
    WbDeviceTag imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, TIME_STEP);
    const double *roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(imu);

    system_state.roll_angle = roll_pitch_yaw[0]; // roll angle of quadcopter
    archive.roll_angle_archive[i] = system_state.roll_angle;
    printf("roll angle: %lf\n", system_state.roll_angle);

    system_state.pitch_angle = roll_pitch_yaw[1]; // pitch angle of quadcopter
    archive.pitch_angle_archive[i] = system_state.pitch_angle;
    printf("pitch angle: %lf\n", system_state.pitch_angle);

    system_state.yaw_angle = roll_pitch_yaw[2]; // yaw angle of quadcopter
    archive.yaw_angle_archive[i] = system_state.yaw_angle;
    printf("yaw angle: %lf\n", system_state.yaw_angle);

    // obtain altitude of UAV using GPS sensor
    WbDeviceTag gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
    const double *coordinate = wb_gps_get_values(gps);

    controller.x_coordinate = coordinate[0]; // x coordinate of the drone
    archive.x_coordinate_archive[i] = controller.x_coordinate;
    printf("x coordinate: %lf\n", controller.x_coordinate);

    controller.y_coordinate = coordinate[1]; // y coordinate of the drone
    archive.y_coordinate_archive[i] = controller.y_coordinate;
    printf("y coordinate: %lf\n", controller.y_coordinate);

    system_state.altitude = coordinate[2]; // flight altitude of the drone
    archive.altitude_archive[i] = system_state.altitude;
    printf("altitude: %lf\n", system_state.altitude);

    // obtain three euler angular velocity of UAV using gyroscopic sensor
    WbDeviceTag gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, TIME_STEP);
    const double *gyro_value = wb_gyro_get_values(gyro);

    controller.roll_angular_velocity = gyro_value[0]; // roll angular velocity of the drone
    archive.roll_angular_velocity_archive[i] = controller.roll_angular_velocity;
    printf("roll angular velocity: %lf\n", controller.roll_angular_velocity);

    controller.pitch_angular_velocity = gyro_value[1]; // pitch angular velocity of the drone
    archive.pitch_angular_velocity_archive[i] = controller.pitch_angular_velocity;
    printf("pitch angular velocity: %lf\n", controller.pitch_angular_velocity);

    controller.yaw_angular_velocity = gyro_value[2]; // yaw angular velocity of the drone
    archive.yaw_angular_velocity_archive[i] = controller.yaw_angular_velocity;
    printf("yaw angular velocity: %lf\n", controller.yaw_angular_velocity);

    WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
    WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
    WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw"); // Not used in this example.

    // based on three Euler angular velocity obtained from gyroscope sensor, camera motors are driven to stabilize camera
    wb_motor_set_position(camera_roll_motor, -0.115 * controller.roll_angular_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * controller.pitch_angular_velocity);
    wb_motor_set_position(camera_yaw_motor, -0.0 * controller.yaw_angular_velocity);

    // perturbation of control algorithm by keyboard input
    controller.roll_disturbance = 0.0;  // perturbation of roll angle by keyboard input
    controller.pitch_disturbance = 0.0; // perturbation of pitch angle by keyboard input
    controller.yaw_disturbance = 0.0;   // perturbation of yaw angle by keyboard input

    int key = wb_keyboard_get_key();
    while (key > 0){
        switch (key){
        case WB_KEYBOARD_UP:
            controller.pitch_disturbance = -2.0;
            break;
        case WB_KEYBOARD_DOWN:
            controller.pitch_disturbance = 2.0;
            break;
        case WB_KEYBOARD_RIGHT:
            controller.yaw_disturbance = -1.3;
            break;
        case WB_KEYBOARD_LEFT:
            controller.yaw_disturbance = 1.3;
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
            controller.roll_disturbance = -1.0;
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
            controller.roll_disturbance = 1.0;
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
            controller.desired_altitude += 0.05;
            printf("desired altitude: %f [m]\n", controller.desired_altitude);
            break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
            controller.desired_altitude -= 0.05;
            printf("desired altitude: %f [m]\n", controller.desired_altitude);
            break;
        }
        key = wb_keyboard_get_key();
    }

    // calculate roll, pitch, yaw and altitude control input
    // control input of roll angle of quadcopter
    controller.control_roll = controller.kp_roll * CLAMP(system_state.roll_angle, -1.0, 1.0) + controller.roll_angular_velocity + controller.roll_disturbance;
    archive.control_roll_archive[i] = controller.control_roll;
    printf("control_roll: %lf\n", controller.control_roll);

    // control input of pitch angle of quadcopter
    controller.control_pitch = controller.kp_pitch * CLAMP(system_state.pitch_angle, -1.0, 1.0) + controller.pitch_angular_velocity + controller.pitch_disturbance;
    archive.control_pitch_archive[i] = controller.control_pitch;
    printf("control_pitch: %lf\n", controller.control_pitch);

    // control input of yaw angle of quadcopter
    controller.control_yaw = controller.yaw_disturbance;
    archive.control_yaw_archive[i] = controller.control_yaw;
    printf("control_yaw: %lf\n", controller.control_yaw);

    // difference of altitude of drone from desired altitude
    controller.altitude_difference = CLAMP(controller.desired_altitude - system_state.altitude + controller.k_vertical_offset, -1.0, 1.0);
    archive.altitude_difference_archive[i] = controller.altitude_difference;
    printf("altitude_difference: %lf\n", controller.altitude_difference);

    // control input of altitude of quadcopter
    controller.control_altitude = controller.kp_altitude * pow(controller.altitude_difference, 3.0);
    archive.control_altitude_archive[i] = controller.control_altitude;
    printf("control_altitude: %lf\n", controller.control_altitude);

    // input value for front left motor of the quadcopter
    controller.front_left_motor_input = controller.k_vertical_thrust + controller.control_altitude - controller.control_roll + controller.control_pitch - controller.control_yaw;
    archive.front_left_motor_input_archive[i] = controller.front_left_motor_input;
    printf("front_left_motor_input: %lf\n", controller.front_left_motor_input);

    // input value for front right motor of the quadcopter
    controller.front_right_motor_input = controller.k_vertical_thrust + controller.control_altitude + controller.control_roll + controller.control_pitch + controller.control_yaw;
    archive.front_right_motor_input_archive[i] = controller.front_right_motor_input;
    printf("front_right_motor_input: %lf\n", controller.front_right_motor_input);

    // input value for back left motor of the quadcopter
    controller.back_left_motor_input = controller.k_vertical_thrust + controller.control_altitude - controller.control_roll - controller.control_pitch + controller.control_yaw;
    archive.back_left_motor_input_archive[i] = controller.back_left_motor_input;
    printf("back_left_motor_input: %lf\n", controller.back_left_motor_input);

    // input value for back right motor of the quadcopter
    controller.back_right_motor_input = controller.k_vertical_thrust + controller.control_altitude + controller.control_roll - controller.control_pitch - controller.control_yaw;
    archive.back_right_motor_input_archive[i] = controller.back_right_motor_input;
    printf("back_right_motor_input: %lf\n", controller.back_right_motor_input);

    wb_motor_set_velocity(front_left_motor, controller.front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -controller.front_right_motor_input);
    wb_motor_set_velocity(back_left_motor, -controller.back_left_motor_input);
    wb_motor_set_velocity(back_right_motor, controller.back_right_motor_input);
}

void saveArchiveToTxt(double *archive, int size, const char *filename){

    FILE *file = fopen(filename, "w+");

    if (file == NULL){
        perror("Failed to open file");
        exit(1);
    }
    else{
        for (int i = 0; i < size; i++){
            fprintf(file, "%lf\n", archive[i]);
        }
        fclose(file);
        printf("Saved to file %s\n", filename);
    }
}

void saveArchive(){
    saveArchiveToTxt(archive.roll_angle_archive, ARRAY_SIZE, "../../../report/roll_angle.txt");
    saveArchiveToTxt(archive.pitch_angle_archive, ARRAY_SIZE, "../../../report/pitch_angle.txt");
    saveArchiveToTxt(archive.yaw_angle_archive, ARRAY_SIZE, "../../../report/yaw_angle.txt");
    saveArchiveToTxt(archive.altitude_archive, ARRAY_SIZE, "../../../report/altitude.txt");
    saveArchiveToTxt(archive.x_coordinate_archive, ARRAY_SIZE, "../../../report/x_coordinate.txt");
    saveArchiveToTxt(archive.y_coordinate_archive, ARRAY_SIZE, "../../../report/y_coordinate.txt");
    saveArchiveToTxt(archive.roll_angular_velocity_archive, ARRAY_SIZE, "../../../report/roll_angular_velocity.txt");
    saveArchiveToTxt(archive.pitch_angular_velocity_archive, ARRAY_SIZE, "../../../report/pitch_angular_velocity.txt");
    saveArchiveToTxt(archive.yaw_angular_velocity_archive, ARRAY_SIZE, "../../../report/yaw_angular_velocity.txt");
    saveArchiveToTxt(archive.control_roll_archive, ARRAY_SIZE, "../../../report/control_roll.txt");
    saveArchiveToTxt(archive.control_pitch_archive, ARRAY_SIZE, "../../../report/control_pitch.txt");
    saveArchiveToTxt(archive.control_yaw_archive, ARRAY_SIZE, "../../../report/control_yaw.txt");
    saveArchiveToTxt(archive.control_altitude_archive, ARRAY_SIZE, "../../../report/control_altitude.txt");
    saveArchiveToTxt(archive.altitude_difference_archive, ARRAY_SIZE, "../../../report/altitude_difference.txt");
    saveArchiveToTxt(archive.front_left_motor_input_archive, ARRAY_SIZE, "../../../report/front_left_motor_input.txt");
    saveArchiveToTxt(archive.front_right_motor_input_archive, ARRAY_SIZE, "../../../report/front_right_motor_input.txt");
    saveArchiveToTxt(archive.back_left_motor_input_archive, ARRAY_SIZE, "../../../report/back_left_motor_input.txt");
    saveArchiveToTxt(archive.back_right_motor_input_archive, ARRAY_SIZE, "../../../report/back_right_motor_input.txt");
}

int main(int argc, char **argv){

    CONTROLLER_init(); // initialize controller parameter
    // PLANT_init();      // initialize plant parameter

    int i = 0;
    while (wb_robot_step(TIME_STEP) != -1){
        for (int j = 0; j < 30; j++){
            printf("*");
        }
        printf("\n");
        double time = i * TIME_STEP / 1000.0 + t0;
        printf("time at step %d: %f\n", i, time);

        if (time > t1){
            break;
        }

        CONTROLLER_realize(i);
        // PLANT_realize(i);
        i++;
    }

    saveArchive();

    wb_robot_cleanup();

    return 0;
}
