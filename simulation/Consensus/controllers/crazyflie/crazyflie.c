/*
 * Crazyflie controller for two drones in Webots
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "pid_controller.h"

#define FLYING_ALTITUDE 1.0
#define FOLLOW_DISTANCE 0.5

int main(int argc, char **argv) {

  wb_robot_init();
  const int timestep = (int)wb_robot_get_basic_time_step();

  const char *robot_name = wb_robot_get_name();
  int is_leader = strcmp(robot_name, "crazyflie0") == 0;

  /* Motors */

  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);

  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);

  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);

  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  /* Sensors */

  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  wb_keyboard_enable(timestep);

  /* Initialization */

  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};

  double past_x = 0;
  double past_y = 0;
  double past_time = wb_robot_get_time();

  gains_pid_t gains_pid;

  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;

  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;

  gains_pid.kp_z = 10;
  gains_pid.ki_z = 5;
  gains_pid.kd_z = 5;

  init_pid_attitude_fixed_height_controller();

  double height_desired = FLYING_ALTITUDE;

  motor_power_t motor_power;

  while (wb_robot_step(timestep) != -1) {

    const double dt = wb_robot_get_time() - past_time;

    /* Sensor measurements */

    actual_state.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actual_state.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    actual_state.altitude = wb_gps_get_values(gps)[2];

    double x = wb_gps_get_values(gps)[0];
    double y = wb_gps_get_values(gps)[1];

    double vx_global = (x - past_x) / dt;
    double vy_global = (y - past_y) / dt;

    double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];

    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);

    actual_state.vx = vx_global * cosyaw + vy_global * sinyaw;
    actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw;

    /* Desired states */

    desired_state.roll = 0;
    desired_state.pitch = 0;
    desired_state.vx = 0;
    desired_state.vy = 0;
    desired_state.yaw_rate = 0;
    desired_state.altitude = height_desired;

    double forward = 0;
    double sideways = 0;
    double yaw_cmd = 0;
    double height_diff = 0;

    if (is_leader) {

      int key = wb_keyboard_get_key();

      while (key > 0) {

        switch (key) {

          case WB_KEYBOARD_UP:
            forward = 0.5;
            break;

          case WB_KEYBOARD_DOWN:
            forward = -0.5;
            break;

          case WB_KEYBOARD_RIGHT:
            sideways = -0.5;
            break;

          case WB_KEYBOARD_LEFT:
            sideways = 0.5;
            break;

          case 'Q':
            yaw_cmd = 1.0;
            break;

          case 'E':
            yaw_cmd = -1.0;
            break;

          case 'W':
            height_diff = 0.1;
            break;

          case 'S':
            height_diff = -0.1;
            break;
        }

        key = wb_keyboard_get_key();
      }

      desired_state.vx = forward;
      desired_state.vy = sideways;
      desired_state.yaw_rate = yaw_cmd;

    } else {

      /* follower drone */

      double target_x = x;
      double target_y = y + FOLLOW_DISTANCE;

      double error_x = target_x - x;
      double error_y = target_y - y;

      desired_state.vx = error_x;
      desired_state.vy = error_y;
      desired_state.yaw_rate = 0;
    }

    height_desired += height_diff * dt;
    desired_state.altitude = height_desired;

    /* PID controller */

    pid_velocity_fixed_height_controller(
        actual_state,
        &desired_state,
        gains_pid,
        dt,
        &motor_power
    );

    /* Motor commands */

    wb_motor_set_velocity(m1_motor, -motor_power.m1);
    wb_motor_set_velocity(m2_motor, motor_power.m2);
    wb_motor_set_velocity(m3_motor, -motor_power.m3);
    wb_motor_set_velocity(m4_motor, motor_power.m4);

    past_time = wb_robot_get_time();
    past_x = x;
    past_y = y;
  }

  wb_robot_cleanup();
  return 0;
}