/**
 * @brief Basic PID Control of UAV
 * @author SeongChang Park
 * @date 2022-03-08 20:16
 */

#include <cmath>
#include <iostream>
#include <string>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#define SIGN(x) ( (x) > 0 ) - ( (x) < 0 )
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

using namespace std;
using namespace webots;

/*
webot_z = ros_x
webot_x = ros_y
webot_y = ros_z
*/

double pi_to_pi(double angle)
{
  while(angle >= M_PI)
    angle -= 2 * M_PI;
  while(angle <= -M_PI)
    angle += 2 * M_PI;

  return angle;
}

int main(int argc, char** argv)
{
  webots::Robot robot;
  int timestep = (int)robot.getBasicTimeStep();

  // Get + Enable Devices
  auto camera = robot.getCamera("camera");
  camera->enable(timestep);

  auto front_left_led = robot.getLED("front left led");
  auto front_right_led = robot.getLED("front right led");

  auto imu = robot.getInertialUnit("inertial unit");
  imu->enable(timestep);

  auto gps = robot.getGPS("gps");
  gps->enable(timestep);

  auto compass = robot.getCompass("compass");
  compass->enable(timestep);

  auto gyro = robot.getGyro("gyro");
  gyro->enable(timestep);

  auto keyboard = robot.getKeyboard();
  keyboard->enable(timestep);

  // auto camera_roll_motor  = robot.getDevice("camera roll");
  // auto camera_pitch_motor = robot.getDevice("camera pitch");
  // auto camera_yaw_motor   = robot.getDevice("camera yaw");

  // Get each propeller motors
  auto front_left_motor = robot.getMotor("front left propeller");
  auto front_right_motor = robot.getMotor("front right propeller");
  auto rear_left_motor = robot.getMotor("rear left propeller");
  auto rear_right_motor = robot.getMotor("rear right propeller");

  // Set them to velocity mode
  front_left_motor->setPosition(INFINITY);
  front_left_motor->setVelocity(1.0);
  front_right_motor->setPosition(INFINITY);
  front_right_motor->setVelocity(1.0);
  rear_left_motor->setPosition(INFINITY);
  rear_left_motor->setVelocity(1.0);
  rear_right_motor->setPosition(INFINITY);
  rear_right_motor->setVelocity(1.0);

  // Emitter and Receiver
  // auto emitter = robot.getEmitter("emitter");
  // auto receiver = robot.getReceiver("receiver");


  // Welcome message
  cout << string(35, '-') << endl;
  cout << "[" << robot.getName() << "] Formation control started !!" << endl;
  cout << string(35, '-') << endl;

  // Wait a sec
  while(robot.step(timestep) != -1)
  {
    if (robot.getTime() > 1.0) break;
  }

  // Display manual control message.
  if (robot.getName() == "rbt1")
  {
    printf("You can control the drone with your computer keyboard:\n");
    printf("-  'up': move forward.\n");
    printf("-  'down': move backward.\n");
    printf("-  'right': turn right.\n");
    printf("-  'left': turn left.\n");
    printf("-  'shift + up': increase the target altitude.\n");
    printf("-  'shift + down': decrease the target altitude.\n");
    printf("-  'shift + right': strafe right.\n");
    printf("-  'shift + left': strafe left.\n");
  }

  // Constants, empirically found.
  const double k_vertical_thrust = 68.5;//68.5;  // with this thrust, the drone lifts.
  const double k_vertical_offset = 0;//0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;           // P constant of the roll PID.
  const double k_pitch_p = 30.0;          // P constant of the pitch PID.

  // Variables.
  double target_altitude = 1.0;  // The target altitude. Can be changed by the user.

   // Main loop
  while (robot.step(timestep) != -1)
  {
    const double time = robot.getTime();  // in seconds.
    // cout << "time: "<< time << endl;

    // Retrieve robot position using the sensors.
    const double roll  = pi_to_pi(* imu->getRollPitchYaw()    + M_PI / 2.0);
    const double pitch = pi_to_pi(*(imu->getRollPitchYaw()+1));

    const double px = *(gps->getValues());          // gps_x {W}
    const double py = *(gps->getValues()+2);        // gps_z {W}
    const double altitude = *(gps->getValues()+1);  // gps_y {W}
    if (robot.getName() == "rbt1")
    {
      cout << "cur_postion: \n" << endl;
      cout <<"  px: " << px << endl;
      cout <<"  py: " << py << endl;
      cout <<"  pz: " << altitude << endl; 
    }

    const double roll_acceleration  = *gyro->getValues();
    const double pitch_acceleration = *(gyro->getValues()+1);

    // Blink the front LEDs alternatively with a 1 second rate.
    const bool led_state = ((int)time) % 2;
    front_left_led->set(led_state);
    front_right_led->set(led_state);


    // Transform the keyboard input to disturbances on the stabilization algorithm.
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;

    int key = keyboard->getKey();
    // cout << "key: " << key << endl;
    while (key > 0) {
      switch (key) {
        case Keyboard::UP:
          pitch_disturbance = 2.0;
          break;
        case Keyboard::DOWN:
          pitch_disturbance = -2.0;
          break;
        case Keyboard::RIGHT:
          yaw_disturbance = 1.0 * 1.5;
          break;
        case Keyboard::LEFT:
          yaw_disturbance = -1.0 * 1.5;
          break;
        case (Keyboard::SHIFT + Keyboard::RIGHT):
          roll_disturbance = -1.0 * 2;
          break;
        case (Keyboard::SHIFT + Keyboard::LEFT):
          roll_disturbance = 1.0 * 2;
          break;
        case (Keyboard::SHIFT + Keyboard::UP):
          target_altitude += 0.01;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (Keyboard::SHIFT + Keyboard::DOWN):
          target_altitude -= 0.01;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = keyboard->getKey();
    }
    double desired_roll = 0;//2.0  * M_PI / 180;
    double desired_pitch = 0;//2.0 * M_PI / 180;
    if (robot.getTime() > 10) desired_pitch = 10;

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input   = k_roll_p  * CLAMP(roll, -2.0, 2.0)  + roll_acceleration  + roll_disturbance;
    const double pitch_input  = k_pitch_p * CLAMP(pitch, -2.0, 2.0) - pitch_acceleration + pitch_disturbance;
    const double yaw_input    = yaw_disturbance;

    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Motor Mixing Algorithm
    const double front_left_motor_input   = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double front_right_motor_input  = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    const double rear_left_motor_input    = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double rear_right_motor_input   = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    
    front_left_motor  ->setVelocity( front_left_motor_input);
    front_right_motor ->setVelocity(-front_right_motor_input);
    rear_left_motor   ->setVelocity(-rear_left_motor_input);
    rear_right_motor  ->setVelocity( rear_right_motor_input);

    if (robot.getName() == "rbt1")
    {
      cout << "motor_input: \n";
      cout <<"   fl : " << front_left_motor_input << endl;
      cout <<"   fr : " << front_right_motor_input << endl;
      cout <<"   rl : " << rear_left_motor_input << endl;
      cout <<"   rr : " << rear_right_motor_input << endl;
    }
  };

  return 0;
}