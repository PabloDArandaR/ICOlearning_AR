// System calls
#include <unistd.h>
// Input/output streams and functions
#include <iostream>

// Interfaces with GPIO
#include "matrix_hal/gpio_control.h"
// Communicates with MATRIX device
#include "matrix_hal/matrixio_bus.h"
// Interfaces with humidity sensor
#include "matrix_hal/humidity_sensor.h"
// Holds data from humidity sensor
#include "matrix_hal/humidity_data.h"

////////////////////////
// MOTOR PORTS //
//////////////////////

#define R_MOTOR_PWM 14 //(ORANGE)
#define L_MOTOR_PWM 15 //(GREEN)
#define R_MOTOR_IN1 13 //(ORANGE)
#define L_MOTOR_IN1 12 //(GREEN)
#define R_MOTOR_IN2 11 //(ORANGE)
#define L_MOTOR_IN2 10  //(GREEN)

////////////////////////
// INITIAL VARIABLES //
//////////////////////


// GPIOOutputMode is 1
const uint16_t GPIOOutputMode = 1;
// GPIOInputMode is 0
const uint16_t GPIOInputMode = 0;

// Holds desired GPIO pin for output [0-15]
uint16_t pin_out;
// Holds desired output state
uint16_t pin_out_state;
// Holds desired GPIO pin for input [0-15]
uint16_t pin_in;

int main() {

  // Create MatrixIOBus object for hardware communication
  matrix_hal::MatrixIOBus bus;

  // Initialize bus and exit program if error occurs
  if (!bus.Init()) {
    return false;
  }

  // Create HumidityData object
  matrix_hal::HumidityData humidity_data;
  // Create HumiditySensor object
  matrix_hal::HumiditySensor humidity_sensor;
  // Set humidity_sensor to use MatrixIOBus bus
  humidity_sensor.Setup(&bus);

  // Endless loop
  while (true) {
      // Overwrites humidity_data with new data from humidity sensor
      humidity_sensor.Read(&humidity_data);
      // Humidity output is represented in %
      float humidity = humidity_data.humidity;
      // Temperature output is represented in Celsius
      float temperature = humidity_data.temperature;
      // Clear console
      std::system("clear");
      // Output sensor data to console
      std::cout << " [ Humidity Sensor Output ]" << std::endl;
      std::cout << " [ Humidity (%) : " << humidity
              << " ] [ Temperature (Celsius) : " << temperature << "]" << std::endl;

      // Sleep for 20000 microseconds
      usleep(500);
  }

  return 0;
  }
