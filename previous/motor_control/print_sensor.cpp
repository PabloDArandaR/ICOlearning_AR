// Interfaces with IMU sensor
#include "matrix_hal/imu_sensor.h"
// Holds data from IMU sensor
#include "matrix_hal/imu_data.h"

int main() {

    // Create MatrixIOBus object for hardware communication
	matrix_hal::MatrixIOBus bus;
    int speed {0};
    int dir {0};
    
    // Initialize bus and exit program if error occurs
    if (!bus.Init())
    {
	    return false;
    }

    // Create GPIOControl object
	matrix_hal::GPIOControl gpio;

    // Create IMUData object
    matrix_hal::IMUData imu_data;

	// Set gpio to use MatrixIOBus bus
	gpio.Setup(&bus);

}