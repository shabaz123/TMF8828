//


#ifndef TMF8828_EXPERIMENT_BOARD_TMF8828_H
#define TMF8828_EXPERIMENT_BOARD_TMF8828_H

#include "pico/stdlib.h"
#include "fsp_error.h"
#include "registers_i2c.h"
#include "wire.h"

// how much logging we want
#define MY_LOG_LEVEL                LOG_LEVEL_NONE

// tmf states
#define TMF8828_STATE_DISABLED      0
#define TMF8828_STATE_STANDBY       1
#define TMF8828_STATE_STOPPED       2
#define TMF8828_STATE_MEASURE       3
#define TMF8828_STATE_ERROR         4
// convenient macro to have a pointer to the driver structure
#define TMF8828_A                   ( &tmf8828 )

// Switch I2C address.
void changeI2CAddress( );

// Select one of the available configurations and configure the device accordingly.
void configure( );

// Function to print the content of these registers, len should be a multiple of 8
void printRegisters( uint8_t regAddr, uint16_t len, char seperator, uint8_t calibId );

// Print the current state (stateTmf8828) in a readable format
void printState( );

// Constructor function
void TMF8828_constructor();
// Start measuring data
void TMF8828_startMeasuring();
// Enables sensor
void TMF8828_enable();
// Update sensor readings
int TMF8828_update_subcapture(int *conf, int *dist, int *subcapture_nr);
// Update sensor readings
int TMF8828_update(int *conf, int *dist);
// Factory calibration
bool TMF8828_factoryCalibration(int8_t conf, bool show);
// Dump registers
void TMF8828_dumpRegisters();
// Update sensor readings 8x8
bool TMF8828_update8x8(int obj0conf[][8], int obj0dist[][8], int obj1conf[][8], int obj1dist[][8]);




#endif //TMF8828_EXPERIMENT_BOARD_TMF8828_H
