//


#ifndef TMF8828_EXPERIMENT_BOARD_TMF8828_SENSOR_H
#define TMF8828_EXPERIMENT_BOARD_TMF8828_SENSOR_H

#include "pico/stdlib.h"
#include "fsp_error.h"
#include "common_utils.h"
#include "tmf8828_image.h"
#include "tmf8828_shim.h"

// tmf8828 has as default i2c slave address
#define TMF8828_SLAVE_ADDR          0x41

// important wait timings
#define CAP_DISCHARGE_TIME_MS       3                     // wait time until we are sure the PCB's CAP has dischared properly
#define ENABLE_TIME_MS              1                     // wait time after enable pin is high
#define CPU_READY_TIME_MS           1                     // wait time for CPU ready

// return codes from the bootloader
#define BL_SUCCESS_OK               0                     // success
#define BL_ERROR_CMD                -1                    // command/communication failed
#define BL_ERROR_TIMEOUT            -2                    // communication timeout

// return codes from the measurement application
#define APP_SUCCESS_OK              (BL_SUCCESS_OK)       // success
#define APP_ERROR_CMD               (BL_ERROR_CMD)        // command/communication failed
#define APP_ERROR_TIMEOUT           (BL_ERROR_TIMEOUT)    // timeout
#define APP_ERROR_PARAM             -3                    // invalid parameter (e.g. spad map id wrong)
#define APP_ERROR_NO_RESULT_PAGE    -4                    // did not receive a measurement result page
#define APP_ERROR_NO_CALIB_PAGE     -5                    // this is no factory calibration page

// Interrupt bits
#define TMF8828_APP_I2C_ANY_IRQ_MASK                        0x01        /*!< any of below interrupts has occured */
#define TMF8828_APP_I2C_RESULT_IRQ_MASK                     0x02        /*!< a measurement result is ready for readout */
#define TMF8828_APP_I2C_ALT_RESULT_IRQ_MASK                 0x04        /*!< used for statistics and electrical calibration results */
#define TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK              0x08        /*!< a raw histogram is ready for readout */
#define TMF8828_APP_I2C_BREAKPOINT_IRQ_MASK                 0x10        /*!< a breakpoint has been hit */
#define TMF8828_APP_I2C_CMD_DONE_IRQ_MASK                   0x20        /*!< a received I2C command has been handled (successfully or failed) */
#define TMF8828_APP_I2C_ERROR_IRQ_MASK                      0x40        /*!< one of the <status> registers has been set to a non-zero value */

// available SPAD map IDs known by this driver version
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_1 1 // 3x3 map, size 14x6    1. Normal Mode (29°x29°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_2 2 // 3x3 map, size 14x9    2. Macro Mode (29°x43,5°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_3 3 // 3x3 map, size 14x9    3. Macro Mode (29°x43,5°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_4 4 // 4x4 map, size 14x9    4. Time-multiplexed, Normal/Macro Mode (29°x43,5°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_5 5 // 4x4 map, size 14x9    5. Time-multiplexed, Normal/Macro Mode (29°x43,5°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_6 6 // 3x3 map, size 18x10   6. Normal Mode (44°x48°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_7 7 // 4x4 map, size 18x10   7. Time-multiplexed, Normal Mode (44°x48°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_8 8 // 9 zones map, size 14x9  8. Normal/Macro Mode (29°x43,5°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_9 9 // 9 zones map, size 14x9  9. Normal/Macro Mode (29°x43,5°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_10 10 // 3x6 map, size 18x12       10. Time-multiplexed, (29°x57°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_11 11 // 3x3 map, size 14x6        11. Checkerboard, Normal Mode(29°x29°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_12 12 // 3x3 map, size 14x6        12. Reverse-Checkerboard, Normal Mode(29°x29°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_13 13 // 4x4 map, size 18x8        13. Time-multiplexed, Narrow Mode (29°x39°)
#define TMF882X_COM_SPAD_MAP_ID__spad_map_id__map_no_15 15 // 8x8 mode

// Factory calibration page size - store the complete page in e.g. a file, and reload the complete page
#define TMF8828_COM_CONFIG_FACTORY_CALIB__factory_calibration_size    ((ENABLE_REG)-(TMF8828_COM_CONFIG_RESULT))

// --------------------------------------------------- bootloader -----------------------------

#define TMF8X2X_BL_MAX_DATA_SIZE                  0x80  // Number of bytes that can be written or read with one BL command
#define TMF8828_COM_CMD_STAT                      0x08

#define TMF8828_COM_CMD_STAT__bl_cmd_ok           0x00
#define TMF8828_COM_CMD_STAT__bl_cmd_errors       0x0F  // all success/error are below or equal to this number
#define TMF8828_COM_CMD_STAT__bl_cmd_ramremap     0x11  // Bootloader command to remap the vector table into RAM (Start RAM application).
#define TMF8828_COM_CMD_STAT__bl_cmd_r_ram        0x40  // Read from BL RAM.
#define TMF8828_COM_CMD_STAT__bl_cmd_w_ram        0x41  // Write to BL RAM.
#define TMF8828_COM_CMD_STAT__bl_cmd_addr_ram     0x43  // Set address pointer in RAM for Read/Write to BL RAM.

#define BL_HEADER           2     // bootloader header is 2 bytes
#define BL_MAX_DATA_PAYLOAD 128   // bootloader data payload can be up to 128
#define BL_FOOTER           1     // bootloader footer is 1 byte

// Bootloader maximum wait sequences
#define BL_CMD_SET_ADDR_TIMEOUT_MS    1
#define BL_CMD_W_RAM_TIMEOUT_MS       1
#define BL_CMD_RAM_REMAP_TIMEOUT_MS   1

// wait for version readout, to switch from ROM to RAM (and have the version published on I2C)
#define APP_PUBLISH_VERSION_WAIT_TIME_MS 10

// --------------------------------------------------- application ----------------------------

// application status, we check only for ok or accepted, everything between 2 and 15 (inclusive)
// is an error
#define TMF8828_COM_CMD_STAT__stat_ok                       0x0  // Everything is okay
#define TMF8828_COM_CMD_STAT__stat_accepted                 0x1  // Everything is okay too, send sop to halt ongoing command

// application commands
#define TMF8828_COM_CMD_STAT__cmd_measure                             0x10  // Start a measurement
#define TMF8828_COM_CMD_STAT__cmd_stop                                0xff  // Stop a measurement
#define TMF8828_COM_CMD_STAT__cmd_write_config_page                   0x15  // Write the active config page
#define TMF8828_COM_CMD_STAT__cmd_load_config_page_common             0x16  // Load the common config page
#define TMF8828_COM_CMD_STAT__cmd_load_config_page_factory_calib      0x19  // Load the factory calibration config page
#define TMF8828_COM_CMD_STAT__cmd_stat__CMD_RESET_FACTORY_CALIBRATION 0x1F // Manually reset the factory calibration. Only supported if 8x8_measurements = 1.
#define TMF8828_COM_CMD_STAT__cmd_factory_calibration                 0x20  // Perform a factory calibration

#define TMF8828_COM_CMD_STAT__cmd_i2c_slave_address                   0x21  // change I2C address

#define TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8821_MODE       0x65 // Switch to 3x3/3x6/4x4 mode. The device will need to be re-configured after this command. Only supported if 8x8_measurements = 1.
#define TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8828_MODE       0x6C // Switch to 8x8 mode. The device will need to be re-configured after this command. Only supported if 8x8_measurements = 1.

// configuration page addresses and defines
#define TMF8828_COM_PERIOD_MS_LSB                           0x24  // period in milliseconds
#define TMF8828_COM_PERIOD_MS_MSB                           0x25
#define TMF8828_COM_KILO_ITERATIONS_LSB                     0x26  // Kilo (1024) iterations
#define TMF8828_COM_KILO_ITERATIONS_MSB                     0x27
#define TMF8828_COM_SPAD_MAP_ID                             0x34  // configure the SPAD map id, with some example maps
#define TMF8828_COM_SPAD_MAP_ID__map_last                   0x15  // maximum allowed spad map id, for tmf8828 only 15 is allowed
#define TMF8X2X_COM_HIST_DUMP                               0x39  // 0 ... all off, 1 ... raw histograms, 2 ... ec histograms
#define TMF8X2X_COM_I2C_SLAVE_ADDRESS                       0x3b  // register that holds the 7-bit shifted slave address
#define TMF8X2X_COM_ALG_SETTING_0                           0x35  // register that holds the algorithm settings

// Application maximum wait sequences
#define APP_CMD_LOAD_CONFIG_TIMEOUT_MS                      3
#define APP_CMD_WRITE_CONFIG_TIMEOUT_MS                     3
#define APP_CMD_MEASURE_TIMEOUT_MS                          5
#define APP_CMD_STOP_TIMEOUT_MS                             25
#define APP_CMD_FACTORY_CALIB_TIMEOUT_MS                    2000
#define APP_CMD_I2C_SLAVE_ADDRESS_TIMEOUT_MS                1
#define APP_CMD_SWITCH_MODE_CMD_TIMEOUT_MS                  1     // timeout until command is accepted
#define APP_CMD_SWITCH_MODE_TIMEOUT_MS                      10

// check that we can read a complete result page also in the dataBuffer
// DATA_BUFFER_SIZE is 0xc0 apparently
#define DATA_BUFFER_SIZE                  (TMF8828_COM_CONFIG_FACTORY_CALIB__factory_calibration_size)

#if ( ( (BL_HEADER + BL_MAX_DATA_PAYLOAD + BL_FOOTER + 1) > DATA_BUFFER_SIZE ) || ( (TMF8828_COM_CONFIG_RESULT__measurement_result_size) > DATA_BUFFER_SIZE ) )
// #error "Increase data buffer size"
#endif


// clock correction pairs index calculation
#define CLK_CORRECTION_IDX_MODULO( x )    ( (x) & ( (CLK_CORRECTION_PAIRS)-1 ) )

// how accurate the calculation is going to be. The higher the accuracy the less far apart are
// the pairs allowed. An 8 precision means that the factor is 1/256 accurate.
#define CALC_PRECISION                                                  8
// Need this to add to the corrected distance before shifting right
#define HALF_CALC_PRECISION                                             ( 1 << ((CALC_PRECISION) - 1 ) )
#define CALC_DISTANCE_CORR_FACTOR( hostTickDiff, tmf8828TickDiff )      ( ( ( (hostTickDiff) * (TMF8828_TICKS_PER_US) ) << (CALC_PRECISION) ) / ( (tmf8828TickDiff) * (HOST_TICKS_PER_US) ) )
// Round before performing the division (right shift), make sure it is a logical shift right and not an arithmetical shift right
#define CALC_DISTANCE( distance, hostTickDiff, tmf8828TickDiff )        ( ( (uint32_t)( (distance) * CALC_DISTANCE_CORR_FACTOR( hostTickDiff, tmf8828TickDiff ) + (HALF_CALC_PRECISION) ) ) >> (CALC_PRECISION) )

// Find the maximum distance values to avoid mathematical errors due to overflow
#define MAX_HOST_DIFF_VALUE                                             ( ( 0xFFFFFFFFUL / (TMF8828_TICKS_PER_US) ) >> CALC_PRECISION )
#define MAX_TMF8828_DIFF_VALUE                                          ( ( 0xFFFFFFFFUL / (HOST_TICKS_PER_US) )

// Saturation macro for 16-bit
#define SATURATE16( v )                                                 ( v > 0xFFFF ? 0xFFFF : (uint16_t)v )

// For TMF882x sys ticks to be valid the LSB must be set.
#define TMF8828_SYS_TICK_IS_VALID( tick )                               ( (tick) & 1 )

// ---------------------------------------------- functions ---------------------------------------
// Power and bootloader functions are available with ROM code.
// ---------------------------------------------- functions ---------------------------------------

// Function to initialise the driver data structure, call this as the first function
// of your program, before using any other function of this driver
// driver ... pointer to an instance of the tmf8828 driver data structure
// enablePin ... pin number for enable line
// interruptPin ... pin number for interrupt line
void tmf8828Initialise( tmf8828Driver * driver, uint8_t enablePin, uint8_t interruptPin );

// Function to configure the driver how chatty it should be. This is only the driver itself.
// driver ... pointer to an instance of the tmf8828 driver data structure
// level ... the log level for printing
void tmf8828SetLogLevel( tmf8828Driver * driver, uint8_t level );

// Function to set clock correction on or off.
// driver ... pointer to an instance of the tmf8828 driver data structure
// enable ... if <>0 clock correction is enabled (default)
// enable ... if ==0 clock correction is disabled
void tmf8828ClkCorrection( tmf8828Driver * driver, uint8_t enable );

// Function to reset the HW and SW on the device
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828Reset( tmf8828Driver * driver );

// Function to set the enable pin high
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828Enable( tmf8828Driver * driver );

// Function to set the enable pin low
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828Disable( tmf8828Driver * driver );

// Function to put the device in standby mode
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828Standby( tmf8828Driver * driver );

// Function to wake the device up from standby mode
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828Wakeup( tmf8828Driver * driver );

// Function returns true if CPU is ready, else false. If CPU is not ready, device cannot be used.
// driver ... pointer to an instance of the tmf8828 driver data structure
// Returns !=0 if CPU is ready (device can be used), else it returns ==0 (device cannot be accessed).
int8_t tmf8828IsCpuReady( tmf8828Driver * driver, uint8_t waitInMs );

// Function to download the firmware image that was linked against the firmware (tmf8828_image.{h,c} files)
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns BL_SUCCESS_OK if successfully downloaded the FW, else it returns an error BL_ERROR_*
int8_t tmf8828DownloadFirmware( tmf8828Driver * driver );

// Function to change the enable pin to high
fsp_err_t en_pin_high();

// Function to change the enable pin to low
fsp_err_t en_pin_low();

// ------------------------------- application functions --------------------------------------------
// Application functions are only available after a successfull firmware download.
// ------------------------------- application functions --------------------------------------------

// Function writes the current configuration page.
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if successfully written, else it returns an error APP_ERROR_*
int8_t tmf8828WriteConfigPage( tmf8828Driver * driver );

// Function to load the common config page into I2C ram that the host can read/write it via I2C
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if successfully loaded, else it returns an error APP_ERROR_*
int8_t tmf8828LoadConfigPageCommon( tmf8828Driver * driver );

// Function to load the factory calibration config page into I2C ram that the host can read/write it via I2C
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if successfully loaded, else it returns an error APP_ERROR_*
int8_t tmf8828LoadConfigPageFactoryCalib( tmf8828Driver * driver );

// Function to configure the tmf8828 - convenience function that show how to use the tmf8828LoadConfigPage*
// and tmf8828WriteConfigPage functions.
// driver ... pointer to an instance of the tmf8828 driver data structure
// periodInMs ... measurement repetion period (may be higher if kilo-iterations require a longer integration time)
// kiloIterations .. the number of 1024 iterations
// dumpHistogram ... if 1 then dump raw histograms, if ==0 do not dump them, if 2 dump EC histograms, if 3 dump both
// Function returns APP_SUCCESS_OK if successfully configure the device, else it returns an error APP_ERROR_*
int8_t tmf8828Configure( tmf8828Driver * driver, uint16_t periodInMs, uint16_t kiloIterations, uint8_t dumpHistogram  );

// Function to execute an i2c address chagne
// driver ... pointer to an instance of the tmf8828 driver data structure
// newI2cSlaveAddress ... the i2c slave address to be used, until the next device disable+enable
// Function returns APP_SUCCESS_OK if successfully changed i2c Address, else it returns an error APP_ERROR_*
int8_t tmf8828ChangeI2CAddress( tmf8828Driver * driver, uint8_t newI2cSlaveAddress );

// Function to execute a factory calibration
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if successfully factory calibrated, else it returns an error APP_ERROR_*
int8_t tmf8828FactoryCalibration( tmf8828Driver * driver );

// Function to reset the factory calibration. Call this function before providing the 4 factory
// calibration pages for tmf8828.
// driver ... pointer to an instance of the tmf8828 driver data structure
int8_t tmf8828ResetFactoryCalibration( tmf8828Driver * driver );

// Function does load the factory calibration page data from file tmf8828_calibration.c and writes
// it to the device's factory calibration page.
// driver ... pointer to an instance of the tmf8828 driver data structure
// calibPage ... pointer to a complete calibration page (must match the SPAD ID)
// Function returns APP_SUCCESS_OK if successfully factory calibrated, else it returns an error APP_ERROR_*
int8_t tmf8828SetStoredFactoryCalibration( tmf8828Driver * driver, const uint8_t * calibPage );

// Function to start a measurement
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if successfully started a measurement, else it returns an error APP_ERROR_*
int8_t tmf8828StartMeasurement( tmf8828Driver * driver );

// Function to stop a measurement
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if successfully stopped the application, else it returns an error APP_ERROR_*
int8_t tmf8828StopMeasurement( tmf8828Driver * driver );

// Function to switch to 8x8 mode
// driver ... pointer to an instance of the tmf8828 driver data structure
int8_t tmf8828SwitchTo8x8Mode( tmf8828Driver * driver );

// Function to switch to legacy mode (tmf8821/8820 mode = 4x4 or 3x3)
// driver ... pointer to an instance of the tmf8828 driver data structure
int8_t tmf8828SwitchToLegacyMode( tmf8828Driver * driver );

// Function reads the interrupts that are set and clears those.
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns the bit-mask of the interrupts that were set.
uint8_t tmf8828GetAndClrInterrupts( tmf8828Driver * driver, uint8_t mask );

// Function clears the given interrupts and enables them.
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828ClrAndEnableInterrupts( tmf8828Driver * driver, uint8_t mask );

// Function disable the given interrupts.
// driver ... pointer to an instance of the tmf8828 driver data structure
void tmf8828DisableInterrupts( tmf8828Driver * driver, uint8_t mask );

// Function to read results and print them on UART. This function should only be called when there was a
// result interrupt (use function tmf8828GetAndClrInterrupts to find this out).
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if there was a result page, else APP_ERROR_NO_RESULT_PAGE.
int8_t tmf8828ReadResults( tmf8828Driver * driver, int *conf, int *dist, int *subcapture_nr );

// Function to read histograms and print them on UART. This function should only be calle dwhen there was a
// raw histogram interrupt (use function tmf8828GetAndClrInterrupts to find this out).
// driver ... pointer to an instance of the tmf8828 driver data structure
// Function returns APP_SUCCESS_OK if there was a histogram page, else APP_ERROR_NO_RESULT_PAGE.
int8_t tmf8828ReadHistogram( tmf8828Driver * driver );



#endif //TMF8828_EXPERIMENT_BOARD_TMF8828_SENSOR_H
