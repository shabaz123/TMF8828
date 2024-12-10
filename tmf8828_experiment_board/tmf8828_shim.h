//

#ifndef TMF8828_EXPERIMENT_BOARD_TMF8828_SHIM_H
#define TMF8828_EXPERIMENT_BOARD_TMF8828_SHIM_H

#include "pico/stdlib.h"
#include "registers_i2c.h"
#include "tmf8828_sensor.h"

#define TMF8828_DRIVER_MAJOR_VERSION  1
#define TMF8828_DRIVER_MINOR_VERSION  14

/**  Return codes for i2c functions:
 */
#define I2C_SUCCESS             0       /**< successfull execution no error */
#define I2C_ERR_DATA_TOO_LONG   -1      /**< driver cannot handle given amount of data for tx/rx */
#define I2C_ERR_SLAVE_ADDR_NAK  -2      /**< device nak'ed slave address */
#define I2C_ERR_DATA_NAK        -3      /**< device nak'ed written data */
#define I2C_ERR_OTHER           -4      /**< any other error */
#define I2C_ERR_TIMEOUT         -5      /**< timeout in waiting for slave to respond */


#define ARDUINO_MAX_I2C_TRANSFER    32

// Pi Pico pin config:
#define ENABLE_PIN                            9
#define INTERRUPT_PIN                         6
#define TMF_IO0_PIN                          7

// for 2nd tmf8828 on the arduino uno the alternate enable pin is connected to digital 4, alternate interrupt to digital 5
#define ALT_ENABLE_PIN                            4
#define ALT_INTERRUPT_PIN                         5

#define TMF8828_NUMBER_RESULT_RECORDS   4                 // TMF8828 has 4 result records to report all 64 results

// print only the first xx results, it this is more than total exist, it will be shortend to (36== maximum)
#define PRINT_NUMBER_RESULTS        36

// Histogram dumping requires sub-packets
// Register offset of sub-packets
#define TMF8828_COM_SUBPACKET_NUMBER                          0x24      // sub-packet number
#define TMF8828_COM_SUBPACKET_PAYLOAD                         0x25      // sub-packet payload
#define TMF8828_COM_SUBPACKET_CFG_IDX                         0x26      // sub-packet config index (0,1)
#define TMF8828_COM_SUBPACKET_PAYLOAD_0                       0x27
#define TMF8828_COM_HISTOGRAM_PACKET_SIZE                     (4+3+128) // 4 bytes packet header, 3 bytes sub-packet header, 128 paylaod
#define TMF8828_COM_OPTIONAL_SUBPACKET_HEADER_MASK            (0x80)                          /*!< this is the bit that has to be set to indicated in the RID that there is a sub-packet header */
#define TMF8828_COM_HIST_DUMP__histogram__raw_24_bit_histogram 1 // Raw 24 bit histogram
#define TMF8828_COM_HIST_DUMP__histogram__electrical_calibration_24_bit_histogram 2 // Electrical calibration 24 bit histogram
#define TMF8828_NUMBER_OF_BINS_PER_CHANNEL                    128       // how many bins are in a raw histogram per channel


// some more info registers from the results page
#define TMF8828_COM_RESULT_NUMBER         0x24
#define TMF8828_COM_TEMPERATURE           0x25
#define TMF8828_COM_NUMBER_VALID_RESULTS  0x26
#define TMF8828_COM_SYS_TICK_0            0x34      // sys tick is 4 bytes

// each of the result records consist of 3 bytes
#define TMF8828_COM_RES_CONFIDENCE_0      0x38
#define TMF8828_COM_RES_DISTANCE_0_LSB    0x39
#define TMF8828_COM_RES_DISTANCE_0_MSB    0x3a

// result page addresses and defines
#define TMF8828_COM_CONFIG_RESULT                           0x20  // config/result register address
#define TMF8828_COM_CONFIG_RESULT__measurement_result       0x10  // page contains measurment result
#define TMF8828_COM_CONFIG_RESULT__measurement_result_size  (0xa4-0x20)

// Use this macro like this: data[ RESULT_REG( RESULT_NUMBER ) ], it calculates the offset into the data buffer
#define RESULT_REG( regName )             ( (TMF8828_COM_##regName) - (TMF8828_COM_CONFIG_RESULT) )

// ---------------------------------------------- logging -----------------------------------------

#define LOG_LEVEL_NONE              0
#define LOG_LEVEL_ERROR             1           // only error logging - recommended
#define LOG_LEVEL_CLK_CORRECTION    8           // this is a bit-mask check for clock correction logging
#define LOG_LEVEL_INFO              0x10        // some information
#define LOG_LEVEL_VERBOSE           0x20        // very chatty firmware
#define LOG_LEVEL_I2C               0x80        // this is a bit-mask check for i2c logging
#define LOG_LEVEL_DEBUG             0xFF        // everything

#define TMF8828_COM_APP_ID                                  0x0   // register address
#define TMF8828_COM_APP_ID__application                     0x3   // measurement application id
#define TMF8828_COM_APP_ID__bootloader                      0x80  // bootloader application id

#define TMF8828_COM_TMF8828_MODE                            0x10 // mode register is either 0x00 == tmf8820/1 or 0x08 == tmf8828
#define TMF8828_COM_TMF8828_MODE__mode__TMF8821             0    // the device is operating in 3x3/3x6/4x4 (TMF8820/TMF8821) mode
#define TMF8828_COM_TMF8828_MODE__mode__TMF8828             8

// ---------------------------------------------- macros ------------------------------------------

// on the arduino the HEX file is placed inside the program memory region, so we need a special
// read function to access it
//#define READ_PROGRAM_MEMORY_BYTE( address )   pgm_read_byte( address )

// control the arduino digital pins for enable and interrupt
//#define PIN_OUTPUT( pin )                     pinMode( (pin), OUTPUT )
//#define PIN_INPUT( pin )                      pinMode( (pin), INPUT )

//#define PIN_HIGH( pin )                       digitalWrite( (pin), HIGH )
//#define PIN_LOW( pin )                        digitalWrite( (pin), LOW )

// to replace the arduino specific printing
#define PRINT_CHAR(c)                         APP_PRINT("%c", c)
#define PRINT_INT(i)                          APP_PRINT("%d", i)
#define PRINT_INT_HEX(i)                      APP_PRINT("%x", i)
#define PRINT_STR(str)                        APP_PRINT( str )
#define PRINT_LN()                            APP_PRINT( "\n" )

// Which character to use to separate the entries in printing
// need a faulty spelling too, because the code I copied had it : (
#define SEPERATOR                             ','
#define SEPARATOR                             ','

// for clock correction insert here the number in relation to your host
#define HOST_TICKS_PER_US                     1         // host counts ticks every microsecond
#define TMF8828_TICKS_PER_US                  5         // tmf8828 counts ticks 0.2 mircosecond (5x faster than host)

// ---------------------------------------------- types -------------------------------------------



// ---------------------------------------------- functions ---------------------------------------



// Function to allow to wait for some time in microseconds
// wait ... number of microseconds to wait before this functionr returns
void delay_in_microseconds( uint32_t wait );

// Function returns the current sys-tick.
uint32_t get_sys_tick( );

// configures the Pi Pico GPIO pins
void configurePins(void);
// configures the Pi Pico GPIO pins for I2C
void i2c_setup(void);

void tmf8828DisableFn(void);
void tmf8828EnableFn(void);

// I2C transmit only function.
// reg ... the register address to write to
// buf ... pointer to a byte-array that will be transmitted
// len ... number of bytes in the buffer to transmit
void i2c_tx( uint8_t slave_addr, uint8_t reg, const uint8_t * buf, uint8_t len );

// I2C receive only function.
// reg ... the register address to read from
// buf ... pointer to a byte-array where the received bytes will be written to
// len ... number of bytes to receive
void i2c_rx( uint8_t slave_addr, uint8_t reg, uint8_t * buf, uint8_t len );



// Convert 4 bytes in little endian format into an uint32_t
uint32_t tmf8828GetUint32( uint8_t * data );


int8_t i2cTxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData );
int8_t i2cRxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData );
int8_t i2cTxRx ( void * dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t * txData, uint16_t toRx, uint8_t * rxData );





void start_timer();


#endif //TMF8828_EXPERIMENT_BOARD_TMF8828_SHIM_H
