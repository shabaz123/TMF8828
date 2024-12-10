/******************************
 * main.c
 * rev 1.0 Sep 2024 shabaz
 * ****************************/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "extrafunc.h"
#include "hardware/gpio.h"
#include "tmf8828.h"
#include "tmf8828_image.h"
#include "tmf8828_calib.h"
#include "tmf8828_shim.h"
#include "tmf8828_sensor.h"

// definitions
// set to 0 to run a simple app
// set to 1 to run the evaluation mode similar to the Arduino version from the manufacturer
#define EVAL_MODE 1

#define TOKEN_RESULT_ERROR 0
#define TOKEN_RESULT_OK 1
#define TOKEN_RESULT_LINE_COMPLETE 2
#define TOKEN_PROGRESS_NONE 0
#define TOKEN_PROGRESS_SEND 1
#define TOKEN_PROGRESS_RECV 2

// eval mode tmf states
#define TMF8828_STATE_DISABLED      0
#define TMF8828_STATE_STANDBY       1
#define TMF8828_STATE_STOPPED       2
#define TMF8828_STATE_MEASURE       3
#define TMF8828_STATE_ERROR         4


#define COL_RED printf("\033[31m")
#define COL_GREEN printf("\033[32m")
#define COL_YELLOW printf("\033[33m")
#define COL_BLUE printf("\033[34m")
#define COL_MAGENTA printf("\033[35m")
#define COL_CYAN printf("\033[36m")
#define COL_RESET printf("\033[0m")

// constants

// eval mode constants
extern const uint16_t configPeriod[3];
extern const uint16_t configKiloIter[3];
const uint8_t  configSpadId[3] = {15, 15, 15};
const uint16_t configLowThreshold = 100; // set the lower threshold to 10cm
const uint16_t configHighThreshold = 500; // set the upper threshold to 50cm
// select perstistence to be: 0==report every distance, even no distance; 1== report every distance that is a distance, 3== report distance only if 3x in range
const uint8_t configPersistance[3] = { 0, 1, 3 };
#define NR_LOG_LEVELS           7
// to increase/decrease logging
const uint8_t logLevels[ NR_LOG_LEVELS ] =
        { LOG_LEVEL_NONE,
          LOG_LEVEL_ERROR,
          LOG_LEVEL_CLK_CORRECTION,
          LOG_LEVEL_INFO,
          LOG_LEVEL_VERBOSE,
          LOG_LEVEL_I2C,
          LOG_LEVEL_DEBUG
        };

// global variables

uint8_t uart_buffer[305];
uint16_t uart_buffer_index = 0;
uint8_t do_echo = 1;
uint8_t token_progress = TOKEN_PROGRESS_NONE;
int expected_num = 0;
uint8_t byte_buffer[256];
uint8_t byte_buffer_index = 0;
uint16_t loop_tick = 0;

// eval mode global variables
//tmf8828Driver tmf8828[1];
extern tmf8828Driver tmf8828; // defined in tmf8828.c
extern int8_t stateTmf8828;              // current state of the device
extern int8_t configNr;                  // this sample application has only a few configurations it will loop through, the variable keeps track of that
int8_t persistenceNr;             // this is to keep track of the selected persistence setting (out of three for this sample application)
extern int8_t clkCorrectionOn;           // if non-zero clock correction is on
extern int8_t dumpHistogramOn;           // if non-zero, dump all histograms
uint8_t logLevelIdx;              // log level index into logLevels array
volatile uint8_t irqTriggered;    // interrupt is triggered or not
extern const unsigned long tmf8828_image_termination;
extern const unsigned long tmf8828_image_start;
extern const unsigned long tmf8828_image_finish;
extern const unsigned long tmf8828_image_length;
extern const unsigned char tmf8828_image[7128];
extern const uint8_t tmf8828_calib_0[192];
extern const uint8_t tmf8828_calib_1[192];
extern const uint8_t tmf8828_calib_2[192];
extern const uint8_t tmf8828_calib_3[192];



/************* prototypes ***************/
// eval mode prototypes
void eval_mode(void);
void print_eval_menu(void);

/************* functions ***************/

// print_buf_hex prints a buffer in hex format, up to 304 bytes
// 000: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F : 0123456789ABCDEF
void
print_buf_hex(uint8_t *buf, uint16_t len) {
    uint16_t i, j;
    uint8_t c;
    uint8_t index = 0;

    for (i = 0; i < len; i += 16) {
        COL_BLUE;
        printf("%03d: ", index);
        COL_CYAN;
        for (j = 0; j < 16; j++) {
            if (i + j < len) {
                printf("%02X ", buf[i + j]);
            } else {
                printf("   ");
            }
        }
        COL_BLUE;
        printf(": ");
        COL_GREEN;
        for (j = 0; j < 16; j++) {
            if (i + j < len) {
                c = buf[i + j];
                if ((c < 32) || (c > 126)) {
                    printf(".");
                } else {
                    printf("%c", c);
                }
            } else {
                printf(" ");
            }
        }
        printf("\n");
        index += 16;
    }
    COL_RESET;
}



// scan_uart_input fill the uart_buffer until a newline is received
// returns number of bytes if a newline is received, 0 otherwise
int
scan_uart_input(void) {
    int c;
    uint16_t num_bytes;
    c = getchar_timeout_us(1000);
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;
    }

    if ((c == 8) || (c==127)) { // backspace pressed
        if (uart_buffer_index > 0) {
            uart_buffer_index--;
            if (do_echo) {
                putchar(8);
                putchar(' ');
                putchar(8);
            }
        }
        return 0;
    }
    if (c == 13) {
        // add a space to simplify token parsing
        uart_buffer[uart_buffer_index++] = ' ';
        uart_buffer[uart_buffer_index] = 0;
        num_bytes = uart_buffer_index;
        uart_buffer_index = 0;
        if (do_echo) {
            printf("\n");
        }
        return num_bytes;
    }
    uart_buffer[uart_buffer_index] = (uint8_t) c;
    if (do_echo) {
        putchar(c);
    }
    uart_buffer_index++;
    if (uart_buffer_index >= 300) {
        uart_buffer_index = 0;
    }
    return 0;
}

int decode_token(char *token) {
    unsigned int val;
    int retval = 0;
    if (strcmp(token, "device?") == 0) {
        printf("TMF8828 Experiment Board\n\r");
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        expected_num = 0;
        byte_buffer_index = 0;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "recv") == 0) {
        if (expected_num == 0) {
            COL_RED;
            printf("No bytes expected\n");
            COL_RESET;
            return TOKEN_RESULT_LINE_COMPLETE;
        }
        byte_buffer_index = 0;
        retval = PICO_ERROR_GENERIC; // nothing to do for now
        if (retval == PICO_ERROR_GENERIC) {
            COL_RED;
            printf("No bytes read.\n");
            COL_RESET;
        } else {
            print_buf_hex(byte_buffer, expected_num);
        }
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "end_tok") == 0) {
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    // done
    COL_RED;
    printf("Unknown command: %s\n", token);
    COL_RESET;
    return TOKEN_RESULT_LINE_COMPLETE;
}

// Parse each space-separated token
int process_line(uint8_t *buf, uint16_t len) {
    int res;
    char token[20];
    uint16_t i = 0;
    uint16_t j = 0;
    if (len == 0) {
        return TOKEN_RESULT_ERROR;
    }
    while (i < len) {
        if (buf[i] == ' ') {
            token[j] = 0;
            //printf("token: %s\n", token);
            res = decode_token(token);
            if (res == TOKEN_RESULT_LINE_COMPLETE) {
                return TOKEN_RESULT_LINE_COMPLETE;
            }
            j = 0;
        } else {
            token[j] = buf[i];
            j++;
        }
        i++;
    }
    res = decode_token("end_tok");
}

int
main(void)
{
    int numbytes;

    stdio_init_all();
    sleep_ms(3000);
    led_setup();
    printf("TMF8828 Experiment Board\n");
    configurePins();
    if (EVAL_MODE) {
        eval_mode(); // this does not return
    }
    // the rest of the code here is a simple app example
    TMF8828_constructor();
    TMF8828_factoryCalibration(0, false); //must precede the startMeasuring function
    TMF8828_startMeasuring();

    printf("Starting main loop\n");
    while (true) {
        numbytes = scan_uart_input();
        if (numbytes > 0) {
            process_line(uart_buffer, numbytes);
        }
        if (loop_tick > 499) {
            loop_tick = 0;
            led_toggle();
            // print 8x8
            int obj0conf[8][8], obj0dist[8][8], obj1conf[8][8], obj1dist[8][8];
            if (TMF8828_update8x8(obj0conf, obj0dist, obj1conf, obj1dist)) {
                APP_PRINT("\nObject0 matrix\n");
                for (int i = 0; i < 8; i++) {
                    for (int j = 0; j < 8; j++) {
                        APP_PRINT("%d, ", obj0dist[i][j]);
                    }
                    APP_PRINT("\n");
                }
                APP_PRINT("\nObject1 matrix\n");
                for (int i = 0; i < 8; i++) {
                    for (int j = 0; j < 8; j++) {
                        APP_PRINT("%d, ", obj1dist[i][j]);
                    }
                    APP_PRINT("\n");
                }
            }
        }
        sleep_ms(1);
        loop_tick++;
    }
    return 0; // OK for warning on this line (unreachable code)
}

// **************************************************************
// ********************** Eval Mode *****************************
// **************************************************************

void resetAppState ( )
{
    stateTmf8828 = TMF8828_STATE_DISABLED;
    configNr = 0;        // rotate through the given configurations
    persistenceNr = 0;
    clkCorrectionOn = 1;
    dumpHistogramOn = 0; // default is off
    irqTriggered = 0;
}

void wakeup ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STANDBY ){
        tmf8828Wakeup( &(tmf8828) );
        if ( tmf8828IsCpuReady( &(tmf8828), CPU_READY_TIME_MS ) ){
            stateTmf8828 = TMF8828_STATE_STOPPED;
        } else {
            stateTmf8828 = TMF8828_STATE_ERROR;
        }
    }
}

// execute a stop
void stop ( )
{
    if ( stateTmf8828 == TMF8828_STATE_MEASURE || stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        tmf8828StopMeasurement( &(tmf8828) );
        tmf8828DisableInterrupts( &(tmf8828), 0xFF );               // just disable all
        stateTmf8828 = TMF8828_STATE_STOPPED;
    }
}

void printDeviceInfo ( )
{
    printf(  "Driver " );
    PRINT_INT( tmf8828.info.version[0] ); PRINT_CHAR( '.' );
    PRINT_INT( tmf8828.info.version[1] );
    printf(  " FW " );
    PRINT_INT( tmf8828.device.appVersion[0] ); PRINT_CHAR( '.' );
    PRINT_INT( tmf8828.device.appVersion[1] ); PRINT_CHAR( '.' );
    PRINT_INT( tmf8828.device.appVersion[2] ); PRINT_CHAR( '.' );
    PRINT_INT( tmf8828.device.appVersion[3] ); PRINT_CHAR( '.' );
    printf(  " Chip " );
    PRINT_INT( tmf8828.device.chipVersion[0] ); PRINT_CHAR( '.' );
    PRINT_INT( tmf8828.device.chipVersion[1] );
    printf(  " Serial 0x");
    printf( "%02x", tmf8828.device.deviceSerialNumber );
    PRINT_LN( );
}

// power down by setting PON=0
void powerDown ( )
{
    if ( stateTmf8828 == TMF8828_STATE_MEASURE ) {
        tmf8828StopMeasurement( &(tmf8828) ); // stop a measurement first
        tmf8828DisableInterrupts( &(tmf8828), 0xFF );               // just disable all
        stateTmf8828 = TMF8828_STATE_STOPPED;
    }
    if ( stateTmf8828 == TMF8828_STATE_STOPPED ) {
        tmf8828Standby( &(tmf8828) );
        stateTmf8828 = TMF8828_STATE_STANDBY;
    }
}

void nextConfiguration ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED ) {
        configNr = configNr + 1;
        if ( configNr > 2 ) {
            configNr = 0;     // wrap around
        }
        configure();
    }
}

static const uint8_t * getPrecollectedFactoryCalibration ( uint8_t id )
{
    const uint8_t * factory_calib;

    factory_calib = tmf8828_calib_0;
    if ( id == 1 )
    {
        factory_calib = tmf8828_calib_1;
    }
    else if ( id == 2 )
    {
        factory_calib = tmf8828_calib_2;
    }
    else if ( id == 3 )
    {
        factory_calib = tmf8828_calib_3;
    }

    return factory_calib;
}

// enable device and download firmware
void enable ( uint32_t imageStartAddress, const unsigned char * image, int32_t imageSizeInBytes )
{
    if ( stateTmf8828 == TMF8828_STATE_DISABLED )
    {
        tmf8828EnableFn();
        sleep_ms(1);
        tmf8828ClkCorrection( &(tmf8828), clkCorrectionOn );
        tmf8828SetLogLevel( &(tmf8828), logLevels[ logLevelIdx ] );
        tmf8828Wakeup( &(tmf8828) );
        if ( tmf8828IsCpuReady( &(tmf8828), CPU_READY_TIME_MS ) )
        {
            if ( tmf8828DownloadFirmware( &(tmf8828)) == BL_SUCCESS_OK )
            {
                printf( " DWNL" );
                PRINT_LN( );
                resetAppState();
                tmf8828SwitchTo8x8Mode( &(tmf8828) );
                configure();
                stateTmf8828 = TMF8828_STATE_STOPPED;
                print_eval_menu(); // prints on UART usage and waits for user input on serial
                tmf8828ReadDeviceInfo( &(tmf8828) );
                printDeviceInfo( );
            }
            else
            {
                stateTmf8828 = TMF8828_STATE_ERROR;
            }
        }
        else
        {
            stateTmf8828 = TMF8828_STATE_ERROR;
        }
    } // else device is already enabled
    else
    {
        tmf8828ReadDeviceInfo( &(tmf8828) );
        printDeviceInfo( );
    }
}

// start measurement
void measure ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        tmf8828ClrAndEnableInterrupts( &(tmf8828), TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );
        tmf8828StartMeasurement( &(tmf8828) );
        stateTmf8828 = TMF8828_STATE_MEASURE;
    }
}

// execute factory calibration in state stopped only
void factoryCalibration ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED ) {
        printf(  "Fact Cal\n");
        tmf8828ConfigureFull( &(tmf8828), 1, 4000, configSpadId[configNr], 0, 0xffff, 0, 0x3ffff, 0 );    // no histogram dumping in factory calibration allowed, 4M iterations for factory calibration recommended
        tmf8828ResetFactoryCalibration( &(tmf8828) );
        if (  APP_SUCCESS_OK == tmf8828FactoryCalibration( &(tmf8828) ) // walk through all 4 calibration
              && APP_SUCCESS_OK == tmf8828FactoryCalibration( &(tmf8828) )
              && APP_SUCCESS_OK == tmf8828FactoryCalibration( &(tmf8828) )
              && APP_SUCCESS_OK == tmf8828FactoryCalibration( &(tmf8828) ))
        {
            configure( );
            return;
        }
        printf(  "#Err,fact calib\n" );
    }
}

// load factory calibration page to I2C registers 0x20...
void loadFactoryCalibration ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {

        tmf8828ResetFactoryCalibration( &(tmf8828) );
        tmf8828LoadConfigPageFactoryCalib( &(tmf8828) );
        printRegisters( 0x20, 0xE0-0x20, ',', 0 );
        tmf8828WriteConfigPage( &(tmf8828) );                // advance to next calib page
        tmf8828LoadConfigPageFactoryCalib( &(tmf8828) );
        printRegisters( 0x20, 0xE0-0x20, ',', 1 );
        tmf8828WriteConfigPage( &(tmf8828) );               // advance to next calib page
        tmf8828LoadConfigPageFactoryCalib( &(tmf8828) );
        printRegisters( 0x20, 0xE0-0x20, ',', 2 );
        tmf8828WriteConfigPage( &(tmf8828) );               // advance to next calib page
        tmf8828LoadConfigPageFactoryCalib( &(tmf8828) );
        printRegisters( 0x20, 0xE0-0x20, ',', 3 );
        tmf8828WriteConfigPage( &(tmf8828) );                // advance to next calib page

    }
}

// restore factory calibration for file tmf8828_calib.c
void restoreFactoryCalibration ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {

        if (  APP_SUCCESS_OK == tmf8828ResetFactoryCalibration( &(tmf8828) )                                             // First reset, then load all 4 calib pages
              && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( &(tmf8828), getPrecollectedFactoryCalibration( 0 ) )
              && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( &(tmf8828), getPrecollectedFactoryCalibration( 1 ) )
              && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( &(tmf8828), getPrecollectedFactoryCalibration( 2 ) )
              && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( &(tmf8828), getPrecollectedFactoryCalibration( 3 ) )
                )
        {
            printf(  "Set fact cal" );
            PRINT_LN( );
            return;
        }


        printf(  "#Err" );
        PRINT_CHAR( ',' );
        printf(  "loadCal"  );
        PRINT_LN( );
    }
}

// enable/disable clock correction
void clockCorrection ( )
{
    clkCorrectionOn = !clkCorrectionOn;       // toggle clock correction on/off
    tmf8828ClkCorrection( &(tmf8828), clkCorrectionOn );
    printf(  "Clk corr is " );
    PRINT_INT( clkCorrectionOn );
    PRINT_LN( );
}

// configure histogram dumping (next dumping bit-mask)
void histogramDumping ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        dumpHistogramOn = dumpHistogramOn + 1;       // select histogram dump on/off, and type of histogram dumping
        if ( dumpHistogramOn > (TMF8828_COM_HIST_DUMP__histogram__electrical_calibration_24_bit_histogram + TMF8828_COM_HIST_DUMP__histogram__raw_24_bit_histogram) )
        {
            dumpHistogramOn = 0; // is off again
        }
        configure( );
        printf(  "Histogram is " );
        PRINT_INT( dumpHistogramOn );
        PRINT_LN( );
    }
}

// set the thresholds to the next configuration
void thresholds ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        persistenceNr = persistenceNr + 1;
        if ( persistenceNr > 2 )
        {
            persistenceNr = 0;     // wrap around
        }
        configure( );
    }
}

#ifdef NOT_REQUIRED
// Switch I2C address.
void changeI2CAddress ( )
{
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        uint8_t newAddr = tmf8828.i2cSlaveAddress;
        if ( newAddr == TMF8828_SLAVE_ADDR )
        {
            newAddr = TMF8828_SLAVE_ADDR + 1;      // use next i2c slave address
        }
        else
        {
            newAddr = TMF8828_SLAVE_ADDR;         // back to original
        }
        if ( tmf8828ChangeI2CAddress( &(tmf8828), newAddr ) != APP_SUCCESS_OK )
        {
            printf(  "#Err"  );
            PRINT_CHAR( ',' );
        }
    }
    printf(  "I2C Addr=" );
    PRINT_INT( tmf8828.i2cSlaveAddress );
    PRINT_LN( );
}
#endif

// decrease logging level
void logLevelDec ( )
{
    if ( logLevelIdx > 0 )
    {
        logLevelIdx--;
        tmf8828SetLogLevel( &(tmf8828), logLevels[ logLevelIdx ] );
    }
    printf(  "Log=" );
    PRINT_INT( logLevels[ logLevelIdx ] );
    PRINT_LN( );
}

// increase logging level
void logLevelInc ( )
{
    if ( logLevelIdx < NR_LOG_LEVELS - 1 )
    {
        logLevelIdx++;
        tmf8828SetLogLevel( &(tmf8828), logLevels[ logLevelIdx ] );
    }
    printf(  "Log=" );
    PRINT_INT( logLevels[ logLevelIdx ] );
    PRINT_LN( );
}

// perform a hardware + software reset
void reset ( )
{
    if ( stateTmf8828 != TMF8828_STATE_DISABLED )
    {
        tmf8828Reset( &(tmf8828) );
        printf(  "Reset TMF8828" );
        PRINT_LN( );
        stateTmf8828 = TMF8828_STATE_STOPPED;
        tmf8828SwitchTo8x8Mode( &(tmf8828) );
    }
}

#ifdef NOT_REQUIRED
// Print the current state (stateTmf8828) in a readable format
void printState ( )
{

    printf(  "TMF8828" );

    printf(  " state="  );
    switch ( stateTmf8828 )
    {
        case TMF8828_STATE_DISABLED: printf(  "disabled" ); break;
        case TMF8828_STATE_STANDBY: printf(  "standby" ); break;
        case TMF8828_STATE_STOPPED: printf(  "stopped" ); break;
        case TMF8828_STATE_MEASURE: printf(  "measure" ); break;
        case TMF8828_STATE_ERROR: printf(  "error" ); break;
        default: printf(  "???" ); break;
    }
    PRINT_LN( );
}
#endif

void print_eval_menu(void) {
    printf(  "TMF8828 Pi Pico Driver\n" );
    printf(  "UART commands\n" );
    printf(  "a ... dump registers\n" );
    printf(  "c ... next configuration\n" );
    printf(  "d ... disable device\n" );
    printf(  "e ... enable device and download TMF8828 FW\n" );
    printf(  "f ... do fact calib\n" );
    printf(  "h ... help \n" );
    printf(  "i ... i2c addr. change\n" );
    printf(  "l ... load fact calib\n" );
    printf(  "m ... measure\n" );
    printf(  "p ... power down\n" );
    printf(  "r ... restore fact calib from file\n" );
    printf(  "s ... stop measure\n" );
    printf(  "t ... next persistance set\n" );
    printf(  "w ... wakeup\n" );
    printf(  "x ... clock corr on/off\n" );
    printf(  "z ... histogram\n" );
    printf(  "+ ... log+\n" );
    printf(  "- ... log-\n" );
    printf(  "# ... reset\n" );
    printf(  "\n");
}

int
eval_mode_scan_serial(void) {
    int c;
    uint16_t num_bytes;
    c = getchar_timeout_us(1000);
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;
    }

    if ((c == 8) || (c==127)) { // backspace pressed
        if (uart_buffer_index > 0) {
            uart_buffer_index--;
            if (do_echo) {
                putchar(8);
                putchar(' ');
                putchar(8);
            }
        }
        return 0;
    }
    if (c == 13) {
        // add a space to simplify token parsing
        uart_buffer[uart_buffer_index++] = ' ';
        uart_buffer[uart_buffer_index] = 0;
        num_bytes = uart_buffer_index;
        uart_buffer_index = 0;
        if (do_echo) {
            printf("\n");
        }
        return num_bytes;
    }
    uart_buffer[uart_buffer_index] = (uint8_t) c;
    if (do_echo) {
        putchar(c);
    }
    // the following is a hack to make the code respond without requiring the user to press Enter.
    // I think this needs to be done, so that the mnfr Python app will run correctly.
    uart_buffer_index++;
    uart_buffer[uart_buffer_index] = ' ';
    uart_buffer_index++;
    uart_buffer[uart_buffer_index] = 0;
    num_bytes = uart_buffer_index;
    uart_buffer_index = 0;
    return num_bytes;

    //if (uart_buffer_index >= 300) {
    //    uart_buffer_index = 0;
    //}
    //return 0;
}

int eval_decode_token(char *token) {
    unsigned int val;
    int retval = 0;
    if (strcmp(token, "h") == 0) {
        print_eval_menu();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "c") == 0) {
        nextConfiguration();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "e") == 0) {
        enable( tmf8828_image_start, tmf8828_image, tmf8828_image_length );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "E") == 0) {
        //we don't support this! Let's just use the TMF8828 image.
        enable( tmf8828_image_start, tmf8828_image, tmf8828_image_length );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "d") == 0) {
        tmf8828DisableFn();
        stateTmf8828 = TMF8828_STATE_DISABLED;
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "w") == 0) {
        wakeup();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "p") == 0) {
        powerDown();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "o") == 0) {
        // we don't support this! Mode is always TMF8828
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "m") == 0) {
        measure( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "s") == 0) {
        stop( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "f") == 0) {
        factoryCalibration( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "l") == 0) {
        loadFactoryCalibration( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "r") == 0) {
        restoreFactoryCalibration( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "z") == 0) {
        histogramDumping( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "a") == 0) {
        if ( stateTmf8828 != TMF8828_STATE_DISABLED ){
            printRegisters( 0x00, 256, ' ', 0 );
        }
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "x") == 0) {
        clockCorrection( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "i") == 0) {
        changeI2CAddress( );
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "t") == 0) {
        thresholds();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "+") == 0) {
        logLevelInc();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "-") == 0) {
        logLevelDec();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "#") == 0) {
        reset();
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "q") == 0) {
        // we don't support this!
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        return TOKEN_RESULT_LINE_COMPLETE;
    }

    if (strcmp(token, "end_tok") == 0) {
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    // done
    printf(  "#Err"  );
    PRINT_CHAR( ',' );
    printf(  "Cmd " );
    if (token != NULL) {
        PRINT_CHAR( token[0] );
    } else {
        PRINT_CHAR( '?' );
    }
    PRINT_LN( );
    return TOKEN_RESULT_LINE_COMPLETE;
}

// Parse each space-separated token
int eval_process_line(uint8_t *buf, uint16_t len) {
    int res;
    char token[20];
    uint16_t i = 0;
    uint16_t j = 0;
    if (len == 0) {
        return TOKEN_RESULT_ERROR;
    }
    while (i < len) {
        if (buf[i] == ' ') {
            token[j] = 0;
            //printf("token: %s\n", token);
            res = eval_decode_token(token);
            if (res == TOKEN_RESULT_LINE_COMPLETE) {
                printState();
                return TOKEN_RESULT_LINE_COMPLETE;
            }
            j = 0;
        } else {
            token[j] = buf[i];
            j++;
        }
        i++;
    }
    res = decode_token("end_tok");
}

void
eval_loop(void) {
    int numbytes;
    int8_t res = APP_SUCCESS_OK;
    uint8_t intStatus = 0;
    numbytes = eval_mode_scan_serial();
    if (numbytes > 0) {
        eval_process_line(uart_buffer, numbytes);
    }
    if ( stateTmf8828 == TMF8828_STATE_MEASURE ) {
        //disableInterrupts( );
        irqTriggered = 0;
        //enableInterrupts( );
        intStatus = tmf8828GetAndClrInterrupts( &(tmf8828), TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_ANY_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );   // always clear also the ANY interrupt
        if ( intStatus & TMF8828_APP_I2C_RESULT_IRQ_MASK )                      // check if a result is available (ignore here the any interrupt)
        {
            res = tmf8828ReadResultsFn( &(tmf8828) );
        }
        if ( intStatus & TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK )
        {
            res = tmf8828ReadHistogram( &(tmf8828) );                                              // read a (partial) raw histogram
        }
    }

    if ( res != APP_SUCCESS_OK )                         // in case that fails there is some error in programming or on the device, this should not happen
    {
        tmf8828StopMeasurement( &(tmf8828) );
        tmf8828DisableInterrupts( &(tmf8828), 0xFF );
        stateTmf8828 = TMF8828_STATE_STOPPED;
        printf(  "#Err"  );
        PRINT_CHAR( ',' );
        printf(  "inter" );
        PRINT_CHAR( ',' );
        PRINT_INT( intStatus );
        PRINT_CHAR( ',' );
        printf(  "but no data" );
        PRINT_LN( );
    }
}

void
eval_mode(void) {
    tmf8828Initialise( &(tmf8828), ENABLE_PIN, INTERRUPT_PIN );
    tmf8828DisableFn();
    sleep_ms(100);
    print_eval_menu();
    while(1) {
        eval_loop();
    }
}

