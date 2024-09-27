//

#include "tmf8828.h"
#include "tmf8828_sensor.h"
#include "tmf8828_calib.h"





// ---------------------------------------------- constants -----------------------------------------

// for each configuration specifiy a period in milli-seconds
const uint16_t configPeriod[3] = {132, 264, 528};

// for each configuration specify the number of Kilo Iterations (Kilo = 1024)
const uint16_t configKiloIter[3] = {250, 500, 1000};


// ---------------------------------------------- variables -----------------------------------------

tmf8828Driver tmf8828;            // instance of tmf8828
extern uint8_t logLevel;          // for i2c logging in shim

int8_t stateTmf8828;              // current state of the device
int8_t configNr;                  // this sample application has only a few configurations it will loop through, the variable keeps track of that
int8_t clkCorrectionOn;           // if non-zero clock correction is on
int8_t dumpHistogramOn;           // if non-zero, dump all histograms

i2c_inst_t *i2c_port_8828;
uint8_t i2c_addr_8828;


// ---------------------------------------------- function declaration ------------------------------

void i2c_setup_8828(void) {
    if (I2C_PORT_SELECTED == 0) {
        i2c_port_8828 = &i2c0_inst;
    } else {
        i2c_port_8828 = &i2c1_inst;
    }
    i2c_init(i2c_port_8828, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

// Switch I2C address.
void changeI2CAddress ( )
{
    uint8_t newAddr = TMF8828_A->i2cSlaveAddress;
    if ( newAddr == TMF8828_SLAVE_ADDR )
    {
        newAddr = TMF8828_SLAVE_ADDR + 1;      // use next i2c slave address
    }
    else
    {
        newAddr = TMF8828_SLAVE_ADDR;         // back to original
    }
    if ( tmf8828ChangeI2CAddress( TMF8828_A, newAddr ) != APP_SUCCESS_OK )
    {
        PRINT_STR( "ERR " );
    }
    PRINT_STR( "I2C Addr=" );
    PRINT_INT( TMF8828_A->i2cSlaveAddress );
    PRINT_LN( );
}

// wrap through the available configurations and configure the device accordingly.
void configure ( )
{
    configNr = configNr + 1;
    if ( configNr > 2 )
    {
        configNr = 0;     // wrap around
    }

    tmf8828Configure( TMF8828_A, configPeriod[configNr], configKiloIter[configNr],(uint8_t) dumpHistogramOn );
    PRINT_STR( "#Conf" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "Period=" );
    PRINT_INT( configPeriod[configNr] );
    PRINT_STR( "ms" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "KIter=" );
    PRINT_INT( configKiloIter[configNr] );
    PRINT_LN( );
}

// Print the current state (stateTmf8828) in a readable format
void printState ( )
{
    PRINT_STR( "state=" );
    switch ( stateTmf8828 )
    {
        case TMF8828_STATE_DISABLED: PRINT_STR( "disabled" ); break;
        case TMF8828_STATE_STANDBY: PRINT_STR( "standby" ); break;
        case TMF8828_STATE_STOPPED: PRINT_STR( "stopped" ); break;
        case TMF8828_STATE_MEASURE: PRINT_STR( "measure" ); break;
        case TMF8828_STATE_ERROR: PRINT_STR( "error" ); break;
        default: PRINT_STR( "???" ); break;
    }
    PRINT_LN( );
}

// print registers either as c-struct or plain
void printRegisters ( uint8_t regAddr, uint16_t len, char seperator, uint8_t calibId )
{
    uint8_t buf[8];
    uint16_t i;
    (void)calibId;
    /*
    if ( seperator == ',' )
    {
      PRINT_STR( "const PROGMEM uint8_t tmf8828_calib_" );
      PRINT_INT( calibId );
      PRINT_STR( "[] = {" );
      PRINT_LN( );
    }
    */
    for ( i = 0; i < len; i += 8 )            // if len is not a multiple of 8, we will print a bit more registers ....
    {
        uint8_t * ptr = buf;
        i2c_rx( TMF8828_A->i2cSlaveAddress, regAddr, buf, 8 );
        if ( seperator == ' ' )
        {
            PRINT_STR( "0x" );
            PRINT_INT_HEX( regAddr );
            PRINT_STR( ": " );
        }
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator );
        PRINT_LN( );
        regAddr = regAddr + 8;
        sleep_ms(25); // shabaz
    }
    /*
    if ( seperator == ',' )
    {
      PRINT_STR( "};" );
      PRINT_LN( );
    }
    */
}


void TMF8828_constructor() {
    TMF8828_enable();
}

void TMF8828_enable() {
    int retval = 0;
    int aux = 0;
    uint8_t buf[1];
    i2c_setup_8828();
    i2c_addr_8828 = 0x41;
    buf[0] = ENABLE_REG;
    retval = i2c_write_blocking(i2c_port_8828, i2c_addr_8828, buf, 1, true);
    retval = i2c_read_blocking(i2c_port_8828, i2c_addr_8828, buf, 1, false);
    aux = buf[0];
    if (aux == 97) {
        tmf8828Enable( TMF8828_A );
        delay_in_microseconds( ENABLE_TIME_MS * 1000 );
        tmf8828ClkCorrection( TMF8828_A,(uint8_t) clkCorrectionOn );
        tmf8828SetLogLevel( TMF8828_A, MY_LOG_LEVEL );
        configNr = 2; // do a wrap around
        configure();// do a default configuration
        stateTmf8828 = TMF8828_STATE_STOPPED;
        return;
    }
    if ( stateTmf8828 == TMF8828_STATE_DISABLED )
    {
        tmf8828Enable( TMF8828_A );
        delay_in_microseconds( ENABLE_TIME_MS * 1000 );
        tmf8828ClkCorrection( TMF8828_A,(uint8_t) clkCorrectionOn );
        tmf8828SetLogLevel( TMF8828_A, MY_LOG_LEVEL );
        tmf8828Wakeup( TMF8828_A );
        if ( tmf8828IsCpuReady( TMF8828_A, CPU_READY_TIME_MS ) )
        {
            if ( tmf8828DownloadFirmware( TMF8828_A ) == BL_SUCCESS_OK )
            {
                stateTmf8828 = TMF8828_STATE_STOPPED;
                if ( APP_SUCCESS_OK != tmf8828SwitchTo8x8Mode( TMF8828_A ) )
                {
                    PRINT_STR( "#Err" );
                    PRINT_CHAR( SEPERATOR );
                    PRINT_STR( "no tmf8828" );
                    PRINT_LN( );
                }
                configNr = 2; // do a wrap around
                configure();// do a default configuration
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
}

void TMF8828_startMeasuring() {
    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        tmf8828ClrAndEnableInterrupts( TMF8828_A, TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );
        tmf8828StartMeasurement( TMF8828_A );
        stateTmf8828 = TMF8828_STATE_MEASURE;
    }
}

int TMF8828_update_subcapture(int *conf, int *dist, int *subcapture_nr) {
    int8_t res = APP_SUCCESS_OK;
    uint8_t intStatus = 0;

    if ( stateTmf8828 == TMF8828_STATE_STOPPED || stateTmf8828 == TMF8828_STATE_MEASURE )
    {
        intStatus = tmf8828GetAndClrInterrupts( TMF8828_A, TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_ANY_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );   // always clear also the ANY interrupt
        if ( intStatus & TMF8828_APP_I2C_RESULT_IRQ_MASK )                      // check if a result is available (ignore here the any interrupt)
        {
            res = tmf8828ReadResults( TMF8828_A, conf, dist, subcapture_nr );
        }
        if ( intStatus & TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK )
        {
            res = tmf8828ReadHistogram( TMF8828_A );                                              // read a (partial) raw histogram
        }
    }

    if ( res != APP_SUCCESS_OK )                         // in case that fails there is some error in programming or on the device, this should not happen
    {
        tmf8828StopMeasurement( TMF8828_A );
        tmf8828DisableInterrupts( TMF8828_A, 0xFF );
        stateTmf8828 = TMF8828_STATE_STOPPED;
        PRINT_STR( "#Err" );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "inter" );
        PRINT_CHAR( SEPERATOR );
        PRINT_INT( intStatus );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "but no data" );
        PRINT_LN( );
        return false;
    }
    return (res == APP_SUCCESS_OK);
}

int TMF8828_update(int *conf, int *dist) {
    int subcapture_nr;
    int8_t res = APP_SUCCESS_OK;
    uint8_t intStatus = 0;

    if ( stateTmf8828 == TMF8828_STATE_STOPPED || stateTmf8828 == TMF8828_STATE_MEASURE )
    {
        intStatus = tmf8828GetAndClrInterrupts( TMF8828_A, TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_ANY_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );   // always clear also the ANY interrupt
        if ( intStatus & TMF8828_APP_I2C_RESULT_IRQ_MASK )                      // check if a result is available (ignore here the any interrupt)
        {
            res = tmf8828ReadResults( TMF8828_A, conf, dist, &subcapture_nr );
        }
        if ( intStatus & TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK )
        {
            res = tmf8828ReadHistogram( TMF8828_A );                                              // read a (partial) raw histogram
        }
    }

    if ( res != APP_SUCCESS_OK )                         // in case that fails there is some error in programming or on the device, this should not happen
    {
        tmf8828StopMeasurement( TMF8828_A );
        tmf8828DisableInterrupts( TMF8828_A, 0xFF );
        stateTmf8828 = TMF8828_STATE_STOPPED;
        PRINT_STR( "#Err" );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "inter" );
        PRINT_CHAR( SEPERATOR );
        PRINT_INT( intStatus );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "but no data" );
        PRINT_LN( );
        return false;
    }
    return (res == APP_SUCCESS_OK);
}

bool TMF8828_factoryCalibration(int8_t conf, bool show) {
    int retval = 0;
    uint8_t buf[1];
    configNr = conf - 1;
    configure( );

    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        PRINT_STR( "Fact Calib" );
        PRINT_LN( );
        tmf8828Configure( TMF8828_A, 1, 4000,  0 );    // no histogram dumping in factory calibration allowed, 4M iterations for factory calibration recommended
        tmf8828ResetFactoryCalibration( TMF8828_A );
        if (  APP_SUCCESS_OK != tmf8828FactoryCalibration( TMF8828_A )
              || APP_SUCCESS_OK != tmf8828FactoryCalibration( TMF8828_A )
              || APP_SUCCESS_OK != tmf8828FactoryCalibration( TMF8828_A )
              || APP_SUCCESS_OK != tmf8828FactoryCalibration( TMF8828_A ))
        {
            PRINT_STR( "#Err" );
            PRINT_CHAR( SEPERATOR );
            PRINT_STR( "fact calib" );
            PRINT_LN( );
        }
        tmf8828Configure( TMF8828_A, configPeriod[configNr], configKiloIter[configNr],(uint8_t) dumpHistogramOn );
    }

    if ( stateTmf8828 == TMF8828_STATE_STOPPED )
    {
        tmf8828ResetFactoryCalibration( TMF8828_A );
        tmf8828LoadConfigPageFactoryCalib( TMF8828_A );
        if (show) {
            printRegisters( 0x20, 0xE0-0x20, ',', 0 );
            APP_PRINT("\n");
        }
        tmf8828WriteConfigPage( TMF8828_A );                // advance to next calib page
        tmf8828LoadConfigPageFactoryCalib( TMF8828_A );
        if (show) {
            printRegisters( 0x20, 0xE0-0x20, ',', 1 );
            APP_PRINT("\n");
        }
        tmf8828WriteConfigPage( TMF8828_A );               // advance to next calib page
        tmf8828LoadConfigPageFactoryCalib( TMF8828_A );
        if (show) {
            printRegisters( 0x20, 0xE0-0x20, ',', 2 );
            APP_PRINT("\n");
        }
        tmf8828WriteConfigPage( TMF8828_A );               // advance to next calib page
        tmf8828LoadConfigPageFactoryCalib( TMF8828_A );
        if (show) {
            printRegisters( 0x20, 0xE0-0x20, ',', 3 );
            APP_PRINT("\n");
        }
        tmf8828WriteConfigPage( TMF8828_A );                // advance to next calib page
    }
    // Check register for accepted calibration
    i2c_setup_8828();
    i2c_addr_8828 = 0x41;
    buf[0] = 0x07;
    retval = i2c_write_blocking(i2c_port_8828, i2c_addr_8828, buf, 1, true);
    int aux = 0x31;
    retval = i2c_read_blocking(i2c_port_8828, i2c_addr_8828, buf, 1, false);
    if (retval > 0) {
        aux = buf[0];
    }
    if (aux == 0) {
        stateTmf8828 = TMF8828_STATE_STOPPED;
        return true;;
    }
    return false;
}

void TMF8828_dumpRegisters() {
    if ( stateTmf8828 != TMF8828_STATE_DISABLED )
    {
        printRegisters( 0x00, 256, ' ', 0 );
    }
}



bool TMF8828_update8x8(int obj0conf[][8], int obj0dist[][8], int obj1conf[][8], int obj1dist[][8]) {
    int conf[32], dist[32], subcapture_nr, freq[4] = {0}, idx = 0;
    int cnt = 0;

    while (idx < 4) {
        if (TMF8828_update_subcapture(conf, dist, &subcapture_nr)) {
            idx++;
            sleep_ms(15); // shabaz

            for (int i = 0; i < 16; i++) {
                obj0dist[cnt / 8][cnt % 8] = dist[i];
                obj0conf[cnt / 8][cnt % 8] = conf[i];
                obj1dist[cnt / 8][cnt % 8] = dist[i + 16];
                obj1conf[cnt / 8][cnt % 8] = conf[i + 16];
                cnt++;
            }

            subcapture_nr &= 3;
            if (freq[subcapture_nr]) {
                return false;
            }
            freq[subcapture_nr]++;
        }
    }
    if (freq[0] * freq[1] * freq[2] * freq[3] == 0) {
        return false;
    }
    return true;
}
