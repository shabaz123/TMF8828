//


#include "tmf8828_shim.h"
#include "wire.h"

uint8_t logLevel = 0;

i2c_inst_t *i2c_port_shim;
uint8_t i2c_addr_shim;

void delay_in_microseconds ( uint32_t wait )
{
    sleep_us(wait);
}

bool started_timer = false;

void start_timer() {
    // not needed for pi pico
}

uint32_t get_sys_tick ( )
{
    uint32_t usec_timestamp;
    absolute_time_t time_64;
    // get pi pico timer value
    time_64 = get_absolute_time();
    time_64 = absolute_time_diff_us(0, time_64);
    uint64_t time_lower = time_64 & 0x00000000FFFFFFFF;
    usec_timestamp = (uint32_t) time_lower;
    return usec_timestamp;
    //if (started_timer == false) {
    //    start_timer();
    //}
    //timer_status_t status;
    //(void) R_AGT_StatusGet(&g_timer0_ctrl, &status);
    //APP_PRINT("Timer value: %d\n", status.counter);
    //return 0xFFFF - status.counter;
    //return micros( );
}

void i2c_setup(void) {
    if (I2C_PORT_SELECTED == 0) {
        i2c_port_shim = &i2c0_inst;
    } else {
        i2c_port_shim = &i2c1_inst;
    }
    i2c_init(i2c_port_shim, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

void i2c_tx ( uint8_t slave_addr, uint8_t reg, const uint8_t *buf, uint8_t len )
{  // split long transfers into max of 32-bytes
    int retval = 0;
    uint8_t txbuf[33];
    do
    {
        uint8_t tx;
        if ( len > ARDUINO_MAX_I2C_TRANSFER - 1)
        {
            tx = ARDUINO_MAX_I2C_TRANSFER - 1;
        }
        else
        {
            tx = len; // less than 31 bytes
        }
        if ( logLevel & LOG_LEVEL_I2C )
        {
            PRINT_STR( "I2C-TX (0x" );
            PRINT_INT_HEX( slave_addr );
            PRINT_STR( ") Reg=0x" );
            PRINT_INT_HEX( reg );
            uint8_t dump_len = tx;
            if ( dump_len )
            {
                PRINT_STR( " len=" );
                PRINT_INT( dump_len );
                if ( logLevel >= LOG_LEVEL_DEBUG )
                {
                    const uint8_t * dump = buf;
                    while ( dump_len-- )
                    {
                        PRINT_STR( " 0x" );
                        PRINT_INT_HEX( *dump );
                        dump++;
                    }
                }
            }
            PRINT_LN( );
        }


        i2c_addr_shim = slave_addr;

        txbuf[0] = reg;
        for (int i = 0; i < tx; i++) {
            txbuf[i+1] = buf[i];
        }
        retval = i2c_write_blocking(i2c_port_shim, i2c_addr_shim, txbuf, tx+1, false);

        len -= tx;
        buf += tx;
        reg += tx;

    } while ( len );
}

void i2c_rx ( uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len )
{   // split long transfers into max of 32-bytes
    int retval = 0;
    do
    {
        if ( logLevel & LOG_LEVEL_I2C )
        {
            PRINT_STR( "I2C-RX (0x" );
            PRINT_INT_HEX( slave_addr );
            PRINT_STR( ") Reg=0x" );
            PRINT_INT_HEX( reg );
        }
        // write the register
        i2c_addr_shim = slave_addr;
        uint8_t txbuf[1];
        txbuf[0] = reg;
        retval = i2c_write_blocking(i2c_port_shim, i2c_addr_shim, txbuf, 1, false);

        uint8_t rx;
        uint8_t * dump = buf; // in case we dump on uart, we need the pointer
        if ( len > ARDUINO_MAX_I2C_TRANSFER )
        {
            rx = ARDUINO_MAX_I2C_TRANSFER;
        }
        else
        {
            rx = len; // less than 32 bytes
        }
        //Wire.requestFrom( slave_addr, rx );
        retval = i2c_read_blocking(i2c_port_shim, i2c_addr_shim, buf, len, false);
        // requestFrom(slave_addr, rx);
        rx = len;
        buf += rx;
        len -= rx;
        // shabaz - the next line seems wrong! I've commented it out.
        //reg += rx;

        if ( logLevel & LOG_LEVEL_I2C )
        {
            if ( rx )
            {
                PRINT_STR( " len=" );
                PRINT_INT( rx );
                if ( logLevel >= LOG_LEVEL_DEBUG )
                {
                    while ( rx-- )
                    {
                        PRINT_STR( " 0x" );
                        PRINT_INT_HEX( *dump );
                        dump++;
                    }
                }
            }
            PRINT_LN( );
        }
    } while ( len );
}


// function prints a single result, and returns incremented pointer
static uint8_t * print_result ( tmf8828Driver * driver, uint8_t * data, int *conf, int *dist, int *idx )
{
    uint8_t confidence = data[0];               // 1st byte is confidence
    uint16_t distance = data[2];                // 3rd byte is MSB distance
    distance = (distance << 8);       // 2nd byte is LSB distnace
    distance = distance + data[1];
    distance = tmf8828CorrectDistance( driver, distance );
    //PRINT_CHAR( SEPERATOR );
    //PRINT_INT( distance );
    //PRINT_CHAR( SEPERATOR );
    //PRINT_INT( confidence );
    int offset = 0;
    if ((*idx) > 35) {
        offset = 4;
    }
    else if ((*idx) > 26) {
        offset = 3;
    }
    else if ((*idx) > 17) {
        offset = 2;
    }
    else if ((*idx) > 8){
        offset = 1;
    }
    conf[(*idx) - offset] = confidence;
    dist[(*idx) - offset] = distance;
    (*idx)++;
    return data+3;                              // for convenience only, return the new pointer
}

// Results printing:
// #Obj,<i2c_slave_address>,<result_number>,<temperature>,<number_valid_results>,<systick>,<distance_0_mm>,<confidence_0>,<distance_1_mm>,<distance_1>, ...
void print_results ( tmf8828Driver * driver, uint8_t * data, uint8_t len, int *conf, int *dist, int *subcapture_nr )
{
    if ( len >= TMF8828_COM_CONFIG_RESULT__measurement_result_size )
    {
        int cnt = 0;
        int8_t i;
        //uint32_t sysTick = tmf8828GetUint32( data + RESULT_REG( SYS_TICK_0 ) );
        //PRINT_STR( "#Obj" );
        //PRINT_CHAR( SEPERATOR );
        //PRINT_INT( driver->i2cSlaveAddress );
        //PRINT_CHAR( SEPERATOR );
        //PRINT_INT( data[ RESULT_REG( RESULT_NUMBER) ] );
        (*subcapture_nr) = data[ RESULT_REG( RESULT_NUMBER) ];
        //PRINT_CHAR( SEPERATOR );
        //PRINT_INT( data[ RESULT_REG( TEMPERATURE )] );
        //PRINT_CHAR( SEPERATOR );
        //PRINT_INT( data[ RESULT_REG( NUMBER_VALID_RESULTS )] );
        //PRINT_CHAR( SEPERATOR );
        //PRINT_INT( sysTick );
        data = data + RESULT_REG( RES_CONFIDENCE_0 );
        for ( i = 0; i < PRINT_NUMBER_RESULTS ; i++ )
        {
            data = print_result( driver, data, conf, dist, &cnt );
        }
        //PRINT_LN( );
    }
    else // result structure too short
    {
        PRINT_STR( "#Err" );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "result too short" );
        PRINT_CHAR( SEPERATOR );
        PRINT_INT( len );
        PRINT_LN( );
    }
}

// Print histograms:
// #Raw,<i2c_slave_address>,<sub_packet_number>,<data_0>,<data_1>,..,,<data_127>
// #Cal,<i2c_slave_address>,<sub_packet_number>,<data_0>,<data_1>,..,,<data_127>
void print_histogram ( tmf8828Driver * driver, uint8_t * data, uint8_t len )
{
    if ( len >= TMF8828_COM_HISTOGRAM_PACKET_SIZE )
    {
        uint8_t i;
        uint8_t * ptr = &( data[ RESULT_REG( SUBPACKET_PAYLOAD_0 ) ] );
        if ( data[0] & TMF8828_COM_HIST_DUMP__histogram__raw_24_bit_histogram )
        {
            PRINT_STR( "#Raw" );
        }
        else if ( data[0] & TMF8828_COM_HIST_DUMP__histogram__electrical_calibration_24_bit_histogram )
        {
            PRINT_STR( "#Cal" );
        }
        else
        {
            PRINT_STR( "#???" );
        }
        PRINT_CHAR( SEPERATOR );
        PRINT_INT( driver->i2cSlaveAddress );
        PRINT_CHAR( SEPERATOR );
        PRINT_INT( data[ RESULT_REG( SUBPACKET_NUMBER ) ] );          // print the sub-packet number indicating the third-of-a-channel/tdc the histogram belongs to

        for ( i = 0; i < TMF8828_NUMBER_OF_BINS_PER_CHANNEL ; i++, ptr++ )
        {
            PRINT_CHAR( SEPERATOR );
            PRINT_INT( *ptr );
        }
        PRINT_LN( );
    }
    // else structure too short
}

