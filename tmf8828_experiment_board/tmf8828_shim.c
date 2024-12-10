//


#include "tmf8828_shim.h"
#include "wire.h"


uint8_t logLevel = 0;
//uint8_t dataBuffer[ DATA_BUFFER_SIZE ];           // transfer/receive buffer

i2c_inst_t *i2c_port_shim;
uint8_t i2c_addr_shim;

void configurePins(void) {
    // configure ENABLE_PIN as output
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    // configure INTERRUPT_PIN as input
    gpio_init(INTERRUPT_PIN);
    gpio_set_dir(INTERRUPT_PIN, GPIO_IN);
    // configure TRIGGER_INTERRUPT_PIN as input
    //gpio_init(TRIGGER_INTERRUPT_PIN);
    //gpio_set_dir(TRIGGER_INTERRUPT_PIN, GPIO_IN);
}



void tmf8828DisableFn(void) {
    gpio_put(ENABLE_PIN, 0);
}

void tmf8828EnableFn(void) {
    gpio_put(ENABLE_PIN, 1);
}

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






// ----------------------------------------- i2c ---------------------------------------

static int8_t i2cTxOnly ( uint8_t logLevel, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData )
{  // split long transfers into max of 32-bytes: 1 byte is register address, up to 31 are payload.
    int8_t res = I2C_SUCCESS;
    int retval = 0;
    uint8_t txbuf[33];
    do
    {
        uint8_t tx;
        if ( toTx > ARDUINO_MAX_I2C_TRANSFER - 1)
        {
            tx = ARDUINO_MAX_I2C_TRANSFER - 1;
        }
        else
        {
            tx = toTx; // less than 31 bytes
        }
        if ( logLevel & LOG_LEVEL_I2C )
        {
            PRINT_STR( "I2C-TX (0x" );
            printf( "%02x", slaveAddr );
            PRINT_STR( ")" );
            PRINT_STR( " tx=" );
            PRINT_INT( tx+1 );          // +1 for regAddr
            PRINT_STR( " 0x" );
            printf( "%02x", regAddr );
            if ( logLevel >= LOG_LEVEL_DEBUG )
            {
                uint8_t dumpTx = tx;
                const uint8_t * dump = txData;
                while ( dumpTx-- )
                {
                    PRINT_STR( " 0x" );
                    printf( "%02x", *dump );
                    dump++;
                }
            }
            PRINT_LN( );
        }

        i2c_addr_shim = slaveAddr;
        txbuf[0] = regAddr;
        for (int i = 0; i < tx; i++) {
            txbuf[i+1] = txData[i];
        }
        retval = i2c_write_blocking(i2c_port_shim, i2c_addr_shim, txbuf, tx+1, false);
        res = I2C_SUCCESS;

        toTx -= tx;
        txData += tx;
        regAddr += tx;

    } while ( toTx && res == I2C_SUCCESS );
    return I2C_SUCCESS;
}

static int8_t i2cRxOnly ( uint8_t logLevel, uint8_t slaveAddr, uint16_t toRx, uint8_t * rxData )
{   // split long transfers into max of 32-bytes
    uint8_t expected = 0;
    int retval = 0;
    uint8_t rx = 0;
    int8_t res = I2C_SUCCESS;
    do
    {
        uint8_t * dump = rxData; // in case we dump on uart, we need the pointer
        if ( toRx > ARDUINO_MAX_I2C_TRANSFER )
        {
            expected = ARDUINO_MAX_I2C_TRANSFER;
        }
        else
        {
            expected = toRx; // less than 32 bytes
        }
        // Wire.requestFrom( slaveAddr, expected );
        rx = 0;
        retval = i2c_read_blocking(i2c_port_shim, i2c_addr_shim, rxData, expected, false);
        rxData += expected;
        toRx -= expected;
        rx += expected;

        if ( logLevel & LOG_LEVEL_I2C )
        {
            PRINT_STR( "I2C-RX (0x" );
            printf( "%02x", slaveAddr );
            PRINT_STR( ")" );
            PRINT_STR( " toRx=" );
            PRINT_INT( rx );
            if ( logLevel >= LOG_LEVEL_DEBUG )
            {
                uint8_t dumpRx = rx;
                while ( dumpRx-- )
                {
                    PRINT_STR( " 0x" );
                    printf( "%02x", *dump );
                    dump++;
                }
            }
            PRINT_LN( );
        }
    } while ( toRx && expected == rx );
    if ( toRx || expected != rx )
    {
        res = I2C_ERR_TIMEOUT;
    }
    return res;
}

int8_t i2cTxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData )
{  // split long transfers into max of 32-bytes
    tmf8828Driver * driver = (tmf8828Driver *)dptr;
    return i2cTxOnly( driver->logLevel, slaveAddr, regAddr, toTx, txData );
}

int8_t i2cRxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData )
{   // split long transfers into max of 32-bytes
    tmf8828Driver * driver = (tmf8828Driver *)dptr;
    int8_t res = i2cTxOnly( driver->logLevel, slaveAddr, regAddr, 0, 0 );
    if ( res == I2C_SUCCESS )
    {
        res = i2cRxOnly( driver->logLevel, slaveAddr, toRx, rxData );
    }
    return res;
}

int8_t i2cTxRx ( void * dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t * txData, uint16_t toRx, uint8_t * rxData )
{
    tmf8828Driver * driver = (tmf8828Driver *)dptr;
    int8_t res = I2C_SUCCESS;
    if ( toTx )
    {
        res = i2cTxOnly( driver->logLevel, slaveAddr, *txData, toTx-1, txData+1 );
    }
    if ( toRx && res == I2C_SUCCESS )
    {
        res = i2cRxOnly( driver->logLevel, slaveAddr, toRx, rxData );
    }
    return res;
}

