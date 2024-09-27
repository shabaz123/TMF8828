//

#include "wire.h"
//#include "i2c_op.h"

i2c_inst_t *i2c_port;
uint8_t i2c_addr;

static bool init = false;

static uint8_t i2c_buff_tx[I2C_BUFF_TX_SIZE] = {0};
static uint8_t i2c_buff_rx[I2C_BUFF_RX_SIZE] = {0};

static int tx_idx, rx_idx, rx_start;

static fsp_err_t wire_clear_buffer_n(uint8_t *buff, uint32_t len) {
    if (buff == NULL) {
        return FSP_ERR_INVALID_POINTER;
    }

    for (size_t i = 0; i < len; i++) {
        buff[i] = 0;
    }
    return FSP_SUCCESS;
}

static fsp_err_t wire_init_communication(uint8_t address) {
    fsp_err_t err = FSP_SUCCESS;

    if (I2C_PORT_SELECTED == 0) {
        i2c_port = &i2c0_inst;
    } else {
        i2c_port = &i2c1_inst;
    }
    i2c_init(i2c_port, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_addr = address;

    /*Clear transmission and reception buffer*/
    tx_idx = rx_idx = 0;
    wire_clear_buffer_n(i2c_buff_tx, I2C_BUFF_TX_SIZE);
    wire_clear_buffer_n(i2c_buff_rx, I2C_BUFF_RX_SIZE);

    return err;
}

static fsp_err_t wire_write(uint8_t* buff, uint32_t len, bool restart) {
    int retval = 0;
    fsp_err_t err = FSP_SUCCESS;
    // restart bool value: true = send a restart condition after the write
    if (restart == true) {
        retval = i2c_write_blocking(i2c_port, i2c_addr, buff, len, true);
    } else {
        retval = i2c_write_blocking(i2c_port, i2c_addr, buff, len, false);
    }
    err = wire_clear_buffer_n(buff, len);
    tx_idx = 0;
    return FSP_SUCCESS;
}

static fsp_err_t wire_read(uint8_t* buff, uint32_t len, bool restart) {
    int retval = 0;
    rx_start = 0;
    fsp_err_t err = wire_clear_buffer_n(buff,(uint32_t) rx_idx);
    rx_idx = 0;
    if (err == FSP_SUCCESS) {
        retval = i2c_read_blocking(i2c_port, i2c_addr, buff, len, restart);
        if (retval < 1) {
            APP_ERR_PRINT("** wire_read failed ** \r\n");
            return FSP_ERR_ABORTED;
        }
        rx_idx =(int) len;
    }
    return FSP_SUCCESS;
}

static fsp_err_t requestFrom(uint8_t devAddr, uint32_t count) {
    (void)devAddr;

    fsp_err_t err = wire_read(i2c_buff_rx, count, false);
    if (err != FSP_SUCCESS) {
        APP_ERR_PRINT("** requestFrom failed ** \r\n");
    }
    return err;
}

static fsp_err_t beginTransmission(uint8_t address) {
    fsp_err_t err = FSP_SUCCESS;
    if (init == false) {
        err = wire_init_communication(address);
        init = true;
    }
    if (err != FSP_SUCCESS) {
        APP_ERR_PRINT("** beginTransmission failed ** \r\n");
        return err;
    }
    return err;
}

static fsp_err_t write(uint8_t data) {
    if (tx_idx < I2C_BUFF_TX_SIZE) {
        i2c_buff_tx[tx_idx++] = data;
        return FSP_SUCCESS;
    }
    else {
        APP_ERR_PRINT("** write failed ** \r\n");
        return FSP_ERR_OUT_OF_MEMORY;
    }
}

static void endTransmission(bool restart) {
    wire_write(i2c_buff_tx,(uint32_t) tx_idx, restart);
    wire_clear_buffer_n(i2c_buff_tx,(uint32_t) tx_idx);
    tx_idx = 0;
}

static fsp_err_t wire_read_register(uint8_t reg_addr, uint8_t* buff, uint32_t len) {
    fsp_err_t err = write(reg_addr);
    endTransmission(true);
    if(err == FSP_SUCCESS) {
        err = wire_read(buff, len, false);
    }
    return err;
}

static uint8_t read() {
    return i2c_buff_rx[rx_start++];
}

static bool available() {
    return rx_start != rx_idx;
}
