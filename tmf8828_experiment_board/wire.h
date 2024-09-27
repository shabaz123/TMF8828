//

#ifndef TMF8828_EXPERIMENT_BOARD_WIRE_H
#define TMF8828_EXPERIMENT_BOARD_WIRE_H

#include "pico/stdlib.h"
#include "fsp_error.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "common_utils.h"

// Pi Pico I2C pins configuration
#ifndef I2C_PORT_SELECTED
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_PORT_SELECTED 0
#endif

#define I2C_BUFF_TX_SIZE		8000  /*I2C Transmission Buffer Size*/
#define I2C_BUFF_RX_SIZE		(0x50)  /*I2C Reception Buffer Size*/




#endif //TMF8828_EXPERIMENT_BOARD_WIRE_H
