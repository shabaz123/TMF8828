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

// definitions
#define TOKEN_RESULT_ERROR 0
#define TOKEN_RESULT_OK 1
#define TOKEN_RESULT_LINE_COMPLETE 2
#define TOKEN_PROGRESS_NONE 0
#define TOKEN_PROGRESS_SEND 1
#define TOKEN_PROGRESS_RECV 2

#define COL_RED printf("\033[31m")
#define COL_GREEN printf("\033[32m")
#define COL_YELLOW printf("\033[33m")
#define COL_BLUE printf("\033[34m")
#define COL_MAGENTA printf("\033[35m")
#define COL_CYAN printf("\033[36m")
#define COL_RESET printf("\033[0m")

// constants

// global variables

uint8_t uart_buffer[305];
uint16_t uart_buffer_index = 0;
uint8_t do_echo = 1;
uint8_t token_progress = TOKEN_PROGRESS_NONE;
int expected_num = 0;
uint8_t byte_buffer[256];
uint8_t byte_buffer_index = 0;
uint16_t loop_tick = 0;

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
#ifdef MODE_4X4
            // print 4x4
            int conf[32], dist[32];
            if (TMF8828_update(conf, dist)) {
                for (int i = 0; i < 32; i++) {
                    APP_PRINT("nr: %d, conf: %d, dist: %d\n", i, conf[i], dist[i]);
                }
                APP_PRINT("\n");
            }
#else
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
#endif
        }
        sleep_ms(1);
        loop_tick++;
    }
    return 0; // OK for warning on this line (unreachable code)
}
