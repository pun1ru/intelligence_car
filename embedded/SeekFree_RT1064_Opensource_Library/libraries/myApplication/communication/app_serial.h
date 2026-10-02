#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>
#include "general_define.h"

typedef struct
{
    uint8_t bytes[SERIAL_FRAME_LENGTH];
    uint8_t length;
} app_serial_parser_t;

typedef struct
{
    uint8_t type;
    int16_t value;
    int16_t value2;
} app_serial_command_t;

void app_serial_init(void);
void app_serial_send(void);
void app_serial_parser_reset(app_serial_parser_t *parser);
uint8_t app_serial_feed_byte(app_serial_parser_t *parser, uint8_t byte,
                             app_serial_command_t *command);
uint8_t app_serial_poll_command(app_serial_command_t *command);

#endif
