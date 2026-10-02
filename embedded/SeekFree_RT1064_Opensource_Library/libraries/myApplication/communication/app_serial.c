#include "app_serial.h"

#include <stdint.h>
#include <stddef.h>
#include "drv_io.h"

static app_serial_parser_t rx_parser;

void app_serial_parser_reset(app_serial_parser_t *parser)
{
    if (parser != NULL)
    {
        parser->length = 0U;
    }
}

uint8_t app_serial_feed_byte(app_serial_parser_t *parser, uint8_t byte,
                             app_serial_command_t *command)
{
    uint8_t checksum;
    uint8_t index;
    uint16_t raw_value;
    uint16_t raw_value2;

    if ((parser == NULL) || (command == NULL))
    {
        return 0U;
    }
    if (parser->length >= SERIAL_FRAME_LENGTH)
    {
        parser->length = 0U;
    }
    if (parser->length == 0U)
    {
        if (byte == SERIAL_FRAME_SYNC_0)
        {
            parser->bytes[parser->length++] = byte;
        }
        return 0U;
    }
    if (parser->length == 1U)
    {
        if (byte == SERIAL_FRAME_SYNC_1)
        {
            parser->bytes[parser->length++] = byte;
        }
        else if (byte != SERIAL_FRAME_SYNC_0)
        {
            parser->length = 0U;
        }
        return 0U;
    }

    parser->bytes[parser->length++] = byte;
    if (parser->length < SERIAL_FRAME_LENGTH)
    {
        return 0U;
    }
    parser->length = 0U;
    checksum = 0U;
    for (index = 2U; index < SERIAL_FRAME_LENGTH - 1U; index++)
    {
        checksum ^= parser->bytes[index];
    }
    if (checksum != parser->bytes[SERIAL_FRAME_LENGTH - 1U])
    {
        return 0U;
    }

    command->type = parser->bytes[2];
    if ((command->type != SERIAL_COMMAND_HEARTBEAT) &&
        (command->type != SERIAL_COMMAND_STATE) &&
        (command->type != SERIAL_COMMAND_SPEED) &&
        (command->type != SERIAL_COMMAND_YAW) &&
        (command->type != SERIAL_COMMAND_MOTION))
    {
        return 0U;
    }
    raw_value = (uint16_t)parser->bytes[3] |
                ((uint16_t)parser->bytes[4] << 8U);
    command->value = (raw_value <= INT16_MAX) ? (int16_t)raw_value :
                     (int16_t)((int32_t)raw_value - (int32_t)UINT16_MAX - 1);
    raw_value2 = (uint16_t)parser->bytes[5] |
                 ((uint16_t)parser->bytes[6] << 8U);
    command->value2 = (raw_value2 <= INT16_MAX) ? (int16_t)raw_value2 :
                      (int16_t)((int32_t)raw_value2 - (int32_t)UINT16_MAX - 1);
    if ((command->type != SERIAL_COMMAND_MOTION) &&
        (command->value2 != 0))
    {
        return 0U;
    }
    if ((command->type == SERIAL_COMMAND_STATE) &&
        (command->value != SERIAL_STATE_PROTECT) &&
        (command->value != SERIAL_STATE_BALANCE) &&
        (command->value != SERIAL_STATE_NAVIGATION))
    {
        return 0U;
    }
    return 1U;
}

void app_serial_init(void)
{
    app_serial_parser_reset(&rx_parser);
    drv_io_uart_init();
}

uint8_t app_serial_poll_command(app_serial_command_t *command)
{
    uint8_t byte;

    if (command == NULL)
    {
        return 0U;
    }
    while (drv_io_uart_try_read(&byte) != 0U)
    {
        if (app_serial_feed_byte(&rx_parser, byte, command) != 0U)
        {
            return 1U;
        }
    }
    return 0U;
}

void app_serial_send(void)
{
    static const uint8_t test_frame[] = {0xAAU, 0x55U, 0x01U, 0x00U, 0x01U, 0xFEU};

    drv_io_uart_write(test_frame, sizeof(test_frame));
}
