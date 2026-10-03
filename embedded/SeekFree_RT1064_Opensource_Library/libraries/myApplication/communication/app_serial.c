#include "app_serial.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "drv_io.h"

static app_serial_parser_t rx_parser;
static uint8_t telemetry_sequence;

static void put_u16(uint8_t *frame, uint8_t offset, uint16_t value)
{
    frame[offset] = (uint8_t)value;
    frame[offset + 1U] = (uint8_t)(value >> 8U);
}

static void put_scaled(uint8_t *frame, uint8_t offset, float value, float scale)
{
    float scaled = value * scale;
    int32_t rounded;

    if (!isfinite(scaled))
    {
        scaled = 0.0f;
    }
    if (scaled > (float)INT16_MAX)
    {
        scaled = (float)INT16_MAX;
    }
    if (scaled < (float)INT16_MIN)
    {
        scaled = (float)INT16_MIN;
    }
    rounded = (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
    put_u16(frame, offset, (uint16_t)(int16_t)rounded);
}

uint8_t app_serial_encode_telemetry(const app_serial_telemetry_t *data,
                                    uint8_t sequence,
                                    uint8_t frame[SERIAL_TELEMETRY_LENGTH])
{
    uint16_t crc = SERIAL_TELEMETRY_CRC_INIT;
    uint8_t index;
    uint8_t bit;

    if ((data == NULL) || (frame == NULL))
    {
        return 0U;
    }
    frame[0] = SERIAL_TELEMETRY_SYNC_0;
    frame[1] = SERIAL_TELEMETRY_SYNC_1;
    frame[2] = SERIAL_TELEMETRY_VERSION;
    frame[3] = SERIAL_TELEMETRY_TYPE;
    frame[4] = sequence;
    frame[5] = data->valid;
    frame[6] = (uint8_t)data->uptime_ms;
    frame[7] = (uint8_t)(data->uptime_ms >> 8U);
    frame[8] = (uint8_t)(data->uptime_ms >> 16U);
    frame[9] = (uint8_t)(data->uptime_ms >> 24U);
    frame[10] = data->control_mode;
    frame[11] = data->vehicle_mode;
    put_scaled(frame, 12U, data->roll_deg, SERIAL_ANGLE_UNITS_PER_DEG);
    put_scaled(frame, 14U, data->pitch_deg, SERIAL_ANGLE_UNITS_PER_DEG);
    put_scaled(frame, 16U, data->yaw_deg, SERIAL_ANGLE_UNITS_PER_DEG);
    put_scaled(frame, 18U, data->pitch_rate_dps, SERIAL_RATE_UNITS_PER_DPS);
    put_scaled(frame, 20U, data->gyro_x_dps, SERIAL_RATE_UNITS_PER_DPS);
    put_scaled(frame, 22U, data->gyro_y_dps, SERIAL_RATE_UNITS_PER_DPS);
    put_scaled(frame, 24U, data->gyro_z_dps, SERIAL_RATE_UNITS_PER_DPS);
    put_scaled(frame, 26U, data->acc_x_g, SERIAL_ACCEL_UNITS_PER_G);
    put_scaled(frame, 28U, data->acc_y_g, SERIAL_ACCEL_UNITS_PER_G);
    put_scaled(frame, 30U, data->acc_z_g, SERIAL_ACCEL_UNITS_PER_G);
    put_scaled(frame, 32U, data->left_speed_m_s, SERIAL_SPEED_UNITS_PER_MPS);
    put_scaled(frame, 34U, data->right_speed_m_s, SERIAL_SPEED_UNITS_PER_MPS);
    put_u16(frame, 36U, (uint16_t)data->left_delta_count);
    put_u16(frame, 38U, (uint16_t)data->right_delta_count);
    put_u16(frame, 40U, (uint16_t)data->left_pwm);
    put_u16(frame, 42U, (uint16_t)data->right_pwm);
    put_scaled(frame, 44U, data->target_speed_m_s, SERIAL_SPEED_UNITS_PER_MPS);
    put_scaled(frame, 46U, data->target_yaw_deg, SERIAL_ANGLE_UNITS_PER_DEG);
    put_scaled(frame, 48U, data->target_pitch_deg, SERIAL_ANGLE_UNITS_PER_DEG);
    for (index = 0U; index < SERIAL_TELEMETRY_LENGTH - 2U; index++)
    {
        crc ^= (uint16_t)frame[index] << 8U;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ?
                (uint16_t)((crc << 1U) ^ SERIAL_TELEMETRY_CRC_POLY) :
                (uint16_t)(crc << 1U);
        }
    }
    put_u16(frame, SERIAL_TELEMETRY_LENGTH - 2U, crc);
    return 1U;
}

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
        (command->type != SERIAL_COMMAND_MOTION) &&
        ((command->type < SERIAL_COMMAND_PID_BASE) ||
         (command->type > SERIAL_COMMAND_PID_LAST)))
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
    command->pid_controller = 0U;
    command->pid_parameter = 0U;
    command->pid_value = 0.0f;
    if ((command->type >= SERIAL_COMMAND_PID_BASE) &&
        (command->type <= SERIAL_COMMAND_PID_LAST))
    {
        uint8_t pid_index = (uint8_t)(command->type - SERIAL_COMMAND_PID_BASE);
        memcpy(&command->pid_value, &parser->bytes[3], sizeof(command->pid_value));
        command->pid_controller = (uint8_t)(pid_index / 3U);
        command->pid_parameter = (uint8_t)(pid_index % 3U);
        if (!isfinite(command->pid_value) ||
            (command->pid_value < 0.0f) ||
            (command->pid_value > SERIAL_PID_MAX_VALUE))
        {
            return 0U;
        }
        return 1U;
    }
    if ((command->type != SERIAL_COMMAND_MOTION) &&
        (command->value2 != 0))
    {
        return 0U;
    }
    if ((command->type == SERIAL_COMMAND_STATE) &&
        (command->value != SERIAL_STATE_PROTECT) &&
        (command->value != SERIAL_STATE_BALANCE) &&
        (command->value != SERIAL_STATE_NAVIGATION) &&
        (command->value != SERIAL_STATE_SUPPORT))
    {
        return 0U;
    }
    return 1U;
}

void app_serial_init(void)
{
    app_serial_parser_reset(&rx_parser);
    telemetry_sequence = 0U;
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

void app_serial_send(const app_serial_telemetry_t *telemetry)
{
    uint8_t frame[SERIAL_TELEMETRY_LENGTH];

    if (app_serial_encode_telemetry(telemetry, telemetry_sequence, frame) != 0U)
    {
        drv_io_uart_write(frame, sizeof(frame));
        telemetry_sequence++;
    }
}
