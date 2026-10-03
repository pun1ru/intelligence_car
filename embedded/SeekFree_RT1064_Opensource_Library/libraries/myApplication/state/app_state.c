#include "app_state.h"

#include <math.h>
#include <stddef.h>
#include "drv_io.h"
#include "general_define.h"

static uint32_t saturating_add(uint32_t value, uint32_t increment)
{
    return (UINT32_MAX - value < increment) ? UINT32_MAX : value + increment;
}

static float clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static uint8_t button_press_event(app_state_t *state, uint8_t index,
                                  uint8_t pressed, uint32_t dt_ms)
{
    uint8_t raw = (pressed != 0U) ? 1U : 0U;

    if (raw != state->button_raw[index])
    {
        state->button_raw[index] = raw;
        state->button_age_ms[index] = 0U;
    }
    else
    {
        state->button_age_ms[index] = saturating_add(
            state->button_age_ms[index], dt_ms);
    }
    if ((state->button_age_ms[index] >= VEHICLE_BUTTON_DEBOUNCE_MS) &&
        (state->button_stable[index] != raw))
    {
        state->button_stable[index] = raw;
        return raw;
    }
    return 0U;
}

static void enter_protection(app_state_t *state)
{
    state->vehicle_mode = APP_VEHICLE_PROTECT;
    state->target_speed_m_s = 0.0f;
    state->target_yaw_deg = 0.0f;
    state->balance_entry_kick_age_ms = 0U;
    state->balance_entry_kick_active = 0U;
}

void app_state_force_protection(app_state_t *state)
{
    if (state != NULL)
    {
        enter_protection(state);
    }
}

void app_state_init(app_state_t *state)
{
    if (state != NULL)
    {
        *state = (app_state_t){0};
        state->control_mode = APP_STATE_BUTTON_DEBUG;
        state->vehicle_mode = APP_VEHICLE_PROTECT;
        state->serial_idle_ms = VEHICLE_SERIAL_TIMEOUT_MS;
    }
}

void app_state_read_buttons(app_state_buttons_t *buttons)
{
    if (buttons != NULL)
    {
        buttons->c12_pressed = drv_io_button_pressed(DRV_IO_BUTTON_C12);
        buttons->c13_pressed = drv_io_button_pressed(DRV_IO_BUTTON_C13);
        buttons->c14_pressed = drv_io_button_pressed(DRV_IO_BUTTON_C14);
        buttons->c15_pressed = drv_io_button_pressed(DRV_IO_BUTTON_C15);
        buttons->switches_on = drv_io_calibration_switches_on();
        buttons->switches_off = drv_io_calibration_switches_off();
    }
}

void app_state_step(app_state_t *state, const app_state_buttons_t *buttons,
                    uint8_t attitude_valid, float pitch_deg, uint32_t dt_ms,
                    const app_serial_command_t *command)
{
    uint8_t nav_pressed;
    uint8_t calibrate_pressed;
    uint8_t arm_pressed;
    uint8_t protect_pressed;
    uint8_t fallen;
    uint8_t can_arm;
    uint8_t support_can_arm;
    uint8_t support_pitch_in_range;
    uint8_t link_lost = 0U;

    if ((state == NULL) || (buttons == NULL))
    {
        return;
    }
    state->calibrate_requested = 0U;
    state->reset_ekf_requested = 0U;
    state->pid_update_requested = 0U;
    if ((command != NULL) &&
        (command->type != SERIAL_COMMAND_HEARTBEAT) &&
        (command->type != SERIAL_COMMAND_STATE) &&
        (command->type != SERIAL_COMMAND_SPEED) &&
        (command->type != SERIAL_COMMAND_YAW) &&
        (command->type != SERIAL_COMMAND_MOTION) &&
        ((command->type < SERIAL_COMMAND_PID_BASE) ||
         (command->type > SERIAL_COMMAND_PID_LAST)))
    {
        command = NULL;
    }

    nav_pressed = button_press_event(state, 0U, buttons->c13_pressed, dt_ms);
    arm_pressed = button_press_event(state, 1U, buttons->c14_pressed, dt_ms);
    protect_pressed = button_press_event(state, 2U, buttons->c15_pressed, dt_ms);
    calibrate_pressed = button_press_event(state, 3U, buttons->c12_pressed, dt_ms);

    if (buttons->switches_on != 0U)
    {
        state->support_pitch_age_ms = 0U;
        state->control_mode = APP_STATE_CALIBRATION;
        enter_protection(state);
        state->target_yaw_deg = 0.0f;
        state->calibrate_requested = calibrate_pressed;
        state->reset_ekf_requested = nav_pressed;
        return;
    }
    if (state->control_mode == APP_STATE_CALIBRATION)
    {
        state->control_mode = APP_STATE_BUTTON_DEBUG;
        state->serial_idle_ms = VEHICLE_SERIAL_TIMEOUT_MS;
    }
    if (buttons->switches_off == 0U)
    {
        state->support_pitch_age_ms = 0U;
        state->support_exit_age_ms = 0U;
        enter_protection(state);
        return;
    }

    support_pitch_in_range = ((state->vehicle_mode == APP_VEHICLE_BALANCE) &&
                              (attitude_valid != 0U) &&
                              isfinite(pitch_deg) &&
                              (pitch_deg >= VEHICLE_SUPPORT_AUTO_MIN_PITCH_DEG) &&
                              (pitch_deg <= VEHICLE_SUPPORT_AUTO_MAX_PITCH_DEG)) ?
                             1U : 0U;
    if (support_pitch_in_range != 0U)
    {
        state->support_pitch_age_ms = saturating_add(
            state->support_pitch_age_ms, dt_ms);
    }
    else
    {
        state->support_pitch_age_ms = 0U;
    }

    if ((state->vehicle_mode == APP_VEHICLE_SUPPORT) &&
        (attitude_valid != 0U) && isfinite(pitch_deg) &&
        (pitch_deg >= VEHICLE_SUPPORT_EXIT_PITCH_DEG))
    {
        state->support_exit_age_ms = saturating_add(
            state->support_exit_age_ms, dt_ms);
    }
    else
    {
        state->support_exit_age_ms = 0U;
    }

    /* A kick belongs to the support-to-balance transition. Support mode
     * itself remains stationary until the exit pitch condition is met. */
    if (state->balance_entry_kick_active != 0U)
    {
        state->balance_entry_kick_age_ms = saturating_add(
            state->balance_entry_kick_age_ms, dt_ms);
        if (state->balance_entry_kick_age_ms <
            VEHICLE_BALANCE_ENTRY_KICK_MS)
        {
            state->target_speed_m_s = VEHICLE_BALANCE_ENTRY_KICK_SPEED_MPS;
        }
        else
        {
            state->target_speed_m_s = 0.0f;
            state->balance_entry_kick_active = 0U;
        }
    }
    else if (state->vehicle_mode == APP_VEHICLE_SUPPORT)
    {
        state->target_speed_m_s = 0.0f;
    }

    fallen = ((attitude_valid == 0U) || !isfinite(pitch_deg) ||
              (fabsf(pitch_deg - BALANCE_UPRIGHT_PITCH_DEG) >
               VEHICLE_FALL_PITCH_MAX_DEG)) ? 1U : 0U;
    can_arm = ((fallen == 0U) &&
               (fabsf(pitch_deg - BALANCE_UPRIGHT_PITCH_DEG) <=
                VEHICLE_ARM_PITCH_MAX_DEG)) ? 1U : 0U;
    support_can_arm = ((fallen == 0U) &&
                       (fabsf(pitch_deg - BALANCE_SUPPORT_TARGET_PITCH_DEG) <=
                        VEHICLE_SUPPORT_ARM_PITCH_MAX_DEG)) ? 1U : 0U;

    if (command != NULL)
    {
        state->control_mode = APP_STATE_SERIAL_DEBUG;
        state->serial_idle_ms = 0U;
    }
    else if (state->control_mode == APP_STATE_SERIAL_DEBUG)
    {
        state->serial_idle_ms = saturating_add(state->serial_idle_ms, dt_ms);
        if (state->serial_idle_ms >= VEHICLE_SERIAL_TIMEOUT_MS)
        {
            state->control_mode = APP_STATE_BUTTON_DEBUG;
            enter_protection(state);
            link_lost = 1U;
        }
    }

    if ((command != NULL) &&
        (command->type >= SERIAL_COMMAND_PID_BASE) &&
        (command->type <= SERIAL_COMMAND_PID_LAST))
    {
        state->pid_update_requested = 1U;
        state->pid_controller = command->pid_controller;
        state->pid_parameter = command->pid_parameter;
        state->pid_value = command->pid_value;
        enter_protection(state);
        return;
    }

    if ((fallen != 0U) || (protect_pressed != 0U) ||
        (state->button_stable[2] != 0U) || (link_lost != 0U))
    {
        if ((protect_pressed != 0U) || (state->button_stable[2] != 0U))
        {
            state->support_pitch_age_ms = 0U;
        }
        enter_protection(state);
        return;
    }

    if ((state->vehicle_mode == APP_VEHICLE_SUPPORT) &&
        (state->support_exit_age_ms >= VEHICLE_SUPPORT_EXIT_MS) &&
        !((command != NULL) && (command->type == SERIAL_COMMAND_STATE) &&
          (command->value == SERIAL_STATE_PROTECT)))
    {
        state->vehicle_mode = APP_VEHICLE_BALANCE;
        state->target_speed_m_s = VEHICLE_BALANCE_ENTRY_KICK_SPEED_MPS;
        state->target_yaw_deg = 0.0f;
        state->support_exit_age_ms = 0U;
        state->balance_entry_kick_age_ms = 0U;
        state->balance_entry_kick_active = 1U;
        return;
    }

    if ((state->support_pitch_age_ms >= VEHICLE_SUPPORT_AUTO_ENTER_MS) &&
        (state->vehicle_mode == APP_VEHICLE_BALANCE) &&
        !((command != NULL) && (command->type == SERIAL_COMMAND_STATE) &&
          (command->value == SERIAL_STATE_PROTECT)))
    {
        state->vehicle_mode = APP_VEHICLE_SUPPORT;
        state->target_speed_m_s = 0.0f;
        state->target_yaw_deg = 0.0f;
        return;
    }

    if ((state->control_mode == APP_STATE_BUTTON_DEBUG) && (can_arm != 0U))
    {
        if (arm_pressed != 0U)
        {
            state->vehicle_mode = APP_VEHICLE_BALANCE;
            state->target_speed_m_s = 0.0f;
        }
        else if (nav_pressed != 0U)
        {
            state->vehicle_mode = APP_VEHICLE_NAVIGATION;
            state->target_speed_m_s = 0.0f;
        }
    }
    if ((state->control_mode != APP_STATE_SERIAL_DEBUG) || (command == NULL))
    {
        return;
    }

    if (command->type == SERIAL_COMMAND_STATE)
    {
        if (command->value == SERIAL_STATE_PROTECT)
        {
            enter_protection(state);
        }
        else if ((can_arm != 0U) &&
                 ((command->value == SERIAL_STATE_BALANCE) ||
                  (command->value == SERIAL_STATE_NAVIGATION)))
        {
            state->vehicle_mode = (command->value == SERIAL_STATE_BALANCE) ?
                APP_VEHICLE_BALANCE : APP_VEHICLE_NAVIGATION;
            state->target_speed_m_s = 0.0f;
        }
        else if ((support_can_arm != 0U) &&
                 (command->value == SERIAL_STATE_SUPPORT))
        {
            state->vehicle_mode = APP_VEHICLE_SUPPORT;
            state->target_speed_m_s = 0.0f;
        }
    }
    else if ((command->type == SERIAL_COMMAND_SPEED) &&
             ((state->vehicle_mode == APP_VEHICLE_BALANCE) ||
              (state->vehicle_mode == APP_VEHICLE_SUPPORT)))
    {
        state->target_speed_m_s = clamp(
            (float)command->value * SERIAL_SPEED_SCALE_MPS,
            BALANCE_SPEED_TARGET_MAX_MPS);
    }
    else if (((command->type == SERIAL_COMMAND_YAW) ||
              (command->type == SERIAL_COMMAND_MOTION)) &&
             ((state->vehicle_mode == APP_VEHICLE_BALANCE) ||
              (state->vehicle_mode == APP_VEHICLE_SUPPORT)))
    {
        state->target_yaw_deg = (float)((command->type == SERIAL_COMMAND_MOTION) ?
            command->value2 : command->value) * SERIAL_YAW_SCALE_DEG;
        if (command->type == SERIAL_COMMAND_MOTION)
        {
            state->target_speed_m_s = clamp(
                (float)command->value * SERIAL_SPEED_SCALE_MPS,
                BALANCE_SPEED_TARGET_MAX_MPS);
        }
    }
}

uint8_t app_state_balance_enabled(const app_state_t *state)
{
    return ((state != NULL) &&
            ((state->vehicle_mode == APP_VEHICLE_BALANCE) ||
             (state->vehicle_mode == APP_VEHICLE_NAVIGATION) ||
             (state->vehicle_mode == APP_VEHICLE_SUPPORT))) ? 1U : 0U;
}
