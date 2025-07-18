/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * motors.c - Motor driver
 *
 */

#include <stdbool.h>

//FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"
#include "platform.h"
#include "crtp.h"

extern void platform_motors_pwm_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};
uint32_t motor_ratios[] = {0, 0, 0, 0};
const MotorPerifDef **motorMap;

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

// This is the command set by the stabilizer
static uint16_t motorCmd[MOTOR_COUNT];
static bool isInit = false;
static bool isTimerInit = false;

ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = MOT_PWM_CH1,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH2,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH3,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH4,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};
/* Private functions */

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

bool pwm_timmer_init()
{
    if (isTimerInit) {
        // First to init will configure it
        return TRUE;
    }

    /*
     * Prepare and set configuration of timers
     * that will be used by MOTORS Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = MOTORS_PWM_BITS, // resolution of PWM duty
        .freq_hz = 50,				// frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .timer_num = LEDC_TIMER_0,			// timer index
        // .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    DEBUG_PRINTI("Set PWM freq to 50Hz for brushless");

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = TRUE;
        return TRUE;
    }

    return FALSE;
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    int i;

    if (isInit) {
        // First to init will configure it
        return;
    }

    motorMap = motorMapSelect;

    if (pwm_timmer_init() != TRUE) {
        return;
    }

    for (i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_channel_config(&motors_channel[i]);
    }

    isInit = true;
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}

bool motorsTest(void)
{
  vTaskDelay(M2T(1000));
  
  DEBUG_PRINTI("motor test starting...\n");
  // For BLHeli 20A ESC 
  // for more info go to docs --blhelis github page
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_10_BIT,  
      .freq_hz = 24000,  
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_1,
  };
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channel = {
      .channel = LEDC_CHANNEL_0,
      .duty = 0,
      .gpio_num = 18,  
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_sel = LEDC_TIMER_1
  };
  ledc_channel_config(&ledc_channel);
  
  // BLHeli standard: 1000us = min, 2000us = max
  // At 24kHz: period = 41.67us, so 1000us = 24 ticks, 2000us = 48 ticks
  // For 10-bit (1024 max): 1000us = 24/41.67 * 1024 = 590, 2000us = 1180
  const uint16_t escMin = 590;  
  const uint16_t escMax = 1180;  
  
  DEBUG_PRINTI("pwm setup done, min=%u max=%u\n", escMin, escMax);
  
  DEBUG_PRINTI("arming esc...\n");
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, escMin);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  vTaskDelay(M2T(8000));
  
  DEBUG_PRINTI("esc should be calibrated now\n");
  vTaskDelay(M2T(2000)); 
   
  DEBUG_PRINTI("checking if motor can spin freely...\n");
  vTaskDelay(M2T(5000));  
  
  DEBUG_PRINTI("testing motor spin\n");
  
  DEBUG_PRINTI("70%% throttle\n");
  uint16_t spinThrottle = escMin + ((escMax - escMin) * 7 / 10); 
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, spinThrottle);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  vTaskDelay(M2T(4000));
  
  DEBUG_PRINTI("90%% throttle\n");
  spinThrottle = escMin + ((escMax - escMin) * 9 / 10); 
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, spinThrottle);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  vTaskDelay(M2T(4000));
  
  DEBUG_PRINTI("back to idle\n");
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, escMin);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  
  DEBUG_PRINTI("motor test done\n");
  
  while(1) { vTaskDelay(M2T(1000)); }

  return isInit;
}

void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit) {
        uint16_t ratio;

        ASSERT(id < NBR_OF_MOTORS);

        ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED

        if (motorMap[id]->drvType == BRUSHED) {
            float thrust = ((float)ithrust / 65536.0f) * 40; //根据实际重量修改
            float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
            float supply_voltage = pmGetBatteryVoltage();
            float percentage = volts / supply_voltage;
            percentage = percentage > 1.0f ? 1.0f : percentage;
            ratio = percentage * UINT16_MAX;
            motor_ratios[id] = ratio;
        }

#endif
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
        motor_ratios[id] = ratio;
#ifdef DEBUG_EP2
        DEBUG_PRINT_LOCAL("motors ID = %d ,ithrust_10bit = %d", id, (uint32_t)motorsConv16ToBits(ratio));
#endif
    }
}

int motorsGetRatio(uint32_t id)
{
    int ratio;
    ASSERT(id < NBR_OF_MOTORS);
    ratio = motorsConvBitsTo16((uint16_t)ledc_get_duty(motors_channel[id].speed_mode, motors_channel[id].channel));
    return ratio;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    uint32_t freq_hz = 15000;
    ASSERT(id < NBR_OF_MOTORS);
    if (ratio != 0) {
        ratio = (uint16_t)(0.05*(1<<16));
    }
    
    if (enable) {
        freq_hz = frequency;
    }
    
    ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_0,freq_hz);
    ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
    ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
}

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    vTaskDelay(M2T(duration_msec));
    motorsBeep(MOTOR_M1, false, frequency, 0);
    motorsBeep(MOTOR_M2, false, frequency, 0);
    motorsBeep(MOTOR_M3, false, frequency, 0);
    motorsBeep(MOTOR_M4, false, frequency, 0);
}

void motorsPlayMelody(uint16_t *notes)
{
    int i = 0;
    uint16_t note;    
    uint16_t duration; 

    do
    {
      note = notes[i++];
      duration = notes[i++];
      motorsPlayTone(note, duration);
    } while (duration != 0);
}
LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
