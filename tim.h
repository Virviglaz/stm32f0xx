/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2020-2024 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * STM32F0xx open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f0xx.h>
#include <stdint.h>

/* tim types redefenition */
typedef const struct tim_dev_t * tim_dev;

/**
  * @brief Initialize the timer and return pointer to device.
  *
  * @param tim		Number of timer.
  * @param freq		Timer frequency in MHz (0 if period is used).
  * @param sec		Timer period in sec.
  *
  * @retval Pointer to device or 0 if failed.
  */
tim_dev get_tim_dev(uint8_t tim, uint32_t freq, uint32_t period);

/**
  * @brief Set timer pre-scaler and period value.
  *
  * @param dev		Pointer to timer device.
  * @param prc		Pre-scaler value.
  * @param period	Period value.
  *
  * @retval None.
  */
void tim_set_timebase(tim_dev dev, uint32_t prc, uint32_t period);

/**
  * @brief  Install and enable the timer interrupt.
  *
  * @param dev		Pointer to timer device.
  * @param handler	Pointer to handler function.
  * @param data		Private data for generic use.
  *
  * @retval None.
  */
void tim_enable_interrupt(tim_dev dev, void (*handler)(void *data), void *data);

/**
  * @brief Disable the timer interrupt.
  *
  * @param dev		Pointer to timer device.
  *
  * @retval None.
  */
void tim_disable_interrupt(tim_dev dev);

/**
  * @brief Start the timer.
  *
  * @param dev		Pointer to timer device.
  *
  * @retval None.
  */
void tim_start(tim_dev dev);

/**
  * @brief Stop the timer.
  *
  * @param dev		Pointer to timer device.
  *
  * @retval None.
  */
void tim_stop(tim_dev dev);

/**
  * @brief Get timer counter value.
  *
  * @param dev		Pointer to timer device.
  *
  * @retval Counter value.
  */
uint32_t tim_get_value(tim_dev dev);

/**
  * @brief Set timer counter value.
  *
  * @param dev		Pointer to timer device.
  * @param value	New counter value.
  *
  * @retval None.
  */
void tim_set_value(tim_dev dev, uint32_t value);

/**
  * @brief Enable PWM output.
  *
  * @param dev		Pointer to timer device.
  * @param ch_num	Channel number [1..4].
  * @param duty		Duty cycle [0..period].
  *
  * @retval 0 if success, EINVAL if no channel found for this timer.
  */
int tim_pwm_enable(tim_dev dev, uint8_t ch_num, uint16_t duty);

/**
  * @brief  Set the PWM duty cycle.
  *
  * @param  dev		Pointer to timer device.
  * @param  ch_num	Channel number [1..4].
  * @param  duty	Duty cycle [0..period].
  *
  * @retval 0 if success.
  */
int tim_pwm_set_duty(tim_dev dev, uint8_t ch_num, uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
