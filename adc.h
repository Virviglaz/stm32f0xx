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

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f0xx.h>
#include <stdint.h>

/* spi types redefinition */
typedef const struct adc_dev_t *adc_dev;

/**
  * @brief  Get adc channel by gpio/pin.
  * @param  gpio	gpio base where channel is connected.
  * @param  pin		pin_number where channel is connected [0..15].
  *
  * @retval 0 if no settings found or a pointer to struct if success.
  */
adc_dev find_adc_dev(GPIO_TypeDef *gpio, uint8_t pin);

/**
  * @brief  Get adc channel by index.
  * @param  num		index of ADC channel [0..16].
  *
  * @retval 0 if no settings found or a pointer to struct if success.
  */
adc_dev get_adc_dev(uint8_t num);

/**
  * @brief  Convert and read the analog value.
  * @param  dev		adc channel device.
  * @param  sample_rate	desired sample rate [0..7].
  *
  * @retval 0 if no settings found or a pointer to struct if success.
  */
uint16_t adc_read(adc_dev dev, uint8_t sample_rate);

/**
  * @brief  Read reference voltage.
  *
  * @retval voltage in [V].
  */
double adc_read_vref(void);

/**
  * @brief  Read internal temperature value in Celcius.
  *
  * @retval internal sensor temperature value.
  */
double adc_read_temp(void);

/**
  * @brief  Read real voltage from analog input.
  *
  * @param dev		Pointer to ADC device.
  * @retval voltage in [V].
  */
double adc_read_voltage(adc_dev dev);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
