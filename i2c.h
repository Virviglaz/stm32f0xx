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

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f0xx.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_SUCCESS	0
#define I2C_ERR_NOACK	1
#define I2C_ERR_ARLO	2

/* spi types redefinition */
typedef const struct i2c_dev_t *i2c_dev;

/**
  * @brief  Lookup for a gpio settings to configure the i2c.
  *
  * This function helps you to find a proper connection to i2c if you only
  * knows the where it is used. You should provide a gpio and any of used pins.
  * @param  gpio: GPIO where one of the SDC/SDA pins are connected to.
  * @param  pin_mask: pin mask of tx or rx pin.
  * @param  fast_mode: 400kHz if true, 100kHz if false.
  *
  * @retval 0 if no settings found or a pointer to struct if success.
  */
i2c_dev find_i2c_dev(GPIO_TypeDef *gpio, uint16_t pin_mask, bool fast_mode);

/**
  * @brief  Get i2c by index.
  *
  * @param  num: index of i2c [1 or 2] depends of device choosen.
  * @param  fast_mode: 400kHz if true, 100kHz if false.
  * @note   you can use find_i2c_dev or get_i2c_dev to get a i2c device pointer.
  *
  * @retval 0 if no settings found or a pointer to struct if success.
  */
i2c_dev get_i2c_dev(uint8_t num, bool fast_mode);

/**
  * @brief  Transfer the data buffer to reg.
  *
  * @param  dev: device pointer.
  * @param  addr: device i2c address
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to transfer
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_write_reg(i2c_dev dev, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size);

/**
  * @brief  Receives the data to buffer starting from reg.
  *
  * @param  dev: device pointer.
  * @param  addr: device i2c address
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to receive
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_read_reg(i2c_dev dev, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size);

#ifdef FREERTOS
/**
  * @brief  Transfer the data buffer to reg using RTOS.
  *
  * @param  dev: device pointer.
  * @param  addr: device i2c address
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to transfer
  * @param  timeout: timeout in [ms]
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_write_reg_rtos(i2c_dev dev,
		       uint8_t addr,
		       uint8_t reg,
		       uint8_t *data,
		       uint16_t size,
		       uint32_t timeout_ms);

/**
  * @brief  Receives the data to buffer starting from reg using RTOS.
  *
  * @param  dev: device pointer.
  * @param  addr: device i2c address
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to receive
  * @param  timeout: timeout in [ms]
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_read_reg_rtos(i2c_dev dev,
		      uint8_t addr,
		      uint8_t reg,
		      uint8_t *data,
		      uint16_t size,
		      uint32_t timeout_ms);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
