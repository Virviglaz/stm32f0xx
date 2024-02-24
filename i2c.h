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

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
#endif

typedef void (*isr_handler_t)(void *prv_ptr, uint8_t err);

/* define i2c message operation */
enum i2c_dir {
	I2C_WRITING = 0,
	I2C_READING,
};

/* i2c message is used to define single transfer i2c action */
struct i2c_msg_t {
	enum i2c_dir dir;
	uint8_t *data;
	uint8_t size;
};

/* transfer handles the messages */
struct i2c_xfer {
	uint8_t address;
	struct i2c_msg_t *msgs;
	uint8_t msgs_left;
	isr_handler_t handler;
	void *prv_ptr;
	bool done;
	uint16_t cnt;			/* byte counter for isr mode */
	int error;
};

typedef struct i2c_dev {
	I2C_TypeDef *i2c;
	struct i2c_xfer xfer;
#ifdef FREERTOS
	SemaphoreHandle_t done;
	SemaphoreHandle_t lock;
	int err;
#endif
} i2c_dev_t;

typedef enum i2c_mode { I2C_NORMAL, I2C_FAST } i2c_mode_t;

/**
  * @brief Initiate i2c device.
  *
  * @Note
  * This function helps you to find a proper connection to i2c device
  * if you only know to which pins it is connected to.
  * You should provide a gpio and pinmask of used pins.
  *
  * @param dev			Pointer to i2c device handle.
  * @param gpio			GPIO port where i2c device is connected.
  * @param scl_pin_mask		Bitmask of SCL pin.
  * @param sda_pin_mask		Bitmask of SDA pin.
  * @param mode			I2C_NORMAL (100kHz) of I2C_FAST(400kHz)
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_init(i2c_dev_t *dev,
	     GPIO_TypeDef *gpio,
	     uint16_t scl_pin_mask,
	     uint16_t sda_pin_mask,
	     i2c_mode_t mode);

/**
  * @brief Perform i2c transfer using posix-like message structures.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param msgs			Pointer to message list.
  * @param nof_msgs		Number of messages in transfer.
  * @param handler		Function called when the transfer is completed.
  * @param prv_ptr		Pointer for generic use, passed to callback.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_transfer(i2c_dev_t *dev,
		 uint8_t addr,
		 struct i2c_msg_t *msgs,
		 uint8_t nof_msgs,
		 isr_handler_t handler,
		 void *prv_ptr);

/**
  * @brief Write data to i2c slave device.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param src1			First message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  * @param src1			Second message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_write(i2c_dev_t *dev,
	      uint8_t addr,
	      uint8_t *src1,
	      uint8_t size1,
	      uint8_t *src2,
	      uint8_t size2);

/**
  * @brief Read data from i2c slave device.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param dst1			First message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  * @param dst2			Second message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_read(i2c_dev_t *dev,
	     uint8_t addr,
	     uint8_t *dst1,
	     uint8_t size1,
	     uint8_t *dst2,
	     uint8_t size2);

/**
  * @brief Simple write action to i2c slave device.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param src			Pointer to buffer with data to be sent.
  * @param size			Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_simple_write(i2c_dev_t *dev,
		     uint8_t addr,
		     uint8_t *src,
		     uint16_t size);

/**
  * @brief Simple read action from i2c slave device.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param dst			Pointer to buffer for received data.
  * @param size			Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_simple_read(i2c_dev_t *dev,
		    uint8_t addr,
		    uint8_t *dst,
		    uint16_t size);

/**
  * @brief Write action to i2c slave device register. Typical usecase.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param reg			Slave device 8-bit register number.
  * @param src			Pointer buffer with data to be sent.
  * @param size			Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_write_reg(i2c_dev_t *dev,
		  uint8_t addr,
		  uint8_t reg,
		  uint8_t *src,
		  uint16_t size);

/**
  * @brief Read action from i2c slave device register. Typical usecase.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param reg			Slave device 8-bit register number.
  * @param dst			Pointer destanation buffer.
  * @param size			Amount of bytes to be received.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_read_reg(i2c_dev_t *dev,
		 uint8_t addr,
		 uint8_t reg,
		 uint8_t *dst,
		 uint16_t size);

#ifdef FREERTOS

/**
  * @brief Write data to i2c slave device with FreeRTOS support.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param src1			First message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  * @param src1			Second message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_write_rtos(i2c_dev_t *dev,
		   uint8_t addr,
		   uint8_t *src1,
		   uint8_t size1,
		   uint8_t *src2,
		   uint8_t size2);

/**
  * @brief Read data from i2c slave device with FreeRTOS support.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param dst1			First message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  * @param dst2			Second message buffer pointer.
  * @param size1		Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_read_rtos(i2c_dev_t *dev,
		  uint8_t addr,
		  uint8_t *dst1,
		  uint8_t size1,
		  uint8_t *dst2,
		  uint8_t size2);

/**
  * @brief Write action to i2c slave device with FreeRTOS support.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param src			Pointer to buffer with data to be sent.
  * @param size			Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_simple_write_rtos(i2c_dev_t *dev,
			  uint8_t addr,
			  uint8_t *src,
			  uint16_t size);

/**
  * @brief Read action from i2c slave device with FreeRTOS support.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param dst			Pointer destanation buffer.
  * @param size			Amount of bytes to be received.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_simple_read_rtos(i2c_dev_t *dev,
			  uint8_t addr,
			  uint8_t *src,
			  uint16_t size);

/**
  * @brief Write action to i2c slave device register with FreeRTOS support.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param reg			Slave device 8-bit register number.
  * @param src			Pointer to buffer with data to be sent.
  * @param size			Amount of bytes to be sent.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_write_reg_rtos(i2c_dev_t *dev,
		       uint8_t addr,
		       uint8_t reg,
		       uint8_t *dst,
		       uint16_t size);

/**
  * @brief Read action from i2c slave device register with FreeRTOS support.
  *
  * @param dev			Pointer to i2c device handle.
  * @param addr			7-bit address of i2c slave device.
  * @param reg			Slave device 8-bit register number.
  * @param dst			Pointer destanation buffer.
  * @param size			Amount of bytes to be received.
  *
  * @retval 0 is success, error code if failed.
  */
int i2c_read_reg_rtos(i2c_dev_t *dev,
		      uint8_t addr,
		      uint8_t reg,
		      uint8_t *src,
		      uint16_t size);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
