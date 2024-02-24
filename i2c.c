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

#include "rcc.h"
#include "i2c.h"
#include "gpio.h"
#include <errno.h>
#include <stdbool.h>

#define NOF_DEVICES			2 /* I2C1 & I2C2 */

#define STRUCT2VAL(x)			(*(uint32_t *)&x)

static i2c_dev_t *devs[NOF_DEVICES]; /* local storage for isr routing */

static const struct i2c_periph_dev {
	uint8_t index;			/* index of i2c i.e. 1..2 */
	I2C_TypeDef *i2c;		/* pointer to i2c base address */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint16_t scl;			/* MSCK pin bit mask BIT(x) */
	uint16_t sda;			/* MOSI pin bit mask BIT(x) */
	enum gpio_alt_t alt_func;	/* alt function index GPIO_AF0..7 */
} devices[] = {
#if defined (STM32F030xC) || defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, I2C1, GPIOA, BIT(9),  BIT(10), GPIO_AF4 },
#endif
	{ 1, I2C1, GPIOB, BIT(6),  BIT(7),  GPIO_AF1 },
	{ 1, I2C1, GPIOB, BIT(9),  BIT(10), GPIO_AF1 },
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, I2C1, GPIOB, BIT(10), BIT(11), GPIO_AF1 },
	{ 1, I2C1, GPIOF, BIT(6),  BIT(7),  GPIO_AF0 },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 2, I2C2, GPIOB, BIT(10), BIT(11), GPIO_AF1 },
#endif
#if defined (STM32F030xC)
	{ 2, I2C2, GPIOB, BIT(13), BIT(14), GPIO_AF5 },
	{ 1, I2C1, GPIOF, BIT(0),  BIT(1),  GPIO_AF1 },
#endif
};

struct cr2_reg {
	uint8_t saddr0		: 1; /* Slave address bit		[0]  */
	uint8_t saddr		: 7; /* Slave address bit	       [7:1] */
	uint8_t saddr98		: 2; /* Slave address bit	       [9:8] */
	enum i2c_dir dir	: 1; /* Transfer direction		[10] */
	bool addr_10		: 1; /* 10-bit addressing mode		[11] */
	uint16_t addr_ho	: 1; /* 10-bit address header		[12] */
	bool start		: 1; /* Start generation		[13] */
	bool stop		: 1; /* Stop generation			[14] */
	bool			: 1; /* NACK generation (slave mode) 	[15] */
	uint8_t size		: 8; /* Number of bytes		     [23:16] */
	bool reload		: 1; /* Reload mode			[24] */
	bool autocomplete	: 1; /* Automatic end mode		[25] */
	bool pec		: 1; /* Packet error checking byte	[26] */
				/* 	Reserved		     [31:27] */
} __attribute__((__packed__));

static inline uint32_t get_timings(i2c_mode_t mode)
{
	struct system_clock_t *clock = get_clocks(); /* TODO: what for? */
	const struct {
		uint8_t scll	: 8;	/* [0:7] */
		uint8_t sclh	: 8;	/* [15:8] */
		uint8_t sdadel	: 4;	/* [19:16] */
		uint8_t scldel	: 4;	/* [23:20] */
		uint8_t 	: 4;	/* [24:27] */
		uint8_t presc	: 4;	/* [31:28] */
	} t = {
		.scll = 0x13,
		.sclh = 0x0F,
		.sdadel = 0x02,
		.scldel = 0x04,
		.presc = mode == I2C_FAST ? 0 : 1,
	};

	return STRUCT2VAL(t);
}

static void initiate(i2c_dev_t *dev,
		     uint8_t addr,
		     enum i2c_dir dir,
		     bool start,
		     uint8_t nbytes)
{
	const struct cr2_reg cr2 = {
		.start = start,
		.saddr = addr,
		.size = dir == I2C_READING ? nbytes : 0xFF,
		.dir = dir,
	};

	dev->i2c->CR2 = STRUCT2VAL(cr2);
}

static void stop(i2c_dev_t *dev)
{
	const struct cr2_reg cr2 = { .stop = true };

	dev->i2c->CR2 = STRUCT2VAL(cr2);
}

static void finish_xfer(i2c_dev_t *dev, uint8_t err)
{
	dev->i2c->CR1 = 0;

	if (dev->xfer.handler)
		dev->xfer.handler(dev->xfer.prv_ptr, err);
}

static int send_message(i2c_dev_t *dev,
			uint8_t addr,
			uint8_t *reg,
			uint8_t reg_size,
			enum i2c_dir dir1,
			uint8_t *buf,
			uint8_t size,
			enum i2c_dir dir2,
			isr_handler_t handler,
			void *prv_ptr)
{
	/*
	 * We can't keep this data in stack
	 * because this memory will be reused by
	 * the underlined calls, therefore, use heap.
	 * Don't use more than 1 device at the same time.
	 *
	 * TODO: fix this limitation.
	 */
	static struct i2c_msg_t msgs[2];

	msgs[0].data = reg;
	msgs[0].dir = dir1;
	msgs[0].size = reg_size;

	msgs[1].data = buf;
	msgs[1].dir = dir2;
	msgs[1].size = size;

	return i2c_transfer(dev, addr, msgs,
		(buf && size) ? 2 : 1, handler, prv_ptr);
}

/* returns true if we should continue */
static bool send_next_message(i2c_dev_t *dev)
{
	if (dev->xfer.msgs_left == 0)
		return false;

	/* if this is a last byte in message */
	if (dev->xfer.msgs->size == 0) {
		enum i2c_dir prev_dir = dev->xfer.msgs->dir;

		dev->xfer.msgs_left--;

		if (dev->xfer.msgs_left == 0) {
			stop(dev);
			return false;
		}

		/* switch to next message */
		dev->xfer.msgs++;

		/* when operation changes, start should be repeated */
		if (dev->xfer.msgs->dir != prev_dir) {
			initiate(dev, dev->xfer.address,
				dev->xfer.msgs->dir, true,
				dev->xfer.msgs->size);
			return false;
		}
	}

	dev->xfer.msgs->size--;
	return true;
}

static void isr(const uint8_t i2c_num)
{
	i2c_dev_t *dev = devs[i2c_num];
	uint32_t isr = dev->i2c->ISR;

	/* 
	 * This flag is set by hardware when a NACK
	 * is received after a byte transmission.
	 */
	if (isr & I2C_ISR_NACKF) {
		dev->i2c->ICR = I2C_ICR_NACKCF;
		finish_xfer(dev, I2C_ERR_NOACK);
		return;
	}

	/* This flag is set by hardware in case of arbitration loss. */
	if (isr & I2C_ISR_ARLO) {
		BIT_CLR(dev->i2c->CR1, I2C_CR1_PE);
		finish_xfer(dev, I2C_ERR_ARLO);
		return;
	}

	/* This flag is set by hardware when a Stop condition is detected. */
	if (isr & I2C_ISR_STOPF) {
		dev->i2c->ICR = I2C_ICR_STOPCF;
		finish_xfer(dev, 0);
		return;
	}

	/* One byte sent */
	if (isr & I2C_ISR_TXIS) {
		if (send_next_message(dev))
			dev->i2c->TXDR = *dev->xfer.msgs->data++;
		return;
	}

	/* One byte received */
	if (isr & I2C_ISR_RXNE) {
		uint8_t rx = dev->i2c->RXDR;
		if (send_next_message(dev))
			*dev->xfer.msgs->data++ = rx;
		if (!dev->xfer.msgs->size)
			stop(dev);
		return;
	}
}

/**************** PUBLIC INTERFACE FUNCIONS ****************/
int i2c_init(i2c_dev_t *dev,
	     GPIO_TypeDef *gpio,
	     uint16_t scl_pin_mask,
	     uint16_t sda_pin_mask,
	     i2c_mode_t mode)
{
	const struct i2c_periph_dev *periph = 0;

	for (int i = 0; i != ARRAY_SIZE(devices); i++) {
		if (devices[i].gpio == gpio &&
		    devices[i].scl == scl_pin_mask &&
		    devices[i].sda == sda_pin_mask)
			periph = &devices[i];
	}

	if (!periph)
		return EINVAL;

	gpio_alt_func_init(periph->gpio, periph->scl, periph->alt_func);
	gpio_alt_func_init(periph->gpio, periph->sda, periph->alt_func);

	if (periph->index == 1) {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
		NVIC_EnableIRQ(I2C1_IRQn);
		dev->i2c = I2C1;
	} else {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
		NVIC_EnableIRQ(I2C2_IRQn);
		dev->i2c = I2C2;
	}

	periph->i2c->TIMINGR = get_timings(mode);

#ifdef FREERTOS
	dev->done = xSemaphoreCreateBinary();
	dev->lock = xSemaphoreCreateMutex();
#endif
	/* Store pointer for ISR mapping */
	devs[periph->index - 1] = dev;

	return 0;
}

int i2c_transfer(i2c_dev_t *dev,
		 uint8_t addr,
		 struct i2c_msg_t *msgs,
		 uint8_t nof_msgs,
		 isr_handler_t handler,
		 void *prv_ptr)
{
	dev->xfer.done = false;
	dev->xfer.handler = handler;
	dev->xfer.prv_ptr = prv_ptr;
	dev->xfer.msgs = msgs;
	dev->xfer.msgs_left = nof_msgs;
	dev->xfer.cnt = 0;
	dev->xfer.error = 0;
	dev->xfer.address = addr;

	dev->i2c->CR1 = I2C_CR1_STOPIE | \
			I2C_CR1_ERRIE | \
			I2C_CR1_PE | \
			I2C_CR1_TXIE | \
			I2C_CR1_RXIE;

	initiate(dev, addr, msgs->dir, true, msgs->size);

	if (!dev->xfer.handler)
		while (!dev->xfer.done) { }
	else
		return 0;

	return dev->xfer.error;
}

int i2c_write(i2c_dev_t *dev,
	      uint8_t addr,
	      uint8_t *src1,
	      uint8_t size1,
	      uint8_t *src2,
	      uint8_t size2)
{
	return send_message(dev, addr,
		src1, size1, I2C_WRITING,
		src2, size2, I2C_WRITING,
		0, 0);
}

int i2c_read(i2c_dev_t *dev,
	     uint8_t addr,
	     uint8_t *dst1,
	     uint8_t size1,
	     uint8_t *dst2,
	     uint8_t size2)
{
	return send_message(dev, addr,
		dst1, size1, I2C_WRITING,
		dst2, size2, I2C_READING,
		0, 0);
}

int i2c_simple_write(i2c_dev_t *dev,
		     uint8_t addr,
		     uint8_t *src,
		     uint16_t size)
{
	return send_message(dev, addr, src, size, I2C_WRITING,
		0, 0, I2C_WRITING, 0, 0);
}

int i2c_simple_read(i2c_dev_t *dev,
		    uint8_t addr,
		    uint8_t *dst,
		    uint16_t size)
{
	return send_message(dev, addr, dst, size, I2C_READING,
		0, 0, I2C_WRITING, 0, 0);
}

int i2c_write_reg(i2c_dev_t *dev,
		  uint8_t addr,
		  uint8_t reg,
		  uint8_t *src,
		  uint16_t size)
{
	return send_message(dev, addr, &reg, 1, I2C_WRITING,
		src, size, I2C_WRITING, 0, 0);
}

int i2c_read_reg(i2c_dev_t *dev,
		 uint8_t addr,
		 uint8_t reg,
		 uint8_t *dst,
		 uint16_t size)
{
	return send_message(dev, addr, &reg, 1, I2C_READING,
		dst, size, I2C_WRITING, 0, 0);
}

void I2C1_IRQHandler(void)
{
	isr(0);
}

void I2C2_IRQHandler(void)
{
	isr(1);
}

#ifdef FREERTOS

#ifndef I2C_RTOS_TIMEOUS_MS
#define I2C_RTOS_TIMEOUS_MS	100
#endif

static void rtos_handler(void *arg, uint8_t err)
{
	i2c_dev_t *dev = (i2c_dev_t *)arg;
	dev->err = err;
	xSemaphoreGiveFromISR(dev->done, 0);
}

static int rw_reg8_rtos(i2c_dev_t *dev,
			uint8_t addr,
			uint8_t *reg,
			uint8_t reg_size,
			enum i2c_dir dir1,
			uint8_t *data,
			uint16_t size,
			enum i2c_dir dir2)
{
	if (xSemaphoreTake(dev->lock, portMAX_DELAY) == pdTRUE) {
		xSemaphoreTake(dev->done, 0); /* clear */
		send_message(dev, addr, reg, reg_size, dir1,
			data, size, dir2, rtos_handler, dev);

		if (xSemaphoreTake(dev->done,
			pdMS_TO_TICKS(I2C_RTOS_TIMEOUS_MS)) != pdTRUE)
			dev->err = EINVAL;
	}

	xSemaphoreGive(dev->lock);
	return dev->err;
}

int i2c_write_rtos(i2c_dev_t *dev,
		   uint8_t addr,
		   uint8_t *src1,
		   uint8_t size1,
		   uint8_t *src2,
		   uint8_t size2)
{
	return rw_reg8_rtos(dev, addr,
		src1, size1, I2C_WRITING,
		src2, size2, I2C_WRITING);
}

int i2c_read_rtos(i2c_dev_t *dev,
		  uint8_t addr,
		  uint8_t *dst1,
		  uint8_t size1,
		  uint8_t *dst2,
		  uint8_t size2)
{
	return rw_reg8_rtos(dev, addr,
		dst1, size1, I2C_WRITING,
		dst2, size2, I2C_READING);
}

int i2c_simple_write_rtos(i2c_dev_t *dev,
			  uint8_t addr,
			  uint8_t *src,
			  uint16_t size)
{
	return rw_reg8_rtos(dev, addr, src, size, I2C_WRITING,
		0, 0, I2C_WRITING);
}

int i2c_simple_read_rtos(i2c_dev_t *dev,
			 uint8_t addr,
			 uint8_t *dst,
			 uint16_t size)
{
	return rw_reg8_rtos(dev, addr, dst, size, I2C_READING,
		0, 0, I2C_READING);
}

int i2c_write_reg_rtos(i2c_dev_t *dev,
		       uint8_t addr,
		       uint8_t reg,
		       uint8_t *src,
		       uint16_t size)
{
	return rw_reg8_rtos(dev, addr, &reg, sizeof(reg), I2C_WRITING,
		src, size, I2C_WRITING);
}

int i2c_read_reg_rtos(i2c_dev_t *dev,
		      uint8_t addr,
		      uint8_t reg,
		      uint8_t *dst,
		      uint16_t size)
{
	return rw_reg8_rtos(dev, addr, &reg, sizeof(reg), I2C_WRITING,
		dst, size, I2C_READING);
}

#endif /* FREERTOS */
