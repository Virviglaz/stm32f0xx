/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2020 Pavel Nadein
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

#define NOF_DEVICES			2 /*< I2C1 & I2C2 */

/* task and error reporting */
enum task_t {
	NO_INIT = 0,
	IN_PROGRESS,
	DONE,
	ERR_NOACK,
	ERR_UNKNOWN,
};

enum dir_t {
	WRITING,
	READING,
};

/* every transfer is considered as the message */
struct msg_t {
	enum dir_t dir;
	uint8_t *data;
	uint16_t size;
	struct msg_t *next;
};

static struct xfer_t {
	struct msg_t *msg;
	i2c_dev dev;
	enum task_t task;
	uint8_t addr;
	void (*handler)(void *data, uint8_t err);
	void *data;
	bool done;
	uint16_t cnt;			/* byte counter for isr mode */
	struct {
		DMA_Channel_TypeDef *tx;
		DMA_Channel_TypeDef *rx;
	} dma;
} xfrs[NOF_DEVICES];

static const struct i2c_dev_t {
	uint8_t index;			/* index of i2c i.e. 1..6 */
	I2C_TypeDef *i2c;		/* pointer to i2c base address */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint16_t scl;			/* MSCK pin bit mask BIT(x) */
	uint16_t sda;			/* MOSI pin bit mask BIT(x) */
	enum gpio_alt_t alt_func;	/* alt function index GPIO_AF0..7 */
	struct xfer_t *xfer;		/* pointer to allocated RAM buffers */
} devices[] = {
#if defined (STM32F030xC) || defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, I2C1, GPIOA, BIT(9),  BIT(10), GPIO_AF4, &xfrs[0] },
#endif
	{ 1, I2C1, GPIOB, BIT(6),  BIT(7),  GPIO_AF1, &xfrs[0] },
	{ 1, I2C1, GPIOB, BIT(9),  BIT(10), GPIO_AF1, &xfrs[0] },
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, I2C1, GPIOB, BIT(10), BIT(11), GPIO_AF1, &xfrs[0] },
	{ 1, I2C1, GPIOF, BIT(6),  BIT(7),  GPIO_AF0, &xfrs[0] },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 2, I2C2, GPIOB, BIT(10), BIT(11), GPIO_AF1, &xfrs[1] },
#endif
#if defined (STM32F030xC)
	{ 2, I2C2, GPIOB, BIT(13), BIT(14), GPIO_AF5, &xfrs[1] },
	{ 1, I2C1, GPIOF, BIT(0),  BIT(1),  GPIO_AF1, &xfrs[0] },
#endif
};

static uint32_t get_timings(bool fast_mode)
{
	struct system_clock_t *clock = get_clocks();
	struct {
		uint8_t scll	: 8;
		uint8_t sclh	: 8;
		uint8_t sdadel	: 4;
		uint8_t scldel	: 4;
		uint8_t 	: 4;
		uint8_t presc	: 4;
	} t;

	*(uint32_t *)&t = fast_mode ? 0x00310309 : 0x00420F13;

	t.presc = (clock->apb1_freq / 8000000) - fast_mode ? 1 : 0; 

	return *(uint32_t *)&t;
}

static int init(i2c_dev dev, bool fast_mode)
{

	I2C_TypeDef *i2c = dev->i2c;
	uint32_t timings = get_timings(fast_mode);
	if (!timings)
		return -EINVAL;

	if (dev->index == 1) {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
		NVIC_EnableIRQ(I2C1_IRQn);
	} else {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
		NVIC_EnableIRQ(I2C2_IRQn);
	}

	gpio_alt_func_init(dev->gpio, dev->scl, dev->alt_func);
	gpio_alt_func_init(dev->gpio, dev->sda, dev->alt_func);

	i2c->TIMINGR = timings;

	return 0;
}

i2c_dev find_i2c_dev(GPIO_TypeDef *gpio, uint16_t pin_mask, bool fast_mode)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(devices); i++) {
		i2c_dev dev = &devices[i];
		if (dev->gpio == gpio && (dev->scl & pin_mask || \
			dev->sda & pin_mask))
			return init(dev, fast_mode) ? 0 : dev;
	}

	return 0; /* no devices found */
}

i2c_dev get_i2c_dev(uint8_t num, bool fast_mode)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(devices); i++) {
		i2c_dev dev = &devices[i];
		if (dev->index == num)
			return init(dev, fast_mode) ? 0 : dev;
	}

	return 0; /* no devices found */
}

static inline void start(I2C_TypeDef *i2c, uint8_t addr, uint8_t size)
{
	i2c->CR2 = I2C_CR2_START | addr | (size << 16) | \
		(addr & 1 ? I2C_CR2_RD_WRN : 0);
}

static inline uint16_t get_total_nof_tx_bytes(struct msg_t *m)
{
	uint16_t i = 0;
	while (m) {
		if (m->dir == WRITING)
			i += m->size;
		m = m->next;
	}
	return i;
}

/* generic transfer function. Condigure the bus and create a messages list */
static int transfer(i2c_dev dev, uint8_t addr, struct msg_t **msg,
	void (*handler)(void *data, uint8_t err), void *data)
{
	I2C_TypeDef *i2c = dev->i2c;
	struct xfer_t *xfer = dev->xfer;
	struct msg_t *m = *msg;

	xfer->addr = addr;
	xfer->cnt = 0;
	xfer->dev = dev;
	xfer->done = false;
	xfer->task = IN_PROGRESS;
	xfer->handler = handler;
	xfer->data = data;
	xfer->msg = m;

	/* build a linked list */
	do {
		do {
			m->next = *++msg;
		} while (m->next && !m->next->size);
		m = m->next;
	} while (m);

	BIT_SET(i2c->CR2, (xfer->msg->size & 0xFF) << 16);

	i2c->CR1 = I2C_CR1_STOPIE | I2C_CR1_ERRIE | I2C_CR1_PE | I2C_CR1_TCIE |\
		I2C_CR1_TXIE | I2C_CR1_RXIE;

	start(i2c, xfer->addr, get_total_nof_tx_bytes(xfer->msg));

	if (!xfer->handler)
		while (xfer->task < DONE);
	else
		return 0;

	return xfer->task == DONE ? I2C_SUCCESS : I2C_ERR_NOACK;
}

static int send_message(
	i2c_dev dev,
	uint8_t addr,
	uint8_t *reg,
	uint8_t reg_size,
	uint8_t *buffer,
	uint16_t size,
	enum dir_t dir,
	void (*handler)(void *data, uint8_t err),
	void *data)
{
	/* we use static variable to reduce the stack usage using RTOS */
	static struct msg_t msg[2];
	static struct msg_t *msgs[3];

	msg[0].dir = WRITING;
	msg[0].data = reg;
	msg[0].size = reg_size;

	msg[1].dir = dir;
	msg[1].data = buffer;
	msg[1].size = size;

	msgs[0] = &msg[0];
	msgs[1] = &msg[1];
	msgs[2] = 0;

	return transfer(dev, addr << 1, msgs, handler, data);
}

static void finish_xfer(struct xfer_t *xfer, uint8_t err)
{
	I2C_TypeDef *i2c = xfer->dev->i2c;

	i2c->CR1 = 0; /* disable i2c */

	if (xfer->handler)
		xfer->handler(xfer->data, err);

	xfer->task = err ? ERR_NOACK : DONE;
}

static inline void next_byte_send(struct xfer_t *xfer, I2C_TypeDef *i2c)
{
	struct msg_t *m = xfer->msg;

	if (xfer->cnt == m->size) {
		m = m->next;
		xfer->msg = m;
		xfer->cnt = 0;
	}

	i2c->TXDR = m->data[xfer->cnt];
		xfer->cnt++;
}

/*
 * this functions using consistently 2 messages transfer.
 * first message holding the address of memory of register where
 * access is planned. Then the next message is a buffer reading/writing
 * followed by repeated start condition in case of reading.
 */
int i2c_write_reg(i2c_dev dev, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(dev, addr, (void *)&reg, sizeof(reg),
		data, size, WRITING, 0, 0);
}

int i2c_read_reg(i2c_dev dev, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(dev, addr, (void *)&reg, sizeof(reg),
		data, size, READING, 0, 0);
}

static void isr(const uint8_t i2c_num)
{
	struct xfer_t *xfer = &xfrs[i2c_num];
	I2C_TypeDef *i2c = xfer->dev->i2c;
	struct msg_t *m = xfer->msg;
	uint16_t isr = (uint16_t)i2c->ISR;

	if (isr & I2C_ISR_NACKF) {
		i2c->ICR = I2C_ICR_NACKCF;
		xfer->task = ERR_NOACK;
	}

	if (isr & I2C_ISR_STOPF) {
		i2c->ICR = I2C_ICR_STOPCF;
		finish_xfer(xfer, I2C_SUCCESS);
	}

	/* end of transfer isr */
	if (isr & I2C_ISR_TC) {
		m = m->next;
		xfer->msg = m;
		xfer->cnt = 0;
		if (m && m->dir == READING)
			start(i2c, xfer->addr | 1, m->size);
		else
			BIT_SET(i2c->CR2, I2C_CR2_STOP);
	}

	if (isr & I2C_ISR_TXIS)
		next_byte_send(xfer, i2c);

	if (isr & I2C_ISR_RXNE) {
		m->data[xfer->cnt] = i2c->RXDR;
		xfer->cnt++;
	}
}

void I2C1_IRQHandler(void)
{
	isr(0);
}

void I2C2_IRQHandler(void)
{
	isr(1);
}
