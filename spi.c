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

#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "rcc.h"
#include <errno.h>
#include <stdbool.h>

#define NOF_DEVICES			2 /*< SPI1 & SPI2 */

struct msg_t {
	uint8_t *tx;			/* tx buffer pointer */
	uint8_t *rx;			/* rx buffer pointer */
	uint16_t size;			/* bytes to transfer */
	struct msg_t *next;		/* next message of linked list */
};

static struct xfer_t {
	struct msg_t *msg;		/* messages linked list */
	spi_dev dev;			/* pointer to spi device */
	void (*handler)(void *data);	/* handler function for RTOS */
	void *data;			/* caller private data */
	volatile bool done;		/* ready flag */
	uint16_t cnt;			/* byte counter for isr mode */
	struct {
		GPIO_TypeDef *gpio;
		uint16_t pin;
	} cs;				/* chip select */
	struct {
		DMA_Channel_TypeDef *tx;
		DMA_Channel_TypeDef *rx;
	} dma;				/* dma channels */
} xfrs[NOF_DEVICES];

static const struct spi_dev_t {
	uint8_t index;			/* index of spi i.e. 1..6 */
	SPI_TypeDef *spi;		/* pointer to spi base address */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint16_t msck;			/* MSCK pin bit mask BIT(x) */
	uint16_t miso;			/* MOSI pin bit mask BIT(x) */
	uint16_t mosi;			/* MISO pin bit mask BIT(x) */
	enum gpio_alt_t alt_func;	/* alt function index GPIO_AF0..7 */
	uint8_t tx_dma_ch;		/* TX dma channel or 0 if not used */
	uint8_t rx_dma_ch;		/* RX dma channel or 0 if not used */
	struct xfer_t *xfer;		/* pointer to allocated RAM buffers */
} devices[] = {
	{ 1, SPI1, GPIOA, BIT(5),  BIT(6),  BIT(7),  GPIO_AF0, 3, 2, &xfrs[0] },
	{ 1, SPI1, GPIOB, BIT(3),  BIT(4),  BIT(5),  GPIO_AF0, 3, 2, &xfrs[0] },
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, SPI1, GPIOB, BIT(13), BIT(14), BIT(15), GPIO_AF0, 3, 2, &xfrs[0] },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 2, SPI2, GPIOB, BIT(13), BIT(14), BIT(15), GPIO_AF0, 5, 4, &xfrs[1] },
#endif
};

static inline uint16_t calc_clock_div(uint32_t bus_freq, uint32_t expected_freq)
{
	uint32_t freq;
	uint16_t div = 1;
	uint16_t n = 0;

	do {
		div <<= 1;
		n++;
		freq = bus_freq / div;
	} while (freq > expected_freq);

	return n - 1;
}

static void finish_transfer(struct xfer_t *xfer)
{
	SPI_TypeDef *spi = xfer->dev->spi;

	spi->CR2 = 0;
	gpio_set(xfer->cs.gpio, xfer->cs.pin);
	BIT_CLR(spi->CR1, SPI_CR1_SPE);
	if (xfer->handler)
		xfer->handler(xfer->data);
	xfer->done = true;
}

static void start_message_dma(struct xfer_t *xfer)
{
	struct msg_t *m = xfer->msg;
	SPI_TypeDef *spi = xfer->dev->spi;

	xfer->dma.tx->CCR = 0;
	xfer->dma.rx->CCR = 0;

	BIT_CLR(spi->CR1, SPI_CR1_SPE);

	if (m) {
		static uint8_t dummy = 0;
		spi->CR2 = (7 << 8) | \
			SPI_CR2_FRXTH | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

		if (m->tx)	/* DMA MEM2DEV */
			dma_setup(xfer->dma.tx, (void *)m->tx, (void *)&spi->DR,
				m->size, DMA_FLAG_MEM2DEV_B);
		else		/* DMA1 DUMMY2DEV */
			dma_setup(xfer->dma.tx, (void *)&dummy,
				(void *)&spi->DR, m->size, DMA_FLAG_DUM2DEV_B);

		if (m->rx)	/* DMA DEV2MEM */
			dma_setup(xfer->dma.rx, (void *)m->rx, (void *)&spi->DR,
				m->size, DMA_FLAG_DEV2MEM_B);
		else		/* DMA DEV2DUMMY */
			dma_setup(xfer->dma.rx, (void *)&dummy,
				(void *)&spi->DR, m->size, DMA_FLAG_DEV2DUM_B);

		BIT_SET(spi->CR1, SPI_CR1_SPE);
	} else {
		dma_release(xfer->dev->tx_dma_ch);
		dma_release(xfer->dev->rx_dma_ch);
		finish_transfer(xfer);
	}
}

static void tx_dma_isr(void *data)
{
	/* dummy call */
}

static void rx_dma_isr(void *data)
{
	struct xfer_t *xfer = data;

	xfer->msg = xfer->msg->next;
	start_message_dma(xfer);
}

static void start_message_isr(struct xfer_t *xfer)
{
	struct msg_t *m = xfer->msg;
	SPI_TypeDef *spi = xfer->dev->spi;

	if (m) {
		BIT_SET(spi->CR1, SPI_CR1_SPE);
		xfer->cnt = 0;
		spi->CR2 = (7 << 8) | SPI_CR2_FRXTH | \
			SPI_CR2_TXEIE | SPI_CR2_RXNEIE;
		*(__IO uint8_t *)&spi->DR = xfer->msg->tx[0];
	} else
		finish_transfer(xfer);
}

static int transfer(
	spi_dev dev,
	struct msg_t **msg,
	GPIO_TypeDef *gpio, uint16_t pin,
	void (*handler)(void *data),
	void *data
)
{
	struct xfer_t *xfer = &xfrs[dev->index - 1];
	struct msg_t *m = *msg;

	xfer->dev = dev;
	xfer->cs.gpio = gpio;
	xfer->cs.pin = pin;
	xfer->msg = m;
	xfer->handler = handler;
	xfer->data = data;
	xfer->done = false;
	xfer->cnt = 0;

	/* build a linked list */
	do {
		do {
			m->next = *++msg;
		} while (m->next && !m->next->size);
		m = m->next;
	} while (m);

	xfer->dma.tx = get_dma_ch(dev->tx_dma_ch, tx_dma_isr, xfer);
	xfer->dma.rx = get_dma_ch(dev->rx_dma_ch, rx_dma_isr, xfer);

	/* chip enable */
	gpio_clr(gpio, pin);

	/* use isr mode, dma is occupied or not available */
	if (!xfer->dma.tx || !xfer->dma.rx) {
		dma_release(dev->tx_dma_ch);
		dma_release(dev->rx_dma_ch);
		xfer->dma.rx = 0;

		/* DMA failed, use ISR mode */
		start_message_isr(xfer);
	} else
		start_message_dma(xfer);

	/* wait for execution if no handler provided */
	if (!xfer->handler)
		while (!xfer->done) { }

	return 0;
}

static int send_message(
	spi_dev dev,
	GPIO_TypeDef *gpio,
	uint16_t pin,
	uint8_t *reg,
	uint8_t reg_size,
	uint8_t *tx_data,
	uint8_t *rx_data,
	uint16_t size,
	void (*handler)(void *data),
	void *data)
{
	/* we use static variable to reduce the stack usage using RTOS */
	static struct msg_t msg[2];
	static struct msg_t *msgs[3];

	msg[0].tx = reg;
	msg[0].rx = 0;
	msg[0].size = reg_size;

	msg[1].tx = tx_data;
	msg[1].rx = rx_data;
	msg[1].size = size;

	msgs[0] = &msg[0];
	msgs[1] = &msg[1];
	msgs[2] = 0;

	return transfer(dev, msgs, gpio, pin, handler, data);
}

static void tx_isr(struct xfer_t *xfer)
{
	struct msg_t *m = xfer->msg;
	SPI_TypeDef *spi = xfer->dev->spi;
	uint16_t cnt = xfer->cnt + 1;

	if (m->size == cnt) {
		BIT_CLR(spi->CR2, SPI_CR2_TXEIE); /* disable TX isr */
		return;
	}

	if (m->tx) {
		*(__IO uint8_t *)&spi->DR = m->tx[cnt];
	} else
		*(__IO uint8_t *)&spi->DR = 0; /* send dummy byte */
}

static void rx_isr(struct xfer_t *xfer)
{
	struct msg_t *m = xfer->msg;
	SPI_TypeDef *spi = xfer->dev->spi;
	uint16_t cnt = xfer->cnt;

	if (xfer->dma.rx) {
		start_message_isr(xfer);
		return;
	}

	if (m->rx) {
		m->rx[cnt] = *(__IO uint8_t *)&spi->DR;
	} else
		spi->DR;

	cnt++;
	xfer->cnt = cnt;
	if (xfer->cnt == m->size) {
		xfer->msg = m->next;
		start_message_isr(xfer);
	}
}

static int init(spi_dev dev, uint32_t freq, bool idle_clock_high)
{
	struct system_clock_t *clocks = get_clocks();
	uint32_t clock_source;
	uint16_t clock_div;
	uint16_t clock_mode = idle_clock_high ? SPI_CR1_CPHA | SPI_CR1_CPOL : 0;
	SPI_TypeDef *spi = dev->spi;

	switch (dev->index) {
	case 1:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);
		clock_source = clocks->apb2_freq;
		NVIC_EnableIRQ(SPI1_IRQn);
		break;
	case 2:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);
		clock_source = clocks->apb1_freq;
		NVIC_EnableIRQ(SPI2_IRQn);
		break;
	default:
		return -EINVAL;
	}

	clock_div = calc_clock_div(clock_source, freq);

	/* initialize gpio, user is responsible for CS gpio init */
	gpio_alt_func_init(dev->gpio, dev->msck, dev->alt_func);
	gpio_alt_func_init(dev->gpio, dev->miso, dev->alt_func);
	gpio_alt_func_init(dev->gpio, dev->mosi, dev->alt_func);

	spi->CR1 = (clock_div << 3) | clock_mode | SPI_CR1_SSI | \
		SPI_CR1_SSM | SPI_CR1_MSTR;

	return 0;
}

spi_dev find_spi_dev(GPIO_TypeDef *gpio, uint16_t pin_mask, uint32_t freq,
	bool idle_clock_high)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(devices); i++) {
		spi_dev dev = &devices[i];
		if (dev->gpio == gpio && (dev->msck & pin_mask || \
			dev->miso & pin_mask || dev->mosi & pin_mask))
			return init(dev, freq, idle_clock_high) ? 0 : dev;
	}

	return 0; /* no devices found */
}

spi_dev get_spi_dev(uint8_t num, uint32_t freq, bool idle_clock_high)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(devices); i++) {
		spi_dev dev = &devices[i];
		if (dev->index == num)
			return init(dev, freq, idle_clock_high) ? 0 : dev;

	}

	return 0; /* no devices found */
}

int spi_write_reg(spi_dev dev, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message(dev, gpio, pin, &reg, 1,
		data, 0, size, 0, 0);
}

int spi_read_reg(spi_dev dev, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message(dev, gpio, pin, &reg, 1,
		0, data, size, 0, 0);
}

uint8_t spi_read_byte(spi_dev dev, GPIO_TypeDef *gpio, uint16_t pin, uint8_t b)
{
	dev->spi->CR2 = (7 << 8) | SPI_CR2_FRXTH;

	BIT_SET(dev->spi->CR1, SPI_CR1_SPE);

	/* chip enable */
	gpio_clr(gpio, pin);

	/* Loop while DR register in not empty */
	while (!(dev->spi->SR & SPI_SR_TXE));

	/* Send byte through the SPI peripheral */
	*((__IO uint8_t *)&dev->spi->DR) = b;

	/* Wait to receive a byte */
	while (!(dev->spi->SR & SPI_SR_RXNE));

	/* chip disable */
	gpio_set(gpio, pin);

	BIT_CLR(dev->spi->CR1, SPI_CR1_SPE);

	return *((__IO uint8_t *)&dev->spi->DR);
}

void SPI1_IRQHandler(void)
{
	struct xfer_t *xfer = &xfrs[0];
	uint16_t status = SPI1->SR;

	if (status & SPI_SR_TXE)
		tx_isr(xfer);

	if (status & SPI_SR_RXNE)
		rx_isr(xfer);
}

void SPI2_IRQHandler(void)
{
	struct xfer_t *xfer = &xfrs[1];
	uint16_t status = SPI2->SR;

	if (status & SPI_SR_TXE)
		tx_isr(xfer);

	if (status & SPI_SR_RXNE)
		rx_isr(xfer);
}

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
static void handler(void *arg)
{
	SemaphoreHandle_t done = (SemaphoreHandle_t)arg;
	xSemaphoreGive(done);
}

static SemaphoreHandle_t lock[NOF_DEVICES] = { 0 };
static SemaphoreHandle_t done[NOF_DEVICES] = { 0 };

static int send_message_rtos(
	spi_dev dev,
	GPIO_TypeDef *gpio,
	uint16_t pin,
	uint8_t *reg,
	uint8_t reg_size,
	uint8_t *tx_data,
	uint8_t *rx_data,
	uint16_t size)
{
	int res;
	uint8_t n = dev->index - 1;

	if (!lock[n]) {
		lock[n] = xSemaphoreCreateMutex();
		done[n] = xSemaphoreCreateBinary();
	}

	xSemaphoreTake(lock[n], portMAX_DELAY);

	res = send_message(dev, gpio, pin, reg, reg_size,
		tx_data, rx_data, size, handler, (void *)done[n]);
	if (!res)
		xSemaphoreTake(done[n], portMAX_DELAY);

	xSemaphoreGive(lock[n]);

	return res;
}

int spi_write_reg_rtos(spi_dev dev, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message_rtos(dev, gpio, pin, &reg, 1, data, 0, size);
}

int spi_read_reg_rtos(spi_dev dev, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message_rtos(dev, gpio, pin, &reg, 1, 0, data, size);
}

uint8_t spi_read_byte_rtos(spi_dev dev, GPIO_TypeDef *gpio,
	uint16_t pin, uint8_t b)
{
	uint8_t n = dev->index - 1;
	uint8_t ret;

	if (!lock[n]) {
		lock[n] = xSemaphoreCreateMutex();
		done[n] = xSemaphoreCreateBinary();
	}

	xSemaphoreTake(lock[n], portMAX_DELAY);

	ret = spi_read_byte(dev, gpio, pin, b);

	xSemaphoreGive(lock[n]);

	return ret;
}

#endif /* FREERTOS */
