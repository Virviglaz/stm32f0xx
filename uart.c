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

#include <errno.h>
#include <stdbool.h>
#include "rcc.h"
#include "uart.h"

#define NOF_DEVICES			6 /*< USART1..USART6 */
#define UART(x)		(void *)uarts[(x) - 1]

__INLINE static uint16_t UART_BRR_SAMPLING8(uint32_t _PCLK_, uint32_t _BAUD_)
{
    uint16_t Div = (_PCLK_ + _BAUD_) / (_BAUD_ << 1);
    return ((Div & ~0x7) << 1 | (Div & 0x07));
}

/* List of UART devices, connections, dma channels and capabilities */
static const struct uart_periph_dev {
	uint8_t index;			/* index of uart i.e. 1..6 */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint16_t tx_pin;		/* tx pin bit mask BIT(x) */
	uint16_t rx_pin;		/* rx pin bit mask BIT(x) */
	enum gpio_alt_t alt_func;	/* alt function index GPIO_AF0..7 */
	IRQn_Type irq;
} uart_periph_devices[] = {
#if defined (STM32F030xC)
	{ 4, GPIOA, BIT(0),  BIT(1),  GPIO_AF4,	0 },
#endif
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, GPIOA, BIT(2),  BIT(2),  GPIO_AF1, USART1_IRQn },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 6, GPIOA, BIT(2),  BIT(2),  GPIO_AF1, USART3_6_IRQn },
#endif
#if defined (STM32F030xC)
	{ 6, GPIOA, BIT(4),  BIT(5),  GPIO_AF5, USART3_6_IRQn },
#endif
	{ 1, GPIOA, BIT(9),  BIT(10), GPIO_AF1, USART1_IRQn },
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, GPIOA, BIT(14), BIT(15), GPIO_AF1, USART1_IRQn },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 2, GPIOA, BIT(14), BIT(15), GPIO_AF1, USART2_IRQn },
#endif
#if defined (STM32F030xC)
	{ 5, GPIOB, BIT(3),  BIT(4),  GPIO_AF4, 0 },
	{ 1, GPIOB, BIT(6),  BIT(7),  GPIO_AF0, USART1_IRQn },
	{ 3, GPIOB, BIT(10), BIT(11), GPIO_AF4, USART3_6_IRQn },
	{ 6, GPIOC, BIT(0),  BIT(1),  GPIO_AF2, USART3_6_IRQn },
	{ 4, GPIOC, BIT(10), BIT(11), GPIO_AF0,	0 },
	{ 3, GPIOC, BIT(10), BIT(11), GPIO_AF1, USART3_6_IRQn },
#endif
};

/* Local storage for interrupt mapping */
static uart_dev_t *uart_devices[NOF_DEVICES];

static inline uint32_t clock_enable(uint8_t uart_num)
{
	struct system_clock_t *clocks = get_clocks();

	switch (uart_num) {
	case 1:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
		return clocks->apb2_freq;
	case 2:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
		return clocks->apb1_freq;
	case 3:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
		return clocks->apb1_freq;
	case 4:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART4EN);
		return clocks->apb1_freq;
	case 5:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART5EN);
		return clocks->apb1_freq;
	case 6:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART6EN);
		return clocks->apb2_freq;
	case 7:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART7EN);
		return clocks->apb2_freq;
	case 8:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART8EN);
		return clocks->apb2_freq;
	}

	return 0;
}

static void send_byte(uart_dev_t *dev)
{
	/* all data send out, call handler */
	if (dev->buffers.tx.cnt == dev->buffers.tx.size) {
		BIT_CLR(dev->base->CR1, USART_CR1_TCIE);
		BIT_CLR(dev->base->CR3, USART_CR3_DMAT);
		if (dev->buffers.tx.handler)
			dev->buffers.tx.handler(dev->buffers.tx.data,
			dev->buffers.tx.cnt,
			dev->buffers.tx.data);
		dev->buffers.tx.done = true;
		return;
	}

	/* send next byte */
	dev->base->TDR = dev->buffers.tx.buf[dev->buffers.tx.cnt];
	dev->buffers.tx.cnt++;
}

static inline void rx_isr(uart_dev_t *dev)
{
	char data = dev->base->RDR;
	bool done = false;

	if (dev->buffers.rx.s_byte_rcv) {
		dev->buffers.rx.s_byte_rcv(data);
		return;
	}

	dev->buffers.rx.buf[dev->buffers.rx.cnt] = data;

	if (dev->buffers.rx.stop > 0 && data == dev->buffers.rx.stop)
		done = true;

	if (dev->buffers.rx.cnt == dev->buffers.rx.size)
		done = true;
	
	if (done) {
		if (dev->buffers.rx.handler)
			dev->buffers.rx.handler(dev->buffers.rx.buf,
				dev->buffers.rx.size,
				dev->buffers.rx.data);
		dev->buffers.rx.cnt = 0;
		dev->buffers.rx.done = true;
		return;
	}

	dev->buffers.rx.cnt++;
}

static void isr(uint8_t uart_num)
{
	uart_dev_t *dev = uart_devices[uart_num - 1];
	uint32_t status = dev->base->ISR;

	/* receive interrupt */
	if (status & USART_ISR_RXNE)
		rx_isr(dev);

	/* overrin error */
	if (status & USART_ISR_ORE) {
		dev->buffers.rx.error |= USART_ISR_ORE;
		dev->base->ICR = USART_ICR_ORECF;
	}

	/* transmit interrupt */
	if (status & USART_ISR_TXE)
		send_byte(dev);
}


/**************** PUBLIC INTERFACE FUNCIONS ****************/
int uart_init(uart_dev_t *dev,
	      uint8_t uart_num,
	      uint16_t tx_pin,
	      uint16_t rx_pin,
	      uint32_t freq)
{
	const struct uart_periph_dev *uart_periph = 0;
	USART_TypeDef *uarts[] = { USART1, USART2, USART3, USART4, USART5, USART6 }; 

	/* Use lookup table to find a desired peripheral usart device */
	for (int i = 0; i != ARRAY_SIZE(uart_periph_devices); i++) {
		if (	uart_periph_devices[i].index == uart_num &&
			uart_periph_devices[i].tx_pin == tx_pin &&
			uart_periph_devices[i].rx_pin == rx_pin) {
			uart_periph = &uart_periph_devices[i];
			break;
		}
	}

	if (!uart_periph)
		return EINVAL;

	dev->uart_periph = uart_periph;
	dev->base = uarts[uart_num - 1];

	gpio_alt_func_init(uart_periph->gpio, uart_periph->tx_pin, uart_periph->alt_func);
	gpio_alt_func_init(uart_periph->gpio, uart_periph->rx_pin, uart_periph->alt_func);

	dev->base->BRR = UART_BRR_SAMPLING8(clock_enable(uart_num), freq);
	dev->base->CR1 = USART_CR1_UE | USART_CR1_TE;
	dev->base->CR2 = 0;
	dev->base->CR3 = 0;

	if (uart_periph->irq)
		NVIC_EnableIRQ(uart_periph->irq);

	/* save for interrupt mapping */
	uart_devices[uart_num - 1] = dev;

#ifdef FREERTOS
	dev->buffers.tx.slock = xSemaphoreCreateMutex();
	dev->buffers.rx.slock = xSemaphoreCreateMutex();
	dev->buffers.tx.sdone = xSemaphoreCreateBinary();
	dev->buffers.rx.sdone = xSemaphoreCreateBinary();
#endif
	return 0;
}

void uart_enable_rx_multi(uart_dev_t *dev,
			  char *dst,
			  uint16_t size,
			  uart_handler_t handler,
			  void *data,
			  int stop)
{
	dev->buffers.rx.buf = dst;
	dev->buffers.rx.size = size - 1;
	dev->buffers.rx.cnt = 0;
	dev->buffers.rx.s_byte_rcv = 0;
	dev->buffers.rx.handler = handler;
	dev->buffers.rx.data = data;
	dev->buffers.rx.error = 0;
	dev->buffers.rx.done = true;
	dev->buffers.rx.stop = stop;

	BIT_SET(dev->base->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
}

void uart_enable_rx_single(uart_dev_t *dev, uart_rx_single_byte_t handler)
{
	dev->buffers.rx.s_byte_rcv = handler;
	dev->buffers.rx.handler = 0;

	BIT_SET(dev->base->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
}

void uart_disable_rx(uart_dev_t *dev)
{
	BIT_CLR(dev->base->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
	dev->buffers.rx.handler = 0;
	dev->buffers.rx.s_byte_rcv = 0;
}

uint16_t uart_send(uart_dev_t *dev,
		   char *src,
		   uint16_t size,
		   uart_handler_t handler,
		   void *data)
{
	dev->buffers.tx.buf = src;
	dev->buffers.tx.size = size;
	dev->buffers.tx.cnt = 0;
	dev->buffers.tx.handler = handler;
	dev->buffers.tx.data = data;
	dev->buffers.tx.done = false;

	BIT_SET(dev->base->CR1, USART_CR1_TCIE);
	send_byte(dev);

	/* handler is not provided, wait for finish */
	if (!handler)
		while (!dev->buffers.tx.done) { }
	else
		return 0; /* not waiting */

	return dev->buffers.tx.cnt;
}

void uart_wait_tx_done(uart_dev_t *dev)
{
	while (!dev->buffers.tx.done) { }
}

uint32_t uart_check_rx_error(uart_dev_t *dev)
{
	uint32_t ret = dev->buffers.rx.error;
	dev->buffers.rx.error = 0;
	return ret;
}

uint16_t uart_get_send_bytes(uart_dev_t *dev)
{
	return dev->buffers.tx.cnt;
}

uint16_t uart_get_recv_bytes(uart_dev_t *dev)
{
	return dev->buffers.rx.cnt;
}

void USART1_IRQHandler(void)
{
	isr(1);
}

void USART2_IRQHandler(void)
{
	isr(2);
}

void USART3_6_IRQHandler(void)
{
	isr(3);
	isr(6);
}

#ifdef FREERTOS
static void rtos_handler(char *buffer, uint16_t size, void *arg)
{
	SemaphoreHandle_t done = (SemaphoreHandle_t)arg;
	xSemaphoreGiveFromISR(done, 0);
}

uint16_t uart_send_rtos(uart_dev_t *dev,
			char *src,
			uint16_t size,
			uint32_t timeout_ms)
{
	uint16_t ret = 0;

	/* obtain lock */
	if (xSemaphoreTake(dev->buffers.tx.slock, portMAX_DELAY) == pdTRUE) {
		xSemaphoreTake(dev->buffers.tx.sdone, 0); /* clear semaphore */
		ret = uart_send(dev, src, size, rtos_handler, (void *)dev->buffers.tx.sdone);

		/* wait for data to be send out */
		if (xSemaphoreTake(dev->buffers.tx.sdone, timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY) == pdTRUE)
			ret = dev->buffers.tx.cnt;

		xSemaphoreGive(dev->buffers.tx.slock);
	} else {
		return 0; /* lock timeout */
	}

	return ret;
}

uint16_t uart_receive_rtos(uart_dev_t *dev,
			   char *dst,
			   uint16_t size,
			   uint32_t timeout_ms,
			   int stop)
{
	uint16_t ret = 0;

	/* obtain lock */
	if (xSemaphoreTake(dev->buffers.rx.slock, portMAX_DELAY) == pdTRUE) {
		xSemaphoreTake(dev->buffers.rx.sdone, 0); /* clear semaphore */
		uart_enable_rx_multi(dev, dst, size, rtos_handler, (void *)dev->buffers.rx.sdone, stop);

		/* wait for data to be received */
		if (xSemaphoreTake(dev->buffers.rx.sdone, timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY) == pdTRUE)
			ret = dev->buffers.rx.cnt;

		xSemaphoreGive(dev->buffers.rx.slock);
	} else {
		return 0; /* lock timeout */
	}

	return ret;
}

#endif /* FREERTOS */
