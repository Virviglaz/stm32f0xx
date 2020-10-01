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
#include "dma.h"
#include "uart.h"
#include <errno.h>
#include <string.h>

#define UART(x)		(void *)uarts[(x) - 1]

__INLINE static uint16_t UART_BRR_SAMPLING8(uint32_t _PCLK_, uint32_t _BAUD_)
{
    uint16_t Div = (_PCLK_ + _BAUD_) / (_BAUD_ << 1);
    return ((Div & ~0x7) << 1 | (Div & 0x07));
}

struct uart_dev_t {
	uint8_t index;			/* index of uart i.e. 1..6 */
	USART_TypeDef *uart;		/* pointer to uart base address */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint16_t tx_pin;		/* tx pin bit mask BIT(x) */
	uint16_t rx_pin;		/* rx pin bit mask BIT(x) */
	enum gpio_alt_t alt_func;	/* alt function index GPIO_AF0..7 */
	uint8_t tx_dma_ch;		/* TX dma channel or 0 if not used */
	struct uart_buf_t *buffers;	/* pointer to allocated RAM buffers */
};

struct uart_buf_t {
	struct {
		char *buf;		/* data will be stored here */
		uint16_t size;		/* maximum buffer size */
		uint16_t cnt;		/* byte counter */
		uart_rx_handler_t handler; /* buffer full/new line handler */
		void *data;		/* private data for generic use */
		uint32_t error;		/* error flags will be stored here */
	} rx;

	struct {
		char *buf;		/* data will be taken from here */
		uint16_t size;		/* amount of bytes to transfer */
		uint16_t cnt;		/* byte counter */
		uart_tx_handler_t handler; /* end of transfer handler */
		void *data;		/* private data for generic use */
		bool done;		/* set to true when done */
	} tx;
};

/* RAM buffers for UART TX/RX */
static struct uart_buf_t bufs[6];

static const struct uart_dev_t pins[] = {
#if defined (STM32F030xC)
	{ 4, USART4, GPIOA, BIT(0),  BIT(1),  GPIO_AF4, 0, &bufs[3] },
#endif
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, USART1, GPIOA, BIT(2),  BIT(2),  GPIO_AF1, 2, &bufs[0] },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 6, USART6, GPIOA, BIT(2),  BIT(2),  GPIO_AF1, 0, &bufs[5] },
#endif
#if defined (STM32F030xC)
	{ 6, USART6, GPIOA, BIT(4),  BIT(5),  GPIO_AF5, 0, &bufs[5] },
#endif
	{ 1, USART1, GPIOA, BIT(9),  BIT(10), GPIO_AF1, 2, &bufs[0] },
#if defined (STM32F030X4) || defined (STM32F030X6)
	{ 1, USART1, GPIOA, BIT(14), BIT(15), GPIO_AF1, &bufs[0] },
#endif
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 2, USART2, GPIOA, BIT(14), BIT(15), GPIO_AF1, 4, &bufs[1] },
#endif
#if defined (STM32F030xC)
	{ 5, USART5, GPIOB, BIT(3),  BIT(4),  GPIO_AF4, 0, &bufs[4] },
	{ 1, USART1, GPIOB, BIT(6),  BIT(7),  GPIO_AF0, 2, &bufs[0] },
	{ 3, USART3, GPIOB, BIT(9),  BIT(10), GPIO_AF4, 0, &bufs[2] },
	{ 6, USART6, GPIOC, BIT(0),  BIT(1),  GPIO_AF2, 0, &bufs[5] },
	{ 4, USART4, GPIOC, BIT(10), BIT(11), GPIO_AF0, 0, &bufs[3] },
	{ 3, USART3, GPIOC, BIT(10), BIT(11), GPIO_AF1, 0, &bufs[2] },
#endif
};

/*
 * Pinning allocation map. Refer to reference_manual
 */
uart_dev find_uart_dev(GPIO_TypeDef *gpio, uint16_t pin_mask)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(pins); i++) {
		uart_dev p = &pins[i];
		if (p->gpio == gpio)
			if (p->tx_pin & pin_mask || p->rx_pin & pin_mask)
				return p;
	}

	return 0;
}

uart_dev get_uart_dev(uint8_t num)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(pins); i++) {
		uart_dev p = &pins[i];
		if (p->index == num)
			return p;
	}

	return 0;
}

static uint32_t clock_enable(USART_TypeDef *uart)
{
	struct system_clock_t *clocks = get_clocks();

	if (uart == USART1) {
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
		return clocks->apb2_freq;
	} else if (uart == USART2) {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART2EN);
		return clocks->apb1_freq;
	} else if (uart == USART3) {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART3EN);
		return clocks->apb1_freq;
	} else if (uart == USART4) {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART4EN);
		return clocks->apb1_freq;
	} else if (uart == USART5) {
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART5EN);
		return clocks->apb1_freq;
	} else if (uart == USART6) {
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART6EN);
		return clocks->apb2_freq;
	} else if (uart == USART7) {
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART7EN);
		return clocks->apb2_freq;
	} else if (uart == USART8) {
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART8EN);
		return clocks->apb2_freq;
	}

	return 0;
}

static inline int enable_isr(uint8_t uart_num)
{
	const enum IRQn irqs[] = { USART1_IRQn, USART2_IRQn,
#if defined (STM32F072) || (STM32F070xB)
		USART3_4_IRQn,
		USART3_4_IRQn,
#endif
#if defined (STM32F091)
		USART3_8_IRQn,
#endif
#if defined (STM32F030xC) || defined (STM32F070xB)
		USART3_6_IRQn,
		USART3_6_IRQn,
		USART3_6_IRQn,
		USART3_6_IRQn,
#endif
	};

	if (uart_num >= ARRAY_SIZE(irqs))
		return -EINVAL;

	NVIC_EnableIRQ(irqs[uart_num]);

	return 0;
}

int uart_init(uart_dev dev, uint32_t freq)
{
	USART_TypeDef *uart = dev->uart;
	uint32_t clock_source = clock_enable(uart);

	if (!uart || !clock_source)
		return -EINVAL;

	gpio_alt_func_init(dev->gpio, dev->tx_pin, dev->alt_func);
	gpio_alt_func_init(dev->gpio, dev->rx_pin, dev->alt_func);

	uart->BRR = UART_BRR_SAMPLING8(clock_source, freq);
	uart->CR1 = USART_CR1_UE | USART_CR1_TE;

	uart->CR2 = 0;
	uart->CR3 = 0;

	return enable_isr(dev->index - 1);
}

void uart_enable_rx(uart_dev dev, char *buf, uint16_t size,
	uart_rx_handler_t handler, void *data)
{
	USART_TypeDef *uart = dev->uart;
	struct uart_buf_t *b = dev->buffers;

	b->rx.buf = buf;
	b->rx.size = size - 1;
	b->rx.cnt = 0;
	b->rx.handler = handler;
	b->rx.data = data;
	b->rx.error = 0;

	BIT_SET(uart->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
}

void uart_disable_rx(uart_dev dev)
{
	USART_TypeDef *uart = dev->uart;

	BIT_CLR(uart->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
}

static void tx_handler(void *data)
{
	struct uart_buf_t *b = data;

	if (b->tx.handler)
		b->tx.handler(b->tx.data);

	b->tx.done = true;
}

static void tx_send(USART_TypeDef *uart, struct uart_buf_t *b)
{
	if (b->tx.cnt == b->tx.size) {
		BIT_CLR(uart->CR1, USART_CR1_TCIE);
		BIT_CLR(uart->CR3, USART_CR3_DMAT);
		tx_handler(b);
		return;
	}

	uart->TDR = b->tx.buf[b->tx.cnt];
	b->tx.cnt++;
}

int uart_send_data(uart_dev dev, char *buf, uint16_t size,
	uart_tx_handler_t handler, void *data)
{
	USART_TypeDef *uart = dev->uart;
	struct uart_buf_t *b = dev->buffers;
	DMA_Channel_TypeDef *ch = 0;

	b->tx.buf = buf;
	b->tx.size = size;
	b->tx.cnt = 0;
	b->tx.handler = handler;
	b->tx.data = data;
	b->tx.done = false;

	if (dev->tx_dma_ch) { /* use DMA */
		ch = get_dma_ch(dev->tx_dma_ch, tx_handler, b);
		if (ch) {
			BIT_SET(uart->CR3, USART_CR3_DMAT);
			ch->CMAR = (uint32_t)buf;
			ch->CPAR = (uint32_t)&uart->TDR;
			ch->CNDTR = size;
			ch->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | \
				DMA_CCR_EN;
		} else { /* use interrupt */
			BIT_SET(uart->CR1, USART_CR1_TCIE);
			tx_send(uart, b);
		}
	}

	if (!handler) /* hanlder is not provided, wait for finish */
			while (b->tx.done == false) { }

	return 0;
}

int uart_send_string(uart_dev dev, char *buf)
{
	return uart_send_data(dev, buf, strlen(buf), 0, 0);
}

static inline void rx_isr(struct uart_buf_t *b, USART_TypeDef *uart)
{
	char data = uart->RDR;

	b->rx.buf[b->rx.cnt] = data;

	if (b->rx.cnt == b->rx.size || data == '\r') {
		if (b->rx.handler)
			b->rx.handler(b->rx.buf, b->rx.size, b->rx.data);
		b->rx.cnt = 0;
		return;
	}

	b->rx.cnt++;
}

static void isr(USART_TypeDef *uart, uint8_t uart_num, uint32_t flag)
{
	struct uart_buf_t *b = &bufs[uart_num - 1];

	/* receive interrupt */
	if (flag & USART_ISR_RXNE)
		rx_isr(b, uart);

	/* OverRun Error */
	if (flag & USART_ISR_ORE) {
		b->rx.error |= USART_ISR_ORE;
		uart->ICR = USART_ICR_ORECF;
	}

	/* transmitting using ISR */
	if (flag * USART_ISR_TXE)
		tx_send(uart, b);
}

void USART1_IRQHandler(void)
{
	isr(USART1, 1, USART1->ISR);
}

void USART2_IRQHandler(void)
{
	isr(USART2, 2, USART2->ISR);
}

void USART3_6_IRQHandler(void)
{
	isr(USART3, 3, USART3->ISR);
	isr(USART6, 6, USART6->ISR);
}
