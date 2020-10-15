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

#define NOF_DEVICES			6 /*< USART1..USART6 */
#define UART(x)		(void *)uarts[(x) - 1]

__INLINE static uint16_t UART_BRR_SAMPLING8(uint32_t _PCLK_, uint32_t _BAUD_)
{
    uint16_t Div = (_PCLK_ + _BAUD_) / (_BAUD_ << 1);
    return ((Div & ~0x7) << 1 | (Div & 0x07));
}

/* RAM buffers for UART TX/RX */
static struct uart_buf_t {
	struct {
		char *buf;		/* data will be stored here */
		char stop_s;		/* end of data marker \r or \n */
		uint16_t size;		/* maximum buffer size */
		uint16_t cnt;		/* byte counter */
		uart_rx_handler_t handler; /* buffer full/new line handler */
		void *data;		/* private data for generic use */
		uint32_t error;		/* error flags will be stored here */
		uart_rx_single_byte_t s_byte_rcv; /* single byte rcv handler */
	} rx;

	struct {
		char *buf;		/* data will be taken from here */
		uint16_t size;		/* amount of bytes to transfer */
		uint16_t cnt;		/* byte counter */
		uart_tx_handler_t handler; /* end of transfer handler */
		void *data;		/* private data for generic use */
		uint8_t dma_ch;		/* number of TX DMA channel used */
		volatile bool done;	/* set to true when done */
		bool dma_en;		/* set if dma is enabled */
	} tx;
} bufs[NOF_DEVICES] = { 0 };

/* List of UART devices, connections, dma channels and capabilities */
static const struct uart_dev_t {
	uint8_t index;			/* index of uart i.e. 1..6 */
	USART_TypeDef *uart;		/* pointer to uart base address */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint16_t tx_pin;		/* tx pin bit mask BIT(x) */
	uint16_t rx_pin;		/* rx pin bit mask BIT(x) */
	enum gpio_alt_t alt_func;	/* alt function index GPIO_AF0..7 */
	uint8_t tx_dma_ch;		/* TX dma channel or 0 if not used */
	struct uart_buf_t *buffers;	/* pointer to allocated RAM buffers */
} devices[] = {
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
	{ 3, USART3, GPIOB, BIT(10), BIT(11), GPIO_AF4, 0, &bufs[2] },
	{ 6, USART6, GPIOC, BIT(0),  BIT(1),  GPIO_AF2, 0, &bufs[5] },
	{ 4, USART4, GPIOC, BIT(10), BIT(11), GPIO_AF0, 0, &bufs[3] },
	{ 3, USART3, GPIOC, BIT(10), BIT(11), GPIO_AF1, 0, &bufs[2] },
#endif
};

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
	switch (uart_num) {
	case 1:
		NVIC_EnableIRQ(USART1_IRQn);
		break;
	case 2:
		NVIC_EnableIRQ(USART2_IRQn);
		break;
	case 3:
#if defined (STM32F091)
		NVIC_EnableIRQ(USART3_8_IRQn);
#endif
#if defined (STM32F030xC) || defined (STM32F070xB)
		NVIC_EnableIRQ(USART3_6_IRQn);
#endif
		break;
	case 6:
		NVIC_EnableIRQ(USART3_6_IRQn);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int uart_init(uart_dev dev, uint32_t freq)
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

	return enable_isr(dev->index);
}

uart_dev find_uart_dev(GPIO_TypeDef *gpio, uint16_t pin_mask, uint32_t freq)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(devices); i++) {
		uart_dev dev = &devices[i];
		if (dev->gpio == gpio)
			if (dev->tx_pin & pin_mask || dev->rx_pin & pin_mask)
				return uart_init(dev, freq) ? 0 : dev;
	}

	return 0; /* no devices found */
}

uart_dev get_uart_dev(uint8_t num, uint32_t freq)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(devices); i++) {
		uart_dev dev = &devices[i];
		if (dev->index == num)
			return uart_init(dev, freq) ? 0 : dev;
	}

	return 0; /* no devices found */
}

void uart_enable_rx(uart_dev dev, char *buf, uint16_t size,
	uart_rx_handler_t handler, void *data, char stop)
{
	USART_TypeDef *uart = dev->uart;
	struct uart_buf_t *b = dev->buffers;

	b->rx.buf = buf;
	b->rx.size = size - 1;
	b->rx.cnt = 0;
	b->rx.handler = handler;
	b->rx.data = data;
	b->rx.error = 0;
	b->rx.stop_s = stop;

	BIT_SET(uart->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
}

void uart_enable_single_byte_int(uart_dev dev, uart_rx_single_byte_t handler)
{
	USART_TypeDef *uart = dev->uart;
	struct uart_buf_t *b = dev->buffers;

	b->rx.s_byte_rcv = handler;

	BIT_SET(uart->CR1, USART_CR1_RE | USART_CR1_RXNEIE);
}

void uart_disable_rx(uart_dev dev)
{
	USART_TypeDef *uart = dev->uart;
	struct uart_buf_t *b = dev->buffers;

	BIT_CLR(uart->CR1, USART_CR1_RE | USART_CR1_RXNEIE);

	b->rx.handler = 0;
	b->rx.s_byte_rcv = 0;
}

static void tx_handler(void *data)
{
	struct uart_buf_t *b = data;

	dma_release(b->tx.dma_ch);

	if (b->tx.handler)
		b->tx.handler(b->tx.data);

	b->tx.done = true;
}

static void tx_send(USART_TypeDef *uart, struct uart_buf_t *b)
{
	if (b->tx.cnt == b->tx.size) {
		BIT_CLR(uart->CR1, USART_CR1_TCIE);
		BIT_CLR(uart->CR3, USART_CR3_DMAT);
		if (b->tx.handler)
			b->tx.handler(b->tx.data);
		b->tx.done = true;
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

	b->tx.buf = buf;
	b->tx.size = size;
	b->tx.cnt = 0;
	b->tx.handler = handler;
	b->tx.data = data;
	b->tx.done = false;
	if (b->tx.dma_en && dev->tx_dma_ch) { /* use DMA */
		DMA_Channel_TypeDef *ch;
		ch = get_dma_ch(dev->tx_dma_ch, tx_handler, b);
		if (ch) {
			b->tx.dma_ch = dev->tx_dma_ch;
			BIT_SET(uart->CR3, USART_CR3_DMAT);
			dma_setup(ch, buf, (void *)&uart->TDR, size,
				DMA_FLAG_MEM2DEV_B);
		} else { /* fall down to use interrupt */
			BIT_SET(uart->CR1, USART_CR1_TCIE);
			tx_send(uart, b);
		}
	} else { /* dma in not used or not available */
		BIT_SET(uart->CR1, USART_CR1_TCIE);
		tx_send(uart, b);
	}

	if (!handler) /* hanlder is not provided, wait for finish */
			while (b->tx.done == false) { }

	return 0;
}

int uart_send_string(uart_dev dev, char *buf)
{
	return uart_send_data(dev, buf, strlen(buf), 0, 0);
}

void uart_dma(uart_dev dev, bool enable)
{
	struct uart_buf_t *b = dev->buffers;

	b->tx.dma_en = enable;
}

static inline void rx_isr(struct uart_buf_t *b, USART_TypeDef *uart)
{
	char data = uart->RDR;

	if (b->rx.s_byte_rcv) {
		b->rx.s_byte_rcv(data);
		return;
	}

	b->rx.buf[b->rx.cnt] = data;

	if (b->rx.cnt == b->rx.size || (b->rx.stop_s && data == b->rx.stop_s)) {
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

#ifdef FREERTOS

static void rtos_tx_handler(void *data)
{
	rtos_schedule_isr(data);
}

static struct params_t {
	TaskHandle_t handle;
	SemaphoreHandle_t tx_mutex;
	SemaphoreHandle_t rx_mutex;
	SemaphoreHandle_t rtimeout;
	uint16_t bytes_received;
} rcv_params[NOF_DEVICES];

static void rtos_rx_handler(char *buffer, uint16_t size,
	void *data)
{
	struct params_t *par = data;

	par->bytes_received = size;

	rtos_schedule_isr(par->handle);
}

void uart_send_data_rtos(uart_dev dev, char *buf, uint16_t size)
{
	TaskHandle_t handle;
	uint8_t dev_num = dev->index - 1;

	if (!rcv_params[dev_num].tx_mutex)
		rcv_params[dev_num].tx_mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(rcv_params[dev_num].tx_mutex, portMAX_DELAY);

	handle = xTaskGetCurrentTaskHandle();

	if (!uart_send_data(dev, buf, size, rtos_tx_handler, handle))
		vTaskSuspend(handle);

	xSemaphoreGive(rcv_params[dev_num].tx_mutex);
}

void uart_send_string_rtos(uart_dev dev, char *string)
{
	uart_send_data_rtos(dev, string, strlen(string));
}

uint16_t uart_receive_rtos(uart_dev dev, char *buf, uint16_t size, char stop)
{
	uint8_t dev_num = dev->index - 1;
	struct params_t *par;

	if (!rcv_params[dev_num].rx_mutex)
		rcv_params[dev_num].rx_mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(rcv_params[dev_num].rx_mutex, portMAX_DELAY);

	par = &rcv_params[dev_num];

	par->handle = xTaskGetCurrentTaskHandle();

	uart_enable_rx(dev, buf, size, rtos_rx_handler, par, stop);

	vTaskSuspend(par->handle);

	uart_disable_rx(dev);

	xSemaphoreGive(rcv_params[dev_num].rx_mutex);

	return par->bytes_received;
}

static void rtos_rx_timeout_isr(char *buffer, uint16_t size,
	void *data)
{
	struct params_t *par = data;

	par->bytes_received = size;

	xSemaphoreGiveFromISR(par->rtimeout, 0);
}

uint16_t uart_receive_by_timeout_rtos(uart_dev dev, char *buf, uint16_t size,
	uint16_t timeout, char stop)
{
	uint8_t dev_num = dev->index - 1;
	struct params_t *par;

	if (!rcv_params[dev_num].rx_mutex)
		rcv_params[dev_num].rx_mutex = xSemaphoreCreateMutex();

	if (!rcv_params[dev_num].rtimeout)
		rcv_params[dev_num].rtimeout = xSemaphoreCreateBinary();

	xSemaphoreTake(rcv_params[dev_num].rx_mutex, portMAX_DELAY);

	par = &rcv_params[dev_num];

	uart_enable_rx(dev, buf, size, rtos_rx_timeout_isr, par, stop);

	xSemaphoreTake(rcv_params[dev_num].rtimeout, timeout);

	uart_disable_rx(dev);

	xSemaphoreGive(rcv_params[dev_num].rx_mutex);

	return par->bytes_received;
}

#endif /* FREERTOS */
