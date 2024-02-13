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

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "gpio.h"

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
#endif

/**
 * @brief Uart types definition.
 */
typedef void (*uart_handler_t)(char *buffer, uint16_t size, void *data);
typedef void (*uart_rx_single_byte_t)(char byte);

/**
 * @brief Uart buffers.
*/
struct uart_buf_t {
	struct {
		char *buf;		/* data will be stored here */
		char stop;		/* end of data marker */
		uint16_t size;		/* maximum buffer size */
		uint16_t cnt;		/* byte counter */
		uart_handler_t handler; /* buffer full/new line handler */
		void *data;		/* private data for generic use */
		uint32_t error;		/* error flags will be stored here */
		uart_rx_single_byte_t s_byte_rcv; /* single byte rcv handler */
		volatile bool done;	/* set to true when done */
#ifdef FREERTOS
		SemaphoreHandle_t slock;
		SemaphoreHandle_t sdone;
	} rx;
#endif
	struct {
		char *buf;		/* data will be taken from here */
		uint16_t size;		/* amount of bytes to transfer */
		uint16_t cnt;		/* byte counter */
		uart_handler_t handler; /* end of transfer handler */
		void *data;		/* private data for generic use */
		volatile bool done;	/* set to true when done */
#ifdef FREERTOS
		SemaphoreHandle_t slock;
		SemaphoreHandle_t sdone;
	} tx;
#endif
};

/**
 * @brief Uart device definition struct including uart buffers.
*/
typedef struct uart_dev {
	const struct uart_periph_dev *uart_periph;
	USART_TypeDef *base;
	struct uart_buf_t buffers;
} uart_dev_t;

/**
  * @brief Initialize the UART.
  *
  * @param dev		Pointer where the device specific data is stored to.
  * @param uart_num	Number of UART device [1..6].
  * @param tx_pin	Pin mask where TX pin is connected to.
  * @param rx_pin	Pin mask where RX pin is connected to.
  * @param freq		UART communication frequency.
  *
  * @retval zero if success, error code if failed.
  */
int uart_init(uart_dev_t *dev,
	      uint8_t uart_num,
	      uint16_t tx_pin,
	      uint16_t rx_pin,
	      uint32_t freq);

/**
  * @brief  Receive multiple bytes using receive interrupt.
  *
  * @param dev		Pointer where the device specific data is stored to.
  * @param dst		Pointer to destination buffer.
  * @param size	Size of destination buffer.
  * @param handler	Callback called when:
  *			1. if 'stop' >=0 when stop character received.
  *			2. when buffer is full (== size).
  *			When handler is provided, this is non-waiting call.
  *			If no handler provided this is a waiting call.
  * @param data		Pointer private data if needed.
  * @param stop		Stop receiving character (negative value if not used).
  *
  * @return none.
  */
void uart_enable_rx_multi(uart_dev_t *dev,
			  char *dst,
			  uint16_t size,
			  uart_handler_t handler,
			  void *data,
			  int stop);

/**
  * @brief Enable receiving interrupt for each individual byte.
  *
  * @param dev		Pointer where the device specific data is stored to.
  * @param handler	Pointer to callback function.
  *
  * @return none.
  */
void uart_enable_rx_single(uart_dev_t *dev, uart_rx_single_byte_t handler);

/**
  * @brief  Disable receiving interrupt.
  *
  * @param dev		Pointer where the device specific data is stored to.
  *
  * @retval none.
  */
void uart_disable_rx(uart_dev_t *dev);

/**
  * @brief Send data over UART.
  *
  * @param dev		Pointer where the device specific data is stored to.
  * @param src		Pointer to transmitting buffer.
  * @param size		Amount of bytes to be send.
  * @param uart_tx_handler_t	Pointer to callback function when done.
  * @param data		Pointer private data if needed.
  *
  * @retval Amount of bytes actually sent.
  */
uint16_t uart_send(uart_dev_t *dev,
		   char *src,
		   uint16_t size,
		   uart_handler_t handler,
		   void *data);

/**
  * @brief Wait for data to be sent.
  *
  * @param dev		Pointer where the device specific data is stored to.
  */
void uart_wait_tx_done(uart_dev_t *dev);

/**
  * @brief Check receive error.
  *
  * @param dev		Pointer where the device specific data is stored to.
  *
  * @retval		Error flag or zero if no errors.
  */
uint32_t uart_check_rx_error(uart_dev_t *dev);

/**
  * @brief Retrieve amount of bytes send.
  *
  * @param dev		Pointer where the device specific data is stored to.
  *
  * @retval		Amount of bytes send.
  */
uint16_t uart_get_send_bytes(uart_dev_t *dev);

/**
  * @brief Retrieve amount of bytes received.
  *
  * @param dev		Pointer where the device specific data is stored to.
  *
  * @retval		Amount of bytes received.
*/
uint16_t uart_get_recv_bytes(uart_dev_t *dev);

#ifdef FREERTOS
/**
  * @brief Send data over UART under FreeRTOS.
  * @param dev		Pointer where the device specific data is stored to.
  * @param src		Pointer to buffer.
  * @param size		Amount of bytes to send.
  *
  * @retval none.
  */
uint16_t uart_send_rtos(uart_dev_t *dev,
			char *src,
			uint16_t size,
			uint32_t timeout_ms);



/**
  * @brief Receive data over UART under FreeRTOS.
  * @param dev		Pointer where the device specific data is stored to.
  * @param buf		Buffer where the data will be copied.
  * @param size		Maximum buffer size.
  * @param timeout_ms	Timeout in [ms] to wait for data to be received.
  *			Use 0 to wait forever.
  * @param stop		Stop marker to detect end of data
  *
  * @retval amount of bytes received.
  */
uint16_t uart_receive_rtos(uart_dev_t *dev,
			   char *dst,
			   uint16_t size,
			   uint32_t timeout_ms,
			   int stop);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
