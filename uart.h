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

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "gpio.h"

/* uart device struct accessible only from source */
struct uart_dev_t;

/* uart types redefenition */
typedef const struct uart_dev_t * uart_dev;
typedef void (*uart_tx_handler_t)(void *buffer);
typedef void (*uart_rx_handler_t)(char *buffer, uint16_t size, void *data);

/**
  * @brief  Lookup for a gpio settings to configure the uart.
  * This function helps you to find a proper connection to uart if you only
  * know the where it is used. You should provide a gpio and any of tx/rx pins.
  * @param  gpio: GPIO where one of the tx/rx pins are connected to.
  * @param  pin_mask: pinmask of tx or rx pin.
  * @param  freq: uart frequency.
  *
  * @retval 0 if no settings found or a pointer to the device if success.
  */
uart_dev find_uart_dev(GPIO_TypeDef *gpio, uint16_t pin_mask, uint32_t freq);

/**
  * @brief  Get uart by index and initialize it.
  * @param  num: inxex of UART [1..6] depends of device choosen.
  * @param  freq: uart frequency.
  * @note   you can use find_uart_dev or get_uart_dev to get a uart settings.
  *
  * @retval 0 if no settings found or a pointer to the device if success.
  */
uart_dev get_uart_dev(uint8_t num, uint32_t freq);

/**
  * @brief  Enable receiving interrupt.
  * @param  dev: Pointer to settings struct.
  * @param  buf: Pointer to receiving buffer.
  * @param  size: Maximum buffer size.
  * @param  uart_rx_handler_t: Pointer handler function if needed.
  * @param  data: Pointer private data if needed.
  *
  * @retval none.
  */
void uart_enable_rx(uart_dev dev, char *buf, uint16_t size,
	uart_rx_handler_t handler, void *data);

/**
  * @brief  Disable receiving interrupt.
  * @param  dev: Pointer to settings struct.
  *
  * @retval none.
  */
void uart_disable_rx(uart_dev dev);

/**
  * @brief  Send data to UART using DMA.
  * @param  dev: Pointer to settings struct.
  * @param  buf: Pointer to trasmitter buffer.
  * @param  size: Maximum buffer size.
  * @param  uart_tx_handler_t: Pointer handler function if needed.
  * @param  data: Pointer private data if needed.
  *
  * @retval 0 if success.
  */
int uart_send_data(uart_dev dev, char *buf, uint16_t size,
	uart_tx_handler_t handler, void *data);

/**
  * @brief  Simply sends the null-terminated string to UART.
  * @param  dev: Pointer to settings struct.
  * @param  buf: Pointer to trasmitter buffer.
  *
  * @retval 0 0 if success.
  */
int uart_send_string(uart_dev dev, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */