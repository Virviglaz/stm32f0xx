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

#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f0xx.h>
#include <stdint.h>
#include <stdbool.h>

#define PA0			GPIOA, BIT(0)
#define PA1			GPIOA, BIT(1)
#define PA2			GPIOA, BIT(2)
#define PA3			GPIOA, BIT(3)
#define PA4			GPIOA, BIT(4)
#define PA5			GPIOA, BIT(5)
#define PA6			GPIOA, BIT(6)
#define PA7			GPIOA, BIT(7)
#define PA8			GPIOA, BIT(8)
#define PA9			GPIOA, BIT(9)
#define PA10			GPIOA, BIT(10)
#define PA11			GPIOA, BIT(11)
#define PA12			GPIOA, BIT(12)
#define PA13			GPIOA, BIT(13)
#define PA14			GPIOA, BIT(14)
#define PA15			GPIOA, BIT(15)

#define PB0			GPIOB, BIT(0)
#define PB1			GPIOB, BIT(1)
#define PB2			GPIOB, BIT(2)
#define PB3			GPIOB, BIT(3)
#define PB4			GPIOB, BIT(4)
#define PB5			GPIOB, BIT(5)
#define PB6			GPIOB, BIT(6)
#define PB7			GPIOB, BIT(7)
#define PB8			GPIOB, BIT(8)
#define PB9			GPIOB, BIT(9)
#define PB10			GPIOB, BIT(10)
#define PB11			GPIOB, BIT(11)
#define PB12			GPIOB, BIT(12)
#define PB13			GPIOB, BIT(13)
#define PB14			GPIOB, BIT(14)
#define PB15			GPIOB, BIT(15)

#define PC0			GPIOC, BIT(0)
#define PC1			GPIOC, BIT(1)
#define PC2			GPIOC, BIT(2)
#define PC3			GPIOC, BIT(3)
#define PC4			GPIOC, BIT(4)
#define PC5			GPIOC, BIT(5)
#define PC6			GPIOC, BIT(6)
#define PC7			GPIOC, BIT(7)
#define PC8			GPIOC, BIT(8)
#define PC9			GPIOC, BIT(9)
#define PC10			GPIOC, BIT(10)
#define PC11			GPIOC, BIT(11)
#define PC12			GPIOC, BIT(12)
#define PC13			GPIOC, BIT(13)
#define PC14			GPIOC, BIT(14)
#define PC15			GPIOC, BIT(15)

#define PD0			GPIOD, BIT(0)
#define PD1			GPIOD, BIT(1)
#define PD2			GPIOD, BIT(2)
#define PD3			GPIOD, BIT(3)
#define PD4			GPIOD, BIT(4)
#define PD5			GPIOD, BIT(5)
#define PD6			GPIOD, BIT(6)
#define PD7			GPIOD, BIT(7)
#define PD8			GPIOD, BIT(8)
#define PD9			GPIOD, BIT(9)
#define PD10			GPIOD, BIT(10)
#define PD11			GPIOD, BIT(11)
#define PD12			GPIOD, BIT(12)
#define PD13			GPIOD, BIT(13)
#define PD14			GPIOD, BIT(14)
#define PD15			GPIOD, BIT(15)

#define PE0			GPIOE, BIT(0)
#define PE1			GPIOE, BIT(1)
#define PE2			GPIOE, BIT(2)
#define PE3			GPIOE, BIT(3)
#define PE4			GPIOE, BIT(4)
#define PE5			GPIOE, BIT(5)
#define PE6			GPIOE, BIT(6)
#define PE7			GPIOE, BIT(7)
#define PE8			GPIOE, BIT(8)
#define PE9			GPIOE, BIT(9)
#define PE10			GPIOE, BIT(10)
#define PE11			GPIOE, BIT(11)
#define PE12			GPIOE, BIT(12)
#define PE13			GPIOE, BIT(13)
#define PE14			GPIOE, BIT(14)
#define PE15			GPIOE, BIT(15)

#define PF0			GPIOF, BIT(0)
#define PF1			GPIOF, BIT(1)
#define PF2			GPIOF, BIT(2)
#define PF3			GPIOF, BIT(3)
#define PF4			GPIOF, BIT(4)
#define PF5			GPIOF, BIT(5)
#define PF6			GPIOF, BIT(6)
#define PF7			GPIOF, BIT(7)
#define PF8			GPIOF, BIT(8)
#define PF9			GPIOF, BIT(9)
#define PF10			GPIOF, BIT(10)
#define PF11			GPIOF, BIT(11)
#define PF12			GPIOF, BIT(12)
#define PF13			GPIOF, BIT(13)
#define PF14			GPIOF, BIT(14)
#define PF15			GPIOF, BIT(15)

enum gpio_mode_t {
	GPIO_INPUT		= 0,
	GPIO_OUTPUT		= 1,
	GPIO_ALTMODE		= 2,
	GPIO_ANALOG		= 3,
};

enum gpio_out_t {
	PUSHPULL_OUTPUT		= 0,
	OPENDRAIN_OUTPUT	= 1,
};

enum gpio_speed_t {
	GPIO_LOW_SPEED		= 0,
	GPIO_MED_SPEED		= 1,
	GPIO_HIGH_SPEED		= 3,
};

enum gpio_pull_t {
	GPIO_NO_PULL		= 0,
	GPIO_PULL_UP		= 1,
	GPIO_PULL_DOWN		= 2,
};

enum gpio_alt_t {
	GPIO_AF0		= 0,
	GPIO_AF1		= 1,
	GPIO_AF2		= 2,
	GPIO_AF3		= 3,
	GPIO_AF4		= 4,
	GPIO_AF5		= 5,
	GPIO_AF6		= 6,
	GPIO_AF7		= 7,
};

static inline uint16_t gpio_read(GPIO_TypeDef *gpio, uint16_t pin_mask)
{
	return gpio->IDR & pin_mask;
}

static inline void gpio_write(GPIO_TypeDef *gpio, uint16_t pin_mask)
{
	gpio->ODR = pin_mask;
}

static inline void gpio_set(GPIO_TypeDef *gpio, uint16_t pin_mask)
{
	gpio->BSRR = pin_mask;
}

static inline void gpio_clr(GPIO_TypeDef *gpio, uint16_t pin_mask)
{
	gpio->BRR = pin_mask;
}

static inline void gpio_inv(GPIO_TypeDef *gpio, uint16_t pin_mask)
{
	gpio->ODR = gpio->ODR ^ pin_mask;
}

void gpio_set_output(GPIO_TypeDef *gpio, uint8_t pin_num, enum gpio_out_t push);

void gpio_set_speed(GPIO_TypeDef *gpio, uint8_t pin_num,
	enum gpio_speed_t speed);

void gpio_set_pull(GPIO_TypeDef *gpio, uint8_t pin_num,
	enum gpio_pull_t pull);

void gpio_set_mode(GPIO_TypeDef *gpio, uint8_t pin_num, enum gpio_mode_t mode);

void gpio_set_alt_func(GPIO_TypeDef *gpio, uint8_t pin_num,
	enum gpio_alt_t alt_func);

void gpio_output_init(GPIO_TypeDef *gpio, uint16_t pinmask, enum gpio_out_t out,
	enum gpio_speed_t speed);

void gpio_digital_input_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum gpio_pull_t pull);

void gpio_analog_input_init(GPIO_TypeDef *gpio, uint16_t pinmask);

void gpio_alt_func_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum gpio_alt_t alt);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */
