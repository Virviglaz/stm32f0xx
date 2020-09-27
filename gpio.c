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
 * STM32F10x open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "gpio.h"

static void enable_rcc(GPIO_TypeDef *gpio)
{
	if (gpio == GPIOA)
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	else if(gpio == GPIOB)
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if(gpio == GPIOC)
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if(gpio == GPIOD)
		RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	else if(gpio == GPIOE)
		RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	else if(gpio == GPIOF)
		RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
}

void gpio_set_mode(GPIO_TypeDef *gpio, uint8_t pin_num, enum gpio_mode_t mode)
{
	pin_num <<= 1;
	gpio->MODER &= ~(3 << pin_num);
	gpio->MODER |= (mode << pin_num);
}

void gpio_set_output(GPIO_TypeDef *gpio, uint8_t pin_num, enum gpio_out_t push)
{
	gpio->OTYPER = push == OPENDRAIN_OUTPUT ? gpio->OTYPER | BIT(pin_num) :
		gpio->OTYPER & ~BIT(pin_num);
}

void gpio_set_speed(GPIO_TypeDef *gpio, uint8_t pin_num,
	enum gpio_speed_t speed)
{
	pin_num <<= 1;
	gpio->OSPEEDR &= ~(3 << pin_num);
	gpio->OSPEEDR |= (speed << pin_num);
}

void gpio_set_pull(GPIO_TypeDef *gpio, uint8_t pin_num,
	enum gpio_pull_t pull)
{
	pin_num <<= 1;
	gpio->PUPDR &= ~(3 << pin_num);
	gpio->PUPDR |= (pull << pin_num);
}

void gpio_set_alt_func(GPIO_TypeDef *gpio, uint8_t pin_num,
	enum gpio_alt_t alt_func)
{
	if (pin_num < 8) {
		pin_num <<= 2;
		gpio->AFR[0] &= ~(alt_func << pin_num);
		gpio->AFR[0] |= alt_func << pin_num;
	} else {
		pin_num -= 8;
		pin_num <<= 2;
		gpio->AFR[1] &= ~(alt_func << pin_num);
		gpio->AFR[1] |= alt_func << pin_num;
	}
}

void gpio_output_init(GPIO_TypeDef *gpio, uint16_t pinmask, enum gpio_out_t out,
	enum gpio_speed_t speed)
{
	uint8_t i;

	enable_rcc(gpio);

	for (i = 0; i != 16; i++)
		if (pinmask & BIT(i)) {
			gpio_set_mode(gpio, i, GPIO_OUTPUT);
			gpio_set_output(gpio, i, out);
			gpio_set_speed(gpio, i, speed);
		}
}

void gpio_digital_input_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum gpio_pull_t pull)
{
	uint8_t i;

	enable_rcc(gpio);

	for (i = 0; i != 16; i++)
		if (pinmask & BIT(i)) {
			gpio_set_mode(gpio, i, GPIO_INPUT);
			gpio_set_pull(gpio, i, pull);
		}
}

void gpio_analog_input_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum gpio_pull_t pull)
{
	uint8_t i;

	enable_rcc(gpio);

	for (i = 0; i != 16; i++)
		if (pinmask & BIT(i)) {
			gpio_set_mode(gpio, i, GPIO_ANALOG);
			gpio_set_pull(gpio, i, pull);
		}
}

void gpio_alt_func_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum gpio_alt_t alt)
{
	uint8_t i;

	enable_rcc(gpio);

	for (i = 0; i != 16; i++)
		if (pinmask & BIT(i)) {
			gpio_set_mode(gpio, i, GPIO_ALTMODE);
			gpio_set_alt_func(gpio, i, alt);
			gpio_set_speed(gpio, i, GPIO_HIGH_SPEED);
		}
}
