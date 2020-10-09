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
#include "tim.h"
#include "gpio.h"
#include <errno.h>

#if defined (STM32F030xC)
#define NOF_TIMERS		8
#else
#error "No timer driver available for this mcu."
#endif /* TODO: add support for other mcus */

static struct isr_t {
	void (*handler)(void *data);
	void *data;
} tim_isr[NOF_TIMERS] = { 0 };

struct pwm_t {
	gp_t *pwm_ch[4];
};

static const gp_t tim1ch1 = { PA8,  GPIO_AF2 };
static const gp_t tim1ch2 = { PA9,  GPIO_AF2 };
static const gp_t tim1ch3 = { PA10, GPIO_AF2 };
static const gp_t tim1ch4 = { PA11, GPIO_AF2 };
static const struct pwm_t tim1_pwm = { &tim1ch1, &tim1ch2, &tim1ch3, &tim1ch4 };

static const gp_t tim3ch1 = { PB4,  GPIO_AF1 };
static const gp_t tim3ch2 = { PB5,  GPIO_AF1 };
static const gp_t tim3ch3 = { PB0,  GPIO_AF1 };
static const gp_t tim3ch4 = { PB1,  GPIO_AF1 };
static const struct pwm_t tim3_pwm = { &tim3ch1, &tim3ch2, &tim3ch3, &tim3ch4 };

#ifdef TIM14PWM1_REMAP
static const gp_t tim14ch1 = { PA4,  GPIO_AF4 };
#else
static const gp_t tim14ch1 = { PB1,  GPIO_AF0 };
#endif
static const struct pwm_t tim14_pwm = { &tim14ch1, 0 };

#if defined (STM32F030xC) || defined (STM32F030X8)
static const gp_t tim15ch1 = { PB14, GPIO_AF0 };
static const struct pwm_t tim15_pwm = { &tim15ch1, 0 };
#endif

#ifdef TIM16PWM1_REMAP
static const gp_t tim16ch1 = { PA6,  GPIO_AF5 };
#else
static const gp_t tim16ch1 = { PB8,  GPIO_AF2 };
#endif
static const struct pwm_t tim16_pwm = { &tim16ch1, 0 };

#ifdef TIM17PWM1_REMAP
static const gp_t tim17ch1 = { PA7,  GPIO_AF5 };
#else
static const gp_t tim17ch1 = { PB9,  GPIO_AF2 };
#endif
static const struct pwm_t tim17_pwm = { &tim17ch1, 0 };

static const struct tim_dev_t {
	uint8_t index;			/* index of timer */
	TIM_TypeDef *tim;		/* pointer to tim base address */
	struct isr_t *isr;		/* pointer to isr parameters */
	const struct pwm_t *pwm;	/* pointer to PWM out channels */
	enum IRQn irq;			/* irq number */
} params[] = {
	{ 1,  TIM1,  &tim_isr[0], &tim1_pwm, 	TIM1_BRK_UP_TRG_COM_IRQn },
	{ 2,  TIM2 },
	{ 3,  TIM3,  &tim_isr[1], &tim3_pwm,	TIM3_IRQn },
	{ 6,  TIM6,  &tim_isr[2], 0,		TIM6_IRQn },
	{ 7,  TIM7,  &tim_isr[3], 0,		TIM7_IRQn },
	{ 14, TIM14, &tim_isr[4], &tim14_pwm,	TIM14_IRQn },
#if defined (STM32F030xC) || defined (STM32F030X8)
	{ 15, TIM15, &tim_isr[5], &tim15_pwm,	TIM15_IRQn },
#else
	{ 15, TIM15, &tim_isr[5], 0,		TIM15_IRQn },
#endif
	{ 16, TIM16, &tim_isr[6], &tim16_pwm,	TIM16_IRQn },
	{ 17, TIM17, &tim_isr[7], &tim17_pwm,	TIM17_IRQn },
};

static uint32_t get_tim_clock(uint8_t tim)
{
	struct system_clock_t *clocks = get_clocks();

	switch (tim) {
	case 1:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
		return clocks->apb2_freq;
	case 2:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
		return clocks->apb1_freq;
	case 3:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
		return clocks->apb1_freq;
	case 6:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
		return clocks->apb1_freq;
	case 7:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
		return clocks->apb1_freq;
	case 14:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
		return clocks->apb1_freq;
	case 15:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM15EN);
		return clocks->apb2_freq;
	case 16:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
		return clocks->apb2_freq;
	case 17:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM17EN);
		return clocks->apb2_freq;
	}

	return 0;
}

tim_dev get_tim_dev(uint8_t tim, uint32_t freq, uint32_t period)
{
	uint32_t clock_freq, prc, arr;
	tim_dev dev = 0;
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(params); i++) {
		dev = &params[i];
		if (tim == dev->index)
			break;
	}

	if (!dev)
		return 0;

	clock_freq = get_tim_clock(dev->index);
	if (!clock_freq)
		return 0;

	if (freq) {
		prc = 0;
		do {
			arr = clock_freq / freq / ++prc;
		} while (arr > 0xFFFF);
	} else {
		prc = 0xFFFF;
		do {
			arr = clock_freq * period / --prc;
		} while (arr > 0xFFFF);
	}

	dev->tim->PSC = prc - 1;
	dev->tim->ARR = arr;
	dev->tim->EGR = TIM_EGR_UG;
	dev->tim->CR1 = TIM_CR1_CEN;

	return dev;
}

void tim_set_timebase(tim_dev dev, uint32_t prc, uint32_t period)
{
	dev->tim->PSC = prc;
	dev->tim->ARR = period;
	dev->tim->EGR = TIM_EGR_UG;
}

void tim_enable_interrupt(tim_dev dev, void (*handler)(void *data), void *data)
{
	struct isr_t *isr = dev->isr;

	isr->handler = handler;
	isr->data = data;
	dev->tim->DIER = TIM_DIER_UIE;

	NVIC_EnableIRQ(dev->irq);
}

void tim_disable_interrupt(tim_dev dev)
{
	dev->tim->DIER = 0;
}

void tim_start(tim_dev dev)
{
	dev->tim->CR1 = TIM_CR1_CEN;
}

void tim_stop(tim_dev dev)
{
	dev->tim->CR1 = 0;
}

uint32_t tim_get_value(tim_dev dev)
{
	return dev->tim->CNT;
}

void tim_set_value(tim_dev dev, uint32_t value)
{
	dev->tim->CNT = value;
	dev->tim->EGR = TIM_EGR_UG;
}

int tim_pwm_enable(tim_dev dev, uint8_t ch_num, uint16_t duty)
{
	const gp_t *pin;
	TIM_TypeDef *tim = dev->tim;

	ch_num--; /* [1..4] -> [0..3] */
	if (!dev->pwm || ch_num > 3)
		return -EINVAL;

	pin = dev->pwm->pwm_ch[ch_num];
	if (!pin)
		return -EINVAL;

	gpio_alt_func_init(pin->gpio, pin->pinmask, pin->alt_func);

	switch (ch_num) {
	case 0:
		tim->CCMR1 = (7 << 4) | TIM_CCMR1_OC1PE;
		tim->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;
		tim->CCR1 = duty;
		break;
	case 1:
		tim->CCMR1 = (7 << 4) | TIM_CCMR1_OC2PE;
		tim->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P;
		tim->CCR2 = duty;
		break;
	case 2:
		tim->CCMR2 = (7 << 4) | TIM_CCMR2_OC3PE;
		tim->CCER = TIM_CCER_CC3E | TIM_CCER_CC3P;
		tim->CCR3 = duty;
		break;
	case 3:
		tim->CCMR2 = (7 << 4) | TIM_CCMR2_OC4PE;
		tim->CCER = TIM_CCER_CC4E | TIM_CCER_CC4P;
		tim->CCR4 = duty;
		break;
	}

	tim->RCR = 0;
	tim->BDTR = TIM_BDTR_MOE;

	return 0;
}

int tim_pwm_set_duty(tim_dev dev, uint8_t ch_num, uint16_t duty)
{
	TIM_TypeDef *tim = dev->tim;

	switch (ch_num) {
	case 1:
		tim->CCR1 = duty;
		break;
	case 2:
		tim->CCR2 = duty;
		break;
	case 3:
		tim->CCR3 = duty;
		break;
	case 4:
		tim->CCR4 = duty;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void isr(uint8_t tim_num)
{
	struct isr_t *isr = &tim_isr[tim_num];
	isr->handler(isr->data);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	isr(0); /* increment index by 1 */
	TIM1->SR = 0;
}

void TIM3_IRQHandler(void)
{
	isr(1);
	TIM3->SR = 0;
}

void TIM6_IRQHandler(void)
{
	isr(2);
	TIM6->SR = 0;
}

void TIM7_IRQHandler(void)
{
	isr(3);
	TIM7->SR = 0;
}

void TIM14_IRQHandler(void)
{
	isr(4);
	TIM14->SR = 0;
}

void TIM15_IRQHandler(void)
{
	isr(5);
	TIM15->SR = 0;
}

void TIM16_IRQHandler(void)
{
	isr(6);
	TIM16->SR = 0;
}

void TIM17_IRQHandler(void)
{
	isr(7);
	TIM17->SR = 0;
}
