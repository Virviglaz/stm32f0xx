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
#define NOF_TIMERS		7
#else
#error "No timer driver available for this mcu."
#endif /* TODO: add support for other mcus */

static struct isr_t {
	void (*handler)(void *data);
	void *data;
} tim_isr[NOF_TIMERS] = { 0 };

static const struct tim_dev_t {
	uint8_t index;			/* index of timer */
	TIM_TypeDef *tim;		/* pointer to tim base address */
	struct isr_t *isr;		/* pointer to isr parameters */
	enum IRQn irq;			/* irq number */
} params[] = {
	{ 1,  TIM1,  &tim_isr[0], TIM1_BRK_UP_TRG_COM_IRQn },
	{ 3,  TIM3,  &tim_isr[1], TIM3_IRQn },
	{ 6,  TIM6,  &tim_isr[2], TIM6_IRQn },
	{ 14, TIM14, &tim_isr[3], TIM14_IRQn },
	{ 15, TIM15, &tim_isr[4], TIM15_IRQn },
	{ 16, TIM16, &tim_isr[5], TIM16_IRQn },
	{ 17, TIM17, &tim_isr[6], TIM17_IRQn },
};

static uint32_t get_tim_clock(uint8_t tim)
{
	struct system_clock_t *clocks = get_clocks();

	switch (tim) {
	case 1:
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
		return clocks->apb2_freq;
	case 3:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
		return clocks->apb1_freq;
	case 6:
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
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

static void isr(uint8_t tim_num)
{
	struct isr_t *isr = &tim_isr[tim_num];
	isr->handler(isr->data);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	isr(0);
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

void TIM14_IRQHandler(void)
{
	isr(3);
	TIM14->SR = 0;
}

void TIM15_IRQHandler(void)
{
	isr(4);
	TIM15->SR = 0;
}

void TIM16_IRQHandler(void)
{
	isr(5);
	TIM16->SR = 0;
}

void TIM17_IRQHandler(void)
{
	isr(6);
	TIM17->SR = 0;
}
