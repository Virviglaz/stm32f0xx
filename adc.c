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
#include "adc.h"
#include "gpio.h"

static const struct adc_dev_t {
	uint8_t index;			/* index of channel i.e. 1..16 */
	GPIO_TypeDef *gpio;		/* pointer to gpio base address */
	uint8_t pin;			/* ADC pin bit mask BIT(x) */
} pins[] = {
	{ 0,  GPIOA, 0 },
	{ 1,  GPIOA, 1 },
	{ 2,  GPIOA, 2 },
	{ 3,  GPIOA, 3 },
	{ 4,  GPIOA, 4 },
	{ 5,  GPIOA, 5 },
	{ 6,  GPIOA, 6 },
	{ 7,  GPIOA, 7 },
	{ 8,  GPIOB, 0 },
	{ 9,  GPIOB, 1 },
	{ 10, GPIOC, 0 },
	{ 11, GPIOC, 1 },
	{ 12, GPIOC, 2 },
	{ 13, GPIOC, 3 },
	{ 14, GPIOC, 4 },
	{ 15, GPIOC, 5 },
};

static void init(void)
{
	static bool init_done = false;

	if (init_done)
		return;

	BIT_SET(RCC->APB2ENR, RCC_APB2ENR_ADCEN);

	init_done = true;

	ADC1->CR = ADC_CR_ADCAL;
	WAIT_BIT_CLR(ADC1->CR, ADC_CR_ADCAL);

	ADC1->CFGR1 = 0;
	ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;
}

adc_dev find_adc_dev(GPIO_TypeDef *gpio, uint8_t pin)
{
	uint8_t i;

	for (i = 0; i != ARRAY_SIZE(pins); i++) {
		const struct adc_dev_t *dev = &pins[i];
		if (dev->gpio == gpio && dev->pin == pin)
			return get_adc_dev(i);
	}

	return 0;
}

adc_dev get_adc_dev(uint8_t num)
{
	const struct adc_dev_t *dev = num < ARRAY_SIZE(pins) ? &pins[num] : 0;

	if (!dev)
		return 0;

	init();
	gpio_set_mode(dev->gpio, dev->pin, GPIO_ANALOG);
	gpio_set_pull(dev->gpio, dev->pin, GPIO_NO_PULL);

	return dev;
}

uint16_t adc_read(adc_dev dev, uint8_t sample_rate)
{
	ADC1->SMPR = sample_rate & ADC_SMPR1_SMPR;
	ADC1->CHSELR = BIT(dev->index);
	ADC1->CR = ADC_CR_ADEN | ADC_CR_ADSTART;

	WAIT_BIT_SET(ADC1->ISR, ADC_ISR_EOC);
	ADC1->CR = 0;

	return ADC1->DR;
}

double adc_read_vref(void)
{
	static const struct adc_dev_t vref = { 17, 0, 0 };
	uint16_t v;
	const uint16_t *cal = (void *)0x1FFFF7BA;
	init();

	ADC->CCR = ADC_CCR_VREFEN;
	v = adc_read(&vref, ADC_SMPR1_SMPR);
	ADC->CCR = 0;

	return 3.3 * (double)(*cal) / (double)v;
}

/*
 * CPU FREQ	   EXECUTION TIME
 * 8MHz			644uS
 * 12MHz		429uS
 * 16MHz		322uS
 * 24MHz		214uS
 * 32MHz		160uS
*/
double adc_read_temp(void)
{
	static const struct adc_dev_t sensor = { 16, 0, 0 };
	uint16_t v;
	const uint16_t *ts1 = (void *)0x1FFFF7B8;
	const uint16_t *ts2 = (void *)0x1FFFF7C2;
	init();

	ADC->CCR = ADC_CCR_TSEN;
	v = adc_read(&sensor, ADC_SMPR1_SMPR);
	ADC->CCR = 0;

	return adc_read_vref() * \
		((v - *ts1) * (110.0 - 30.0) / (*ts2 - *ts1) + 30.0) / 3.3;
}

/*
 * CPU FREQ	   EXECUTION TIME
 * 8MHz			510uS
 * 12MHz		342uS
 * 16MHz		255uS
 * 24MHz		170uS
 * 32MHz		128uS
*/
double adc_read_voltage(adc_dev dev)
{
	return adc_read_vref() * adc_read(dev, ADC_SMPR1_SMPR) / 4095.0;
}
