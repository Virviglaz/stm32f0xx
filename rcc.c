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

/* macro to convert enum to value */
#define AHB_DIV_VALUE(x)		(BIT(x - 7))
#define APB_DIV_VALUE(x)		(BIT(x - 3))

/* fixed high speed rc-based internal oscillator */
#define HSI_CLOCK_FREQ			8000000

/* default high speed rc-based external oscillator */
#define HSE_CLOCK_FREQ			8000000

static uint32_t hse_clock = HSE_CLOCK_FREQ;

static uint32_t get_pll_freq(void)
{
	if (get_pll_source() == HSI_DIV2)
		return HSI_CLOCK_FREQ >> 1;

	return hse_clock / get_pll_prediv();
}

static void rcc_configure_pll(uint32_t value)
{
	/* disable PLL */
	BIT_CLR(RCC->CR, RCC_CR_PLLON);

	/* wait pll disable */
	WAIT_BIT_CLR(RCC->CR, RCC_CR_PLLRDY);

	set_pll_value(value);

	/* enable PLL */
	BIT_SET(RCC->CR, RCC_CR_PLLON);

	/* wait pll enable */
	WAIT_BIT_SET(RCC->CR, RCC_CR_PLLRDY);
}

struct system_clock_t *get_clocks(void)
{
	static struct system_clock_t clocks;
	enum ahb_clock_source_t system_clock_source;

	system_clock_source = get_ahb_clock_source();

	switch (system_clock_source) {
	case HSI_CLOCK:
		clocks.system_freq = HSI_CLOCK_FREQ;
		break;
	case HSE_CLOCK:
		clocks.system_freq = hse_clock;
		break;
	case PLL_CLOCK:
		clocks.system_freq = get_pll_freq() * get_pll_value();
		break;
	}

	/* SYSCLL / [AHB_DIV] = AHB frequency */
	clocks.ahb_freq =
		clocks.system_freq / AHB_DIV_VALUE(get_ahb_clock_divider());

	/* AHB / [APB_DIV] = APB frequency */
	clocks.apb_freq =
		clocks.system_freq / APB_DIV_VALUE(get_apb_clock_divider());

	clocks.ahb_source = get_ahb_clock_source();
	
	return &clocks;
}

void rcc_enable_hsi_pll(uint8_t pll)
{
	set_ahb_clock_source(HSI_CLOCK);

	rcc_configure_pll(pll);

	set_ahb_clock_source(PLL_CLOCK);
}

