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
 
#ifndef __RCC_H__
#define __RCC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f0xx.h>
#include <stdint.h>

enum pll_input_clock_src_t {
	HSI_DIV2	= 0,
	HSE_PREDIV	= 1,
};

enum ahb_clock_source_t {
	HSI_CLOCK	= 0,
	HSE_CLOCK	= 1,
	PLL_CLOCK	= 2,
};

enum ahb_clock_divider_t {
	AHB_DIV_1	= 7,
	AHB_DIV_2	= 8,
	AHB_DIV_4	= 9,
	AHB_DIV_8	= 10,
	AHB_DIV_16	= 11,
	AHB_DIV_64	= 12,
	AHB_DIV_128	= 13,
	AHB_DIV_256	= 14,
	AHB_DIV_512	= 15,
};

enum apb_clock_divider_t {
	APB_DIV_1	= 3,
	APB_DIV_2	= 4,
	APB_DIV_4	= 5,
	APB_DIV_8	= 6,
	APB_DIV_16	= 7,
};

struct system_clock_t {
	uint32_t system_freq;
	uint32_t ahb_freq;
	uint32_t apb_freq;
	enum ahb_clock_source_t ahb_source;
};

/**
  * @brief  Get the PLL value.
  * @retval PLL value [2..16].
  */
static inline uint32_t get_pll_value(void)
{
	return ((RCC->CFGR & RCC_CFGR_PLLMUL) >> 18) + 2;
}

/**
  * @brief  Set the PLL value.
  * @param  value: PLL value [2..16].
  * @retval None
  */
static inline void set_pll_value(uint32_t value)
{
	RCC->CFGR &= ~RCC_CFGR_PLLMUL;
	RCC->CFGR |= (value - 2) << 18;
}

/**
  * @brief  Get the PLL predivider value.
  * @retval value: PLL predivider value [1..16].
  */
static inline uint32_t get_pll_prediv(void)
{
	return (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
}

/**
  * @brief  Set the PLL predivider value.
  * @param  value: PLL value [1..16].
  * @retval None
  */
static inline void set_pll_prediv(uint32_t value)
{
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV1;
	RCC->CFGR2 |= value - 1;
}

/**
  * @brief  Set the PLL clock source.
  * @param  src: HSI_DIV2 or HSE_PREDIV.
  * @retval None
  */
static inline void set_pll_source(enum pll_input_clock_src_t src)
{
	if (src == HSE_PREDIV)
		RCC->CFGR |= RCC_CFGR_PLLSRC_1;
	else
		RCC->CFGR &= ~RCC_CFGR_PLLSRC_1;
}

/**
  * @brief  Get the PLL clock source.
  * @retval HSI_DIV2 or HSE_PREDIV.
  */
static inline enum pll_input_clock_src_t get_pll_source(void)
{
	return RCC->CFGR & RCC_CFGR_PLLSRC_1 ? HSE_PREDIV : HSI_DIV2;
}

/**
  * @brief  Set the AHB clock source.
  * @param  src: HSI_CLOCK, HSE_CLOCK or PLL_CLOCK.
  * @retval None
  */
static inline void set_ahb_clock_source(enum ahb_clock_source_t src)
{
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= src;
}

/**
  * @brief  Get the AHB clock source.
  * @retval HSI_CLOCK, HSE_CLOCK or PLL_CLOCK.
  */
static inline enum ahb_clock_source_t get_ahb_clock_source(void)
{
	return (RCC->CFGR & RCC_CFGR_SWS) >> 2;
}

/**
  * @brief  Set the AHB clock divider.
  * @param  div: AHB_DIV_1..AHB_DIV_512
  * @retval None
  */
static inline void set_ahb_clock_divider(enum ahb_clock_divider_t div)
{
	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= div << 4;
}

/**
  * @brief  Get the AHB clock divider.
  * @retval AHB_DIV_1..AHB_DIV_512.
  */
static inline enum ahb_clock_divider_t get_ahb_clock_divider(void)
{
	uint32_t ret = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
	return ret < AHB_DIV_1 ? AHB_DIV_1 : ret;
}

/**
  * @brief  Set the APB clock divider.
  * @param  div: APB_DIV_1..APB_DIV_16
  * @retval None
  */
static inline void set_apb_clock_divider(enum apb_clock_divider_t div)
{
	RCC->CFGR &= ~RCC_CFGR_PPRE;
	RCC->CFGR |= div << 8;
}

/**
  * @brief  Get the AHB clock divider.
  * @retval APB_DIV_1..APB_DIV_16.
  */
static inline enum apb_clock_divider_t get_apb_clock_divider(void)
{
	uint32_t ret = (RCC->CFGR & RCC_CFGR_PPRE) >> 8;
	return ret < APB_DIV_1 ? APB_DIV_1 : ret;
}

/**
  * @brief  Calculate and return all available clock sources.
  * @retval Pointer to system_clock_t struct.
  */
struct system_clock_t *get_clocks(void);

/**
  * @brief  Switch to pll clock source.
  * @param  pll: new pll value [2..16]
  * @retval None. * TODO: return error from clock guard *
  */
void rcc_enable_hsi_pll(uint8_t pll);

/* TODO: enable HSE and HSE_PLL */
/* I can't test it so skip for now */

#ifdef __cplusplus
}
#endif

#endif /* __RCC_H__ */

