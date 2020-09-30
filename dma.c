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

#include "dma.h"
#include <string.h>

#define NOF_DMA_CHANNELS	5

static struct isr_t {
	void (*handler)(void *data);
	void *data;
} isrs[NOF_DMA_CHANNELS] = { 0 };

const DMA_Channel_TypeDef *dma1_chs[] = {
	DMA1_Channel1,
	DMA1_Channel2,
	DMA1_Channel3,
	DMA1_Channel4,
	DMA1_Channel5,
};

static void dma_enable_isr(uint8_t channel)
{
	IRQn_Type IRQn;

	switch (channel) {
	case 0:
		IRQn = DMA1_Channel1_IRQn;
		break;
	case 1:
		IRQn = DMA1_Channel2_3_IRQn;
		break;
	case 2:
		IRQn = DMA1_Channel2_3_IRQn;
		break;
	case 3:
		IRQn = DMA1_Channel4_5_IRQn;
		break;
	case 4:
		IRQn = DMA1_Channel4_5_IRQn;
		break;
	default:
		return;
	}
	NVIC_EnableIRQ(IRQn);
}

DMA_Channel_TypeDef *get_dma_ch(uint8_t channel,
	void (*handler)(void *data), void *data)
{

	uint8_t i;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	if (channel > ARRAY_SIZE(dma1_chs))
		return 0;

	/* find specified channel */
	if (channel) {
		DMA_Channel_TypeDef *ch;
		channel--;
		ch = (void *)dma1_chs[channel];

		/* check channel is free */
		if (ch->CCR & DMA_CCR_EN)
			return 0;

		/* assign handler */
		isrs[channel].handler = handler;
		isrs[channel].data = data;

		dma_enable_isr(channel);

		return ch;
	}

	/* find first free channel */
	for (i = 0; i != ARRAY_SIZE(dma1_chs); i++) {
		DMA_Channel_TypeDef *ch = (void *)dma1_chs[i];
		if (ch->CCR & DMA_CCR_EN)
			continue;

		/* assign handler */
		isrs[i].handler = handler;
		isrs[i].data = data;

		dma_enable_isr(i);
		return ch;
	}

	/* no channels found */
	return 0;
}

void DMA1_Channel1_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF1;
	dma_release(DMA1_Channel1);
	isrs[0].handler(isrs[0].data);
}

void DMA1_Channel2_3_IRQHandler(void)
{
	uint8_t ch_num = DMA1->ISR & DMA_ISR_TCIF2 ? 1 : 2;
	DMA1->IFCR = ch_num == 1 ? DMA_IFCR_CGIF2 : DMA_IFCR_CGIF3;
	isrs[ch_num].handler(isrs[ch_num].data);
	dma_release((void *)dma1_chs[ch_num]);
}

void DMA1_Channel4_5_IRQHandler(void)
{
	uint8_t ch_num = DMA1->ISR & DMA_ISR_TCIF4 ? 3 : 4;
	DMA1->IFCR = ch_num == 1 ? DMA_IFCR_CGIF4 : DMA_IFCR_CGIF5;
	isrs[ch_num].handler(isrs[ch_num].data);
	dma_release((void *)dma1_chs[ch_num]);
}

#ifndef FREERTOS

static volatile int busy;
static void handler(void *data)
{
	busy = 0;
}

static void memcpy_dma(void *dst, const void *src, uint16_t size, uint16_t flag)
{
	DMA_Channel_TypeDef *ch;

	ch = get_dma_ch(0, handler, 0);

	if (!ch) /* failed to get channel, use cpu instead */
		memcpy(dst, src, size);
	else {
		busy = !0;
		ch->CNDTR = size;
		ch->CMAR = (uint32_t)src;
		ch->CPAR = (uint32_t)dst;
		ch->CCR = DMA_CCR_MEM2MEM | DMA_CCR_MINC | DMA_CCR_PINC | \
			DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_EN | flag;
		while(busy) { }
		dma_release(ch);
	}
}

#else /* FREERTOS */

static void handler(void *data)
{
	rtos_schedule_isr(data);
}

static void memcpy_dma(void *dst, const void *src, uint16_t size, uint16_t flag)
{
	static SemaphoreHandle_t mutex = 0;
	DMA_Channel_TypeDef *ch;
	TaskHandle_t handle;

	if (!mutex)
		mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(mutex, portMAX_DELAY);

	handle = xTaskGetCurrentTaskHandle();

	ch = get_dma_ch(0, handler, handle);

	if (!ch) /* failed to get channel, use cpu instead */
		memcpy(dst, src, size);
	else {
		ch->CNDTR = size;
		ch->CMAR = (uint32_t)src;
		ch->CPAR = (uint32_t)dst;
		ch->CCR = DMA_CCR1_MEM2MEM | DMA_CCR1_MINC | DMA_CCR1_PINC | \
			DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN | flag;
		vTaskSuspend(handle);
		dma_release(ch);
	}

	xSemaphoreGive(mutex);
}

#endif /* FREERTOS */

void memcpy_dma8(uint8_t *dst, const uint8_t *src, uint16_t size)
{
	memcpy_dma(dst, src, size, 0);
}

void memcpy_dma16(uint16_t *dst, const uint16_t *src, uint16_t size)
{
	memcpy_dma(dst, src, size, DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0);
}

void memcpy_dma32(uint32_t *dst, const uint32_t *src, uint16_t size)
{
	memcpy_dma(dst, src, size, DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1);
}
