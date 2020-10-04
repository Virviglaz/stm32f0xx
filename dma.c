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
#include <stdbool.h>

#define NOF_DMA_CHANNELS	5

static struct isr_t {
	void (*handler)(void *data);	/* also used to mark 'occupied' */
	void *data;			/* private data if needed */
} isrs[NOF_DMA_CHANNELS] = { 0 };

const DMA_Channel_TypeDef *dma1_chs[] = {
	DMA1_Channel1,
	DMA1_Channel2,
	DMA1_Channel3,
	DMA1_Channel4,
	DMA1_Channel5,
};

static void dma_enable_isr(void)
{
	static bool is_done = false;

	if (is_done)
		return;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

	is_done = true;
}

DMA_Channel_TypeDef *get_dma_ch(uint8_t channel,
	void (*handler)(void *data), void *data)
{
	DMA_Channel_TypeDef *ch;
	struct isr_t *isr;

	dma_enable_isr();

	if (channel >= ARRAY_SIZE(dma1_chs))
		return 0;

	channel--;
	ch = (void *)dma1_chs[channel];
	isr = &isrs[channel];

	/* check channel is free */
	if (isr->handler)
		return 0;
	/* assign handler */
	isrs[channel].handler = handler;
	isrs[channel].data = data;
	isr->handler = handler;
	isr->data = data;

	return ch;
}


DMA_Channel_TypeDef *find_free_dma_ch(uint8_t *found,
	void (*handler)(void *data), void *data)
{
	uint8_t i;
	struct isr_t *isr;

	/* find first free channel */
	for (i = 0; i != ARRAY_SIZE(dma1_chs); i++) {
		isr = &isrs[i];

		/* check channel is free */
		if (!isr->handler) {
			*found = i;
			return get_dma_ch(i, handler, data);
		}
	}

	/* no channels found */
	return 0;
}

void dma_release(uint8_t ch)
{
	struct isr_t *isr;
	DMA_Channel_TypeDef *dma_ch;

	if (!ch) /* skip dummy action */
		return;

	ch--;
	isr = &isrs[ch];
	dma_ch = (void *)dma1_chs[ch];

	dma_ch->CCR = 0;	/* disable the channel */
	isr->handler = 0;	/* remove the handler and mark 'free' */
}

void DMA1_Channel1_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF1;
	if (isrs[0].handler)
		isrs[0].handler(isrs[0].data);
}

void DMA1_Channel2_3_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	if (isr & DMA_ISR_TCIF2) {
		DMA1->IFCR = DMA_IFCR_CGIF2;
		if (isrs[1].handler)
			isrs[1].handler(isrs[1].data);
	}

	if (isr & DMA_ISR_TCIF3) {
		DMA1->IFCR = DMA_IFCR_CGIF3;
		if (isrs[2].handler)
			isrs[2].handler(isrs[2].data);
	}
}

void DMA1_Channel4_5_IRQHandler(void)
{
	uint32_t isr = DMA1->ISR;
	if (isr & DMA_ISR_TCIF4) {
		DMA1->IFCR = DMA_IFCR_CGIF4;
		if (isrs[3].handler)
			isrs[3].handler(isrs[3].data);
	}

	if (isr & DMA_ISR_TCIF5) {
		DMA1->IFCR = DMA_IFCR_CGIF5;
		if (isrs[4].handler)
			isrs[4].handler(isrs[4].data);
	}
}

#ifndef FREERTOS

static volatile int busy;
static void handler(void *data)
{
	busy = 0;
}

void memcpy_dma(void *dst, const void *src, uint16_t size, uint16_t flag)
{
	uint8_t dma_num;
	DMA_Channel_TypeDef *ch = find_free_dma_ch(&dma_num, handler, 0);

	if (!ch) /* failed to get channel, use cpu instead */
		memcpy(dst, src, size);
	else {
		busy = !0;
		dma_setup(ch, (void *)src, dst, size, DMA_MEM2MEM_B | flag);
		while(busy) { }
		dma_release(dma_num);
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
	uint8_t ch_num;

	if (!mutex)
		mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(mutex, portMAX_DELAY);

	handle = xTaskGetCurrentTaskHandle();

	ch = find_free_dma_ch(&ch_num, handler, handle);

	if (!ch) /* failed to get channel, use cpu instead */
		memcpy(dst, src, size);
	else {
		dma_setup(ch, (void *)src, dst, size, DMA_MEM2MEM_B | flag);
		vTaskSuspend(handle);
		dma_release(ch_num);
	}

	xSemaphoreGive(mutex);
}

#endif /* FREERTOS */

