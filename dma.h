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
 
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f0xx.h>
#include <stdint.h>

/**
  * @brief  Initialize DMA and find a channel.
  * @param  channel: Number of channel or first free if 0.
  * @param  handler: Pointer to function to be called at finish.
  * @param  data: Pointer to data to be provided to handler.
  *
  * @retval 0 if failed, pointer to channel if found.
  */
DMA_Channel_TypeDef *get_dma_ch(uint8_t channel,
	void (*handler)(void *data), void *data);

/**
  * @brief  Release DMA channed and disable the interrupt.
  * @param  ch: Pointer to DMA channel.
  *
  * @retval None.
  */
static inline void dma_release(DMA_Channel_TypeDef *ch)
{
	if (ch)
		ch->CCR = 0;
}

#ifdef FREERTOS

/**
  * @brief  Copy data memory to memory using DMA and RTOS.
  * @param  dst: pointer to the destination array.
  * @param  src: pointer to the source of data to be copied.
  * @param  size: number of bytes to copy.
  *
  * @retval 0 if failed, pointer to channel if found.
  */
void memcpy_dma8(uint8_t *dst, const uint8_t *src, uint16_t size);
void memcpy_dma16(uint16_t *dst, const uint16_t *src, uint16_t size);
void memcpy_dma32(uint32_t *dst, const uint32_t *src, uint16_t size);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

