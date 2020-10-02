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

#include "crc.h"

static inline void push_data8(uint8_t *data, uint32_t size)
{
	while (size--)
		CRC->DR = *data++;
}

static inline void push_data16(uint16_t *data, uint32_t size)
{
	while (size--)
		CRC->DR = *data++;
}

static inline void push_data32(uint32_t *data, uint32_t size)
{
	while (size--)
		CRC->DR = *data++;
}

/*
 * Calculation CRC32 of 65536 x 32b = 0x40000 bytes
 * CPU FREQ	   EXECUTION TIME	RESULT
 * 8MHz			196.5ms		1.3 MB/s
 * 12MHz		131.0ms		1.9 MB/s
 * 16MHz		98.26mS		2.5 MB/s
 * 24MHz		65.50ms		3.7 MB/s
 * 32MHz		49.12ms		5.0 MB/s
*/

uint32_t crc(void *data, uint32_t size, const uint8_t data_size)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN;
	CRC->CR = CRC_CR_RESET;

	switch (data_size) {
	case sizeof(uint8_t):
		push_data8(data, size);
		break;
	case sizeof(uint16_t):
		push_data16(data, size);
		break;
	case sizeof(uint32_t):
		push_data32(data, size);
		break;
	}

	return CRC->DR;
}
