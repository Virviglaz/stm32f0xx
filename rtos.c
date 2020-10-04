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

#ifdef FREERTOS

#include "rtos.h"

/*
 * Centralised Deferred Interrupt Handling
 * is so called because each interrupt that uses this method
 * executes in the context of the same RTOS daemon task.
 * https://www.freertos.org/deferred_interrupt_processing.html
 */

#define QUEUE_SIZE		10

static QueueHandle_t isr_queue = 0;
static void (*rtos_err_handler)(const char *err);

static void isr_daemon_task(void *par)
{
	while (1) {
		TaskHandle_t handle;
		if (xQueueReceive(isr_queue, &handle, portMAX_DELAY) == pdPASS)
			vTaskResume(handle);
	}
}

void rtos_deferred_isr_init(void (*err_handler)(const char *err))
{
	rtos_err_handler = err_handler;
	if (!isr_queue) {
		isr_queue = xQueueCreate(QUEUE_SIZE, sizeof(TaskHandle_t));
		xTaskCreate(isr_daemon_task, "isr", MIN_STACK_SIZE, 0, \
			tskIDLE_PRIORITY, 0);

	}
}

void rtos_schedule_isr(TaskHandle_t handle)
{
	BaseType_t hiprio_flag;

	if (xQueueSendFromISR(isr_queue, &handle, &hiprio_flag) != pdTRUE) {
		if (rtos_err_handler)
			rtos_err_handler("isr_queue overflow");
		else {
			portENTER_CRITICAL();
			while (1) { }
		}
	}

	portYIELD_FROM_ISR(hiprio_flag);
}

#endif /* FREERTOS */
