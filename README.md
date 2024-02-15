# STM32F0xx open source highly efficient peripheral driver

## Clock setup
```c
/* Setup clocking from HSIxPLL for 16 MHz (8..48 MHz) */
rcc_enable_hsi_pll(4);
```

## GPIO example
```c
gpio_output_init(PB01, PUSHPULL_OUTPUT, GPIO_LOW_SPEED);

/* set pin HIGH */
gpio_set(PB01);

/* set pin LOW */
gpio_clr(PB01);
```
## UART example
```c
/* pointer to uart settings and buffers */
uart_dev uart;

/* uart rx handler, executed from ISR when data is filled or end of a line detected */
static void uart_rx_handler(char *buffer, uint16_t size, void *data)
{
  /* your code here, data is stored in buffer */
}

/* find uart connected to GPIOB, pin 6 (can be tx or rx pin), init for 9600 bod */
uart_init(&uart, 1, BIT(6), BIT(7), 9600);

/* enabling receiver */
uart_enable_rx(uart, rx_buf, sizeof(rx_buf),
		uart_rx_handler, 0);

/* send data using dma or interrupt depends of dma availability */
uart_send_string(uart, "Hallo world!\r\n");
```
## SPI example
If DMA is available, transfer will be done using DMA. Otherwise interrupt will be used.
```c
/* use SPI1 and initialize for 10MHz and idle clock mode */
spi_dev spi = get_spi_dev(1, 10000000, false);

/* send 16 bytes to 0xEE register of some chip connected to PA4 */
spi_write_reg(spi, PA4, 0xEE, (void *)rx_buf, 16);

/* receive 16 bytes from 0xEE register of some chip connected to PA4 */
spi_read_reg(spi, PA4, 0xEE, (void *)rx_buf, 16);
```
## I2C example
```c
/* init I2C1 in fast mode (true == fast, false == normal)
i2c_dev i2c = i2c = get_i2c_dev(1, true);

/* read 10 bytes of data from MPU6050 address 0x68 register 0xC3 in buffer */
i2c_read_reg(i2c, 0x68, 0xC3, buf, 10);

/* write 10 bytes of data from buffer to MPU6050 at 0x68 address */
i2c_write_reg(i2c, 0x68, 0xC3, buf, 10);
```
## DELAYS
Simply use, do not init anything.
```c
/* wait for 250ms */
delay_ms(250);

/* wait for 250us */
delay_us(250);
```
## ADC example
```c
/* channel0 will be used */
adc_dev adc_ch0 = get_adc_dev(0);

/* convert and read the value */
adc_res = adc_read(adc_ch0, 7);

/* read internal temperature value */
double t = adc_read_temp();

/* read vref value */
double v = adc_read_vref();
```
## Timer example
```c
/* get handler and initialize timer 7 for 2.5kHz */
tim_dev tim7 = get_tim_dev(7, 2500, 0);

/* this code will be run by the interrupt */
static void tim_isr(void *data)
{
	/* invert pin state (LED flash) */
	gpio_inv(PA4);
}

/* enable timer interrupt */
tim_enable_interrupt(tim7, tim_isr, 0);
```
## PWM example
```c
/* use timer 14 with 2500kHz frequency */
tim_dev tim14 = get_tim_dev(14, 2500, 0);

/* set manually the time-base (pre-scaler and period) */
tim_set_timebase(tim14, 1000, 1500);

/* enable PWM output at PB1 or if defined TIM14PWM1_REMAP at PA4 */
tim_pwm_enable(tim14, 1, 1000);

/* change the duty cycle */
tim_pwm_set_duty(tim14, 1, 200);
```
## CRC example
```c
/* calculate CRC32 of whole mcu flash memory */
uint32 crc32 = crc((void *)0x8000000, 0x40000 / 4, sizeof(uint32_t));
```
