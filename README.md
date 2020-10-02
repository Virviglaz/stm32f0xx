# STM32F0xx open source highly efficient peripheral driver

This drivers is a simplified and highly enchansed alternative to the official HAL/SPL ST's drivers.
It is specially made to be used with realtime operational systems as well as with the bare metall platforms.

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

/* find uart connected to GPIOB, pin 6 (can be tx or rx pin) */
uart = find_uart_dev(PB6);;
or
/* use UART1 insted if you know where it is connected to */
uart = get_uart_dev(1);

/* err will be 0 if uart manage to initialize with 9600 bod */
err = uart_init(uart, 9600);

/* enabling receiver */
uart_enable_rx(uart, rx_buf, sizeof(rx_buf),
		uart_rx_handler, 0);

/* send data using dma or interrupt depends of dma availability */
uart_send_string(uart, "Hallo world!\r\n");
```
## SPI example
If DMA is available, transfer will be done using DMA. Otherwise interrupt will be used.
```c
/* use SPI1 */
spi_dev spi = get_spi_dev(1);

/* init SPI1 with 10MHz and idle clock mode */
err = spi_init(spi, 10000000, false);

/* send 16 bytes to 0xEE register of some chip connected to PA4 */
spi_write_reg(spi, PA4, 0xEE, (void *)rx_buf, 16);

/* receive 16 bytes from 0xEE register of some chip connected to PA4 */
spi_read_reg(spi, PA4, 0xEE, (void *)rx_buf, 16);
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
