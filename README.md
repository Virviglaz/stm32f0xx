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
uart = find_uart_dev(GPIOB, BIT(6));
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
## DELAYS
Simply use, do not init anything.
```c
/* wait for 250ms */
delay_ms(250);

/* wait for 250us */
delay_us(250);
```
