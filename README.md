# STM32F0xx open source highly efficient peripheral driver

This drivers is a simplified and highly enchansed alternative to the official HAL/SPL ST's drivers.
It is specially made to be used with realtime operational systems as well as with the bare metall platforms.

## GPIO example
```c
gpio_output_init(PB01, PUSHPULL_OUTPUT, GPIO_LOW_SPEED);

/* set pin HIGH */
gpio_set(PB01);

/* set pin LOW */
gpio_clr(PB01);
```
