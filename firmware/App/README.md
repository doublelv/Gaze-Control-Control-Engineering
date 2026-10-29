# firmware/App

Handwritten modules (portable C99 except `bsp/bsp_stm32h7.c`).

Execution context:
- `app_uart_rx_byte()`: UART RX interrupt, only pushes into a ring buffer.
- `app_poll()`: main loop, parses frames, replies, applies outputs, enforces watchdog + endstops.

Initialisation order: CubeMX `MX_*_Init()` -> `bsp_start()` -> `app_init()`.
Errors: malformed frames are dropped and counted (`rx_errors`, `RX_ERR` flag); nothing is retried on the MCU side.
