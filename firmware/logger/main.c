#include <avr/io.h>
#include <util/delay.h>


/*
 * UART configuration.
 * This logger only transmits; RX is intentionally unused.
 */
#define BAUD 115200
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

/*
 * Initialise UART0 for simple debug output.
 *
 * Declared static to restrict visibility to this file.
 * UART is an internal diagnostic mechanism, not a module interface.
 */
static void uart_init(void)
{
    /* Set baud rate */
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);

    /* Enable transmitter only */
    UCSR0B = (1 << TXEN0);

    /* 8 data bits, 1 stop bit, no parity */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/*
 * Transmit a single character over UART.
 *
 * Blocking by design: this is a bootstrap / diagnostic path only.
 * Timing-critical code will not call this directly.
 */
static void uart_putc(char c)
{
    /* Wait for transmit buffer to be empty */
    while (!(UCSR0A & (1 << UDRE0))) {
        /* intentional busy-wait */
    }

    UDR0 = c;
}

/*
 * Transmit a null-terminated string over UART.
 *
 * Convenience wrapper used only during early bring-up.
 * Not suitable for high-rate or time-critical logging.
 */
static void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

int main(void)
{
    /*
     * Minimal firmware bring-up.
     * No interrupts, no timers, no sensor I/O at this stage.
     */
    uart_init();

    uart_puts("\r\nValidation Logger boot\r\n");

    /*
     * Heartbeat loop.
     * Provides a visible proof that the MCU is alive,
     * clock is running correctly, and UART is functional.
     */
    for (;;) {
        uart_puts("alive\r\n");
        _delay_ms(1000);
    }
}
