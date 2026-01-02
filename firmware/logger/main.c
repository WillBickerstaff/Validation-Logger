#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>

#include "timer1_capture.h"

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
static void uart_init(void) {
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
static void uart_putc(char c) {
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
static void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

/*
 * Transmit an unsigned 32-bit integer as decimal ASCII.
 *
 * Used for log headers and event records.
 */
static void uart_put_uint32(uint32_t value) {
    char buf[11];
    uint8_t i = 0;

    if (value == 0) {
        uart_putc('0');
        return;
    }

    while (value > 0 && i < sizeof(buf)) {
        buf[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (i > 0) {
        uart_putc(buf[--i]);
    }
}

/*
 * Transmit an unsigned 16-bit integer as decimal ASCII.
 */
static void uart_put_uint16(uint16_t value) {
    uart_put_uint32((uint32_t)value);
}

/* Logging active indicator LED on PD7 */
#define LOG_LED_PORT  PORTD
#define LOG_LED_DDR   DDRD
#define LOG_LED_BIT   PD7

/* SW2 is active-low on PB1 with internal pull-up enabled. */
#define SW2_PORT   PORTB
#define SW2_PINR   PINB
#define SW2_DDR    DDRB
#define SW2_BIT    PB1

/*
 * Debounce lockout in Timer1 ticks.
 *
 * Timer1 is configured by timer1_capture_init().
 * TIMER1_PRESCALER=1, so we assume tick rate is F_CPU.
 * 50 ms => F_CPU / 20.
 */
#define SW2_DEBOUNCE_TICKS  (F_CPU / 20UL)

int main(void) {
    /*
     * Minimal firmware bring-up.
     * No interrupts, no timers, no sensor I/O at this stage.
     */
    uart_init();

    /*
     * Emit self-describing log header.
     * Printed once at startup before any timed or interrupt-driven activity.
     */
    uart_puts("# validation-logger\r\n");

    uart_puts("# F_CPU=");
    uart_put_uint32(F_CPU);
    uart_puts("\r\n");

    uart_puts("# TIMER1_PRESCALER=1\r\n");

    #if TIMER1_CAPTURE_USE_NOISE_CANCEL
        uart_puts("# ICNC1=ON\r\n");
    #else
        uart_puts("# ICNC1=OFF\r\n");
    #endif

    uart_puts("# CAPTURE_BUFFER_SIZE=");
    uart_put_uint16(CAPTURE_BUFFER_SIZE);
    uart_puts("\r\n");

    uart_puts("# ---\r\n");

    /*
     * Configure SW2 (PB1) as input with internal pull-up.
     * Do this after banner output to keep headers clean.
     */
    SW2_DDR &= (uint8_t)~_BV(SW2_BIT);
    SW2_PORT |= _BV(SW2_BIT);

    /* Configure logging indicator LED (PD7) as output, initially OFF */
    LOG_LED_DDR |= _BV(LOG_LED_BIT);
    LOG_LED_PORT &= (uint8_t)~_BV(LOG_LED_BIT);

    /*
     * Start Timer1 capture after headers.
     * Capture runs continuously; SW2 only gates printing.
     */
    timer1_capture_init();
    sei();

    bool logging = false;
    bool sw2_prev = true;  /* pulled-up = released */
    uint32_t sw2_lockout_until = 0;
    uint32_t last_tick = 0;
    uint32_t next_heartbeat = 0;

    for (;;) {
        uint32_t now = timer1_capture_now();

        /* ---- SW2 press-to-toggle (active-low) ---- */
        bool sw2_now = (SW2_PINR & _BV(SW2_BIT)) != 0;

        if (!sw2_now && sw2_prev && now >= sw2_lockout_until) {
            logging = !logging;
            sw2_lockout_until = now + (uint32_t)SW2_DEBOUNCE_TICKS;

            if (logging) {
                LOG_LED_PORT |= _BV(LOG_LED_BIT);   /* LED ON */
                uart_puts("# START\r\n");
                uart_puts("ticks,edge,dt_ticks,dropped\r\n");
                last_tick = 0;

                /* Drain any queued events at start-of-run boundary. */
                {
                    capture_event_t ev_discard;
                    while (timer1_capture_pop(&ev_discard)) {
                        /* discard */
                    }
                }
            } else {
                LOG_LED_PORT &= (uint8_t)~_BV(LOG_LED_BIT);  /* LED OFF */
                uart_puts("# STOP\r\n");
            }
        }

        sw2_prev = sw2_now;

        /* ---- Optional heartbeat when NOT logging ---- */
        if (!logging) {
            if (now >= next_heartbeat) {
                uart_puts("alive\r\n");
                next_heartbeat = now + (uint32_t)F_CPU;
            }
        }

        /* ---- Drain capture buffer ---- */
        {
            capture_event_t ev;
            while (timer1_capture_pop(&ev)) {
                if (!logging) {
                    continue;
                }

                uint32_t dt = 0;
                if (last_tick != 0) {
                    dt = ev.ticks - last_tick;
                }
                last_tick = ev.ticks;

                uart_put_uint32(ev.ticks);
                uart_putc(',');
                uart_putc((ev.edge == CAPTURE_EDGE_RISING) ? 'R' : 'F');
                uart_putc(',');
                uart_put_uint32(dt);
                uart_putc(',');
                uart_put_uint16(timer1_capture_dropped());
                uart_puts("\r\n");
            }
        }
    }
}
