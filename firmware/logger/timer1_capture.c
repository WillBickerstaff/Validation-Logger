#include "timer1_capture.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Timer1 input capture noise canceller (ICNC1).
// Enable to suppress short glitches; disable to observe raw edge timing
// and signal integrity effects during validation.
#ifndef TIMER1_CAPTURE_USE_NOISE_CANCEL
#define TIMER1_CAPTURE_USE_NOISE_CANCEL 1
#endif

// Ring buffer for capture events. Size must be a power of two for fast masking.
#define CAPTURE_BUFFER_MASK (CAPTURE_BUFFER_SIZE - 1)

static capture_event_t capture_buffer[CAPTURE_BUFFER_SIZE];
static volatile uint8_t buffer_head = 0;
static volatile uint8_t buffer_tail = 0;
static volatile uint16_t dropped_events = 0;
static volatile uint16_t timer1_overflow_hi = 0;

// Enforce Ring buffer power of two
_Static_assert((CAPTURE_BUFFER_SIZE & (CAPTURE_BUFFER_SIZE - 1)) == 0,
               "CAPTURE_BUFFER_SIZE must be a power of two");

// Enforce capture size <= 256
_Static_assert(CAPTURE_BUFFER_SIZE <= 256,
               "CAPTURE_BUFFER_SIZE must be <= 256 when using uint8_t indices");

void timer1_capture_init(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        buffer_head = 0;
        buffer_tail = 0;
        dropped_events = 0;
        timer1_overflow_hi = 0;
    }

    /* Stop Timer1 during configuration */
    TCCR1B = 0;
    TCCR1A = 0;
    TCNT1 = 0;

    /* Clear pending flags: input capture + overflow */
    TIFR1 = _BV(ICF1) | _BV(TOV1);

    /* Optional input capture noise filtering */
    uint8_t tccr1b = _BV(ICES1) | _BV(CS10);
#if TIMER1_CAPTURE_USE_NOISE_CANCEL
    tccr1b |= _BV(ICNC1);
#endif

    /* Rising edge + no prescaler (+ optional noise cancel) */
    TCCR1B = tccr1b;

    /* Enable input capture interrupt */
    TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);
}

/*
 * Check whether at least one captured event is available in the ring buffer.
 *
 * This function provides a non-blocking hint to the caller and does not
 * consume any data. Access to the head and tail indices is wrapped in an
 * atomic block to ensure a coherent snapshot with respect to the capture ISR.
 *
 * Note: This function is optional; callers may instead repeatedly call
 * timer1_capture_pop(), which performs its own empty check atomically.
 */
bool timer1_capture_available(void) {
    bool available;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        available = (buffer_head != buffer_tail);
    }

    return available;
}

/*
 * Pop the oldest capture event from the ring buffer.
 *
 * The pop operation is performed atomically to prevent concurrent modification
 * of the buffer indices by the input capture ISR. This avoids torn reads of
 * multi-byte event data (e.g. the 16-bit timer value).
 *
 * The atomic section is deliberately kept very short to minimise the time
 * during which interrupts are masked. Given the expected event rates, this
 * does not materially increase the risk of missed captures.
 *
 * Returns true if an event was retrieved, or false if the buffer was empty.
 */
bool timer1_capture_pop(capture_event_t *out_event) {
    bool ok = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (buffer_head != buffer_tail) {
            const uint8_t tail = buffer_tail;
            *out_event = capture_buffer[tail];
            buffer_tail = (tail + 1) & CAPTURE_BUFFER_MASK;
            ok = true;
        }
    }

    return ok;
}

/*
 * Return the number of capture events dropped due to ring buffer overflow.
 *
 * This counter is incremented within the input capture ISR when the buffer
 * is full and a new event cannot be queued. The value wraps naturally at
 * 65535.
 *
 * The read is performed atomically to guarantee a coherent snapshot when
 * accessed from non-interrupt context.
 */
uint16_t timer1_capture_dropped(void) {
    uint16_t val;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        val = dropped_events;
    }

    return val;
}

ISR(TIMER1_OVF_vect) {
    timer1_overflow_hi++;
}

/*
 * Timer1 Input Capture Interrupt Service Routine.
 *
 * This ISR is invoked on each input capture event on ICP1. Its sole
 * responsibilities are to:
 *   - record the captured timer value and edge polarity,
 *   - enqueue the event into a fixed-size ring buffer, or
 *   - account for the event as dropped if the buffer is full.
 *
 * The ISR is intentionally kept short and deterministic. No blocking
 * operations, logging, or dynamic behaviour are permitted here, as this
 * would directly increase the risk of missed captures.
 */
ISR(TIMER1_CAPT_vect) {
    /*
     * Determine which edge triggered this capture.
     *
     * The ICES1 bit reflects the edge sense configuration *prior* to this
     * capture. It must be read before toggling so that the recorded edge
     * polarity corresponds to the event that just occurred.
     */
    const capture_edge_t edge =
        (TCCR1B & _BV(ICES1)) ? CAPTURE_EDGE_RISING : CAPTURE_EDGE_FALLING;

    /*
     * Read the captured timer value.
     *
     * The hardware latches TCNT1 into ICR1 at the moment of the qualifying
     * edge. Reading ICR1 here retrieves that latched value; it is not affected
     * by subsequent timer progression.
     */
    uint16_t ovf_hi = timer1_overflow_hi;
    const uint16_t icr_ticks = ICR1;
    const uint8_t tifr = TIFR1;

    /*
    * Boundary guard:
    * If overflow occurred (TOV1 set) but the overflow ISR hasn't yet incremented
    * timer1_overflow_hi, then for captures just after overflow ICR1 will be low.
    */
    if ((tifr & _BV(TOV1)) && (icr_ticks < 0x8000u)) {
        ovf_hi++;
    }

    const uint32_t ticks = ((uint32_t)ovf_hi << 16) | icr_ticks;

    /*
     * Attempt to enqueue the event into the ring buffer.
     *
     * The buffer is considered full if advancing the head index would collide
     * with the tail. In that case, the event is not stored and is instead
     * counted as dropped to preserve transparency of data loss.
     */
    const uint8_t head = buffer_head;
    const uint8_t next = (head + 1) & CAPTURE_BUFFER_MASK;

    if (next != buffer_tail) {
        capture_buffer[head].ticks = ticks;
        capture_buffer[head].edge = edge;
        buffer_head = next;
    } else {
        /*
         * Buffer overflow: record that an event was lost.
         *
         * Dropped events are explicitly counted so that downstream analysis
         * can detect and account for overload conditions.
         */
        dropped_events++;
    }

    /*
     * Prepare for the next capture.
     *
     * - Clear the input capture and overflow flags to prevent spurious
     *   re-entry.
     * - Toggle the edge sense so that successive rising and falling edges
     *   are captured alternately.
     *
     * The order here ensures that the current event is fully acknowledged
     * before re-arming the capture logic.
     */
    TIFR1 = _BV(ICF1);
    TCCR1B ^= _BV(ICES1);
}

