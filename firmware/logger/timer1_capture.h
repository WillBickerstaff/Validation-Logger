#ifndef TIMER1_CAPTURE_H
#define TIMER1_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Capture edge type recorded for each event.
typedef enum {
    CAPTURE_EDGE_RISING = 1u,
    CAPTURE_EDGE_FALLING = 0u,
} capture_edge_t;

// Timer1 runs at F_CPU with no prescaler (tick period = 1 / F_CPU seconds).
// The hardware Timer1 counter is 16-bit and wraps every 65536 ticks
// (≈ 8.192 ms at 8 MHz).
//
// Capture timestamps are extended in software using a Timer1 overflow
// counter, providing a monotonically increasing 32-bit tick value.
typedef struct {
    uint32_t ticks;          // Absolute Timer1 count captured in ICR1.
    capture_edge_t edge;     // Edge polarity observed.
} capture_event_t;

#ifndef CAPTURE_BUFFER_SIZE
#define CAPTURE_BUFFER_SIZE 64
#endif

// Configure Timer1 for input capture on ICP1 (PB0 on ATmega328P).
// Timer1 runs at F_CPU with no prescaler; ticks are raw timer counts.
void timer1_capture_init(void);

// Returns true when at least one event is queued.
bool timer1_capture_available(void);

// Pop the oldest event from the ring buffer. Returns false if empty.
bool timer1_capture_pop(capture_event_t *out_event);

// Number of events dropped due to ring-buffer overflow (wraps at 65535).
// Returned value is a coherent snapshot (read atomically).
uint16_t timer1_capture_dropped(void);

#ifdef __cplusplus
}
#endif

#endif  // TIMER1_CAPTURE_H
