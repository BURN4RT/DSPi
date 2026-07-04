/*
 * uart_control.c - UART control transport for DSPi
 *
 * See uart_control.h for the wire protocol.  Design rule: nothing here ever
 * blocks or busy-waits.  The exclusive UART IRQ only drains the RX FIFO into a
 * ring; every parse, dispatch and TX byte is produced from uart_ctrl_poll() in
 * main-loop context, so the real-time audio path is never delayed.
 */

#include "uart_control.h"
#include "vendor_commands.h"
#include "usb_audio.h"
#include "bulk_params.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

#define UART_SYNC            0xA5u
#define UART_TYPE_SET        0x01u
#define UART_TYPE_GET        0x02u
#define UART_TYPE_NOTIFY     0x40u  // reserved (device->host); not implemented
#define UART_RESP_SET        0x81u
#define UART_RESP_GET        0x82u

#define RX_RING_SIZE         256u                 // power of two
#define RX_RING_MASK         (RX_RING_SIZE - 1u)
#define PAYLOAD_MAX          64u                  // non-bulk SET payload cap
#define TX_COPY_MAX          80u                  // GET response copy buffer

#define FRAME_TIMEOUT_US     100000u              // inter-byte mid-frame timeout
#define DISPATCH_RETRY_US    50000u               // transient-status retry window

// ---------------------------------------------------------------------------
// Peripheral / live-config state
// ---------------------------------------------------------------------------

static uart_inst_t   *g_uart = NULL;
static uint           g_irq = 0;
static bool           g_irq_installed = false;
static bool           g_is_live = false;
static UartCtrlConfig g_live;

// ---------------------------------------------------------------------------
// RX ring (single-producer ISR, single-consumer poll)
// ---------------------------------------------------------------------------

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;   // written by ISR only
static volatile uint16_t rx_tail = 0;   // written by consumer only
static volatile uint32_t rx_overrun = 0;

// ---------------------------------------------------------------------------
// Parser FSM state
// ---------------------------------------------------------------------------

typedef enum {
    RX_WAIT_SYNC = 0,
    RX_TYPE,
    RX_HEADER,   // 7 header bytes
    RX_PAYLOAD,  // SET only
    RX_CRC,      // 2 CRC bytes
} RxState;

static RxState  rx_state = RX_WAIT_SYNC;
static bool     rx_is_get = false;
static uint8_t  hdr[7];
static uint8_t  hdr_pos = 0;
static uint8_t  bReq = 0;
static uint16_t wValue = 0, wIndex = 0, wLen = 0;
static uint16_t payload_pos = 0;
static bool     discard = false;         // consume+CRC without storing
static bool     to_bulk = false;         // stream payload into bulk_param_buf
static uint8_t  deferred_status = CTRL_STATUS_OK;
static uint16_t rx_crc = 0xFFFF;
static uint8_t  crc_pos = 0;
static uint8_t  crc_rx_lo = 0;
static bool     parser_holds_bulk = false;  // parser owns the bulk lock

static uint8_t  payload_buf[PAYLOAD_MAX];
static uint8_t  tx_copy[TX_COPY_MAX];
static uint32_t last_byte_time = 0;

// ---------------------------------------------------------------------------
// Pending dispatch (one request in flight)
// ---------------------------------------------------------------------------

static struct {
    bool     active;
    bool     is_get;
    uint8_t  bReq;
    uint16_t wValue, wIndex, wLen;
    uint32_t retry_start;
} pending;

// ---------------------------------------------------------------------------
// TX state machine (pumped only from poll)
// ---------------------------------------------------------------------------

typedef enum {
    TX_SYNC = 0, TX_TYPE, TX_STATUS, TX_LEN_L, TX_LEN_H,
    TX_PAYLOAD, TX_CRC_L, TX_CRC_H,
} TxState;

static struct {
    bool           active;
    bool           bulk;      // payload is bulk_param_buf, holds the bulk lock
    TxState        state;
    uint8_t        type;
    uint8_t        status;
    uint16_t       len;
    uint16_t       pos;
    uint16_t       crc;
    const uint8_t *payload;
} tx;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// CRC16-CCITT-FALSE, one byte at a time (bitwise; only ever called per-byte).
static uint16_t crc16_step(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    return crc;
}

// UARTx instance index for a GPIO from its pin-mux position: 0 = uart0, 1 = uart1.
static inline uint8_t uart_instance_index(uint8_t pin) {
    return (uint8_t)((((pin >> 2) + 1) >> 1) & 1);
}

static void release_parser_bulk(void) {
    if (parser_holds_bulk) {
        vendor_bulk_release(CTRL_SOURCE_UART);
        parser_holds_bulk = false;
    }
}

// Reset all parser/TX state; release any bulk lock we hold (parser or TX).
static void reset_proto_state(void) {
    release_parser_bulk();
    if (tx.bulk) {
        vendor_bulk_release(CTRL_SOURCE_UART);
        tx.bulk = false;
    }
    rx_state = RX_WAIT_SYNC;
    hdr_pos = 0;
    payload_pos = 0;
    discard = false;
    to_bulk = false;
    crc_pos = 0;
    pending.active = false;
    tx.active = false;
    rx_head = 0;
    rx_tail = 0;
}

// Abort a partially-received frame on inter-byte timeout; send nothing.
static void abort_frame(void) {
    release_parser_bulk();
    rx_state = RX_WAIT_SYNC;
    hdr_pos = 0;
    payload_pos = 0;
    discard = false;
    to_bulk = false;
    crc_pos = 0;
}

static uint8_t status_from_dispatch(CtrlDispatchResult r) {
    switch (r) {
        case CTRL_DISPATCH_OK:          return CTRL_STATUS_OK;
        case CTRL_DISPATCH_ERROR:       return CTRL_STATUS_ERROR;
        case CTRL_DISPATCH_BLOCKED:     return CTRL_STATUS_BLOCKED;
        case CTRL_DISPATCH_BULK_LOCKED: return CTRL_STATUS_BULK_LOCKED;
        case CTRL_DISPATCH_BUSY:        return CTRL_STATUS_BUSY;
        default:                        return CTRL_STATUS_ERROR;
    }
}

// Arm the TX state machine.  Payload is emitted only when status == OK.
static void start_tx(uint8_t type, uint8_t status,
                     const uint8_t *payload, uint16_t len, bool bulk) {
    tx.type    = type;
    tx.status  = status;
    tx.len     = (status == CTRL_STATUS_OK) ? len : 0;
    tx.payload = payload;
    tx.bulk    = bulk;
    tx.crc     = 0xFFFF;
    tx.state   = TX_SYNC;
    tx.pos     = 0;
    tx.active  = true;
}

static inline void start_tx_status(uint8_t type, uint8_t status) {
    start_tx(type, status, NULL, 0, false);
}

// ---------------------------------------------------------------------------
// RX IRQ: drain the FIFO into the ring (never blocks, never parses)
// ---------------------------------------------------------------------------

static void uart_ctrl_irq(void) {
    while (uart_is_readable(g_uart)) {
        uint8_t b = (uint8_t)uart_get_hw(g_uart)->dr;
        uint16_t next = (uint16_t)((rx_head + 1) & RX_RING_MASK);
        if (next == rx_tail) { rx_overrun++; continue; }  // full: drop
        rx_ring[rx_head] = b;
        rx_head = next;
    }
}

static int ring_pop(void) {
    if (rx_tail == rx_head) return -1;
    uint8_t b = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) & RX_RING_MASK);
    return (int)b;
}

// ---------------------------------------------------------------------------
// Frame parsing
// ---------------------------------------------------------------------------

// SET header parsed: decide the payload destination (bulk / normal / discard).
static void header_complete(void) {
    bReq   = hdr[0];
    wValue = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
    wIndex = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    wLen   = (uint16_t)hdr[5] | ((uint16_t)hdr[6] << 8);
    discard = false;
    to_bulk = false;
    payload_pos = 0;
    deferred_status = CTRL_STATUS_OK;

    if (rx_is_get) { crc_pos = 0; rx_state = RX_CRC; return; }  // GET: no payload
    if (wLen == 0) { crc_pos = 0; rx_state = RX_CRC; return; }

    bool bulk = (bReq == REQ_SET_ALL_PARAMS &&
                 wLen >= WIRE_BULK_PARAMS_MIN_SIZE &&
                 wLen <= sizeof(WireBulkParams));
    if (bulk) {
        if (vendor_bulk_try_acquire(CTRL_SOURCE_UART)) {
            parser_holds_bulk = true;
            to_bulk = true;                    // stream straight into bulk_param_buf
        } else {
            discard = true;                    // buffer busy; answer after the frame
            deferred_status = CTRL_STATUS_BULK_LOCKED;
        }
    } else if (wLen > PAYLOAD_MAX) {
        discard = true;
        deferred_status = CTRL_STATUS_OVERSIZE;
    }
    rx_state = RX_PAYLOAD;
}

// A full frame arrived; `good` is the CRC verdict.  Either respond directly
// (errors / discarded frames) or hand the request to the dispatch retry loop.
static void frame_complete(bool good) {
    uint8_t resp_type = rx_is_get ? UART_RESP_GET : UART_RESP_SET;

    if (!good) {
        release_parser_bulk();
        discard = false;
        start_tx_status(resp_type, CTRL_STATUS_CRC_ERROR);
        return;
    }
    if (discard) {
        // Nothing was acquired (bulk-locked never acquired; oversize is non-bulk).
        start_tx_status(resp_type, deferred_status);
        discard = false;
        return;
    }
    // Good CRC on a real frame: dispatch it (retried across polls if transient).
    pending.is_get      = rx_is_get;
    pending.bReq        = bReq;
    pending.wValue      = wValue;
    pending.wIndex      = wIndex;
    pending.wLen        = wLen;
    pending.retry_start = time_us_32();
    pending.active      = true;
}

static void feed_byte(uint8_t b) {
    last_byte_time = time_us_32();
    switch (rx_state) {
        case RX_WAIT_SYNC:
            if (b == UART_SYNC) rx_state = RX_TYPE;
            break;

        case RX_TYPE:
            if (b == UART_TYPE_SET || b == UART_TYPE_GET) {
                rx_is_get = (b == UART_TYPE_GET);
                rx_crc = crc16_step(0xFFFF, b);
                hdr_pos = 0;
                rx_state = RX_HEADER;
            } else {
                rx_state = RX_WAIT_SYNC;  // noise (includes reserved 0x40): resync
            }
            break;

        case RX_HEADER:
            hdr[hdr_pos++] = b;
            rx_crc = crc16_step(rx_crc, b);
            if (hdr_pos == 7) header_complete();
            break;

        case RX_PAYLOAD:
            rx_crc = crc16_step(rx_crc, b);
            if (!discard) {
                if (to_bulk) bulk_param_buf[payload_pos] = b;
                else         payload_buf[payload_pos] = b;
            }
            payload_pos++;
            if (payload_pos == wLen) { crc_pos = 0; rx_state = RX_CRC; }
            break;

        case RX_CRC:
            if (crc_pos == 0) {
                crc_rx_lo = b;
                crc_pos = 1;
            } else {
                uint16_t rec = (uint16_t)crc_rx_lo | ((uint16_t)b << 8);
                rx_state = RX_WAIT_SYNC;
                frame_complete(rec == rx_crc);
            }
            break;
    }
}

// Consume ring bytes until a frame completes (sets pending/tx) or the ring
// empties.  Only called when no response is transmitting and none is pending.
static void parse_ring(void) {
    int c;
    while (!tx.active && !pending.active && (c = ring_pop()) >= 0)
        feed_byte((uint8_t)c);
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

static void try_dispatch(void) {
    CtrlDispatchResult res;
    const uint8_t *rd = NULL;
    uint16_t rl = 0;

    if (pending.is_get) {
        res = vendor_dispatch_get(CTRL_SOURCE_UART, pending.bReq, pending.wValue,
                                  pending.wIndex, pending.wLen, &rd, &rl);
    } else {
        // For bulk SET the payload already sits in bulk_param_buf under our lock;
        // vendor_dispatch_set ignores the payload arg in that case.
        res = vendor_dispatch_set(CTRL_SOURCE_UART, pending.bReq, pending.wValue,
                                  pending.wIndex, payload_buf, pending.wLen);
    }

    // Transient results retry internally for a short window before giving up.
    bool retryable = (res == CTRL_DISPATCH_BUSY) ||
                     (pending.is_get && res == CTRL_DISPATCH_BULK_LOCKED);
    if (retryable &&
        (uint32_t)(time_us_32() - pending.retry_start) < DISPATCH_RETRY_US)
        return;  // keep pending; retry on the next poll

    uint8_t resp_type = pending.is_get ? UART_RESP_GET : UART_RESP_SET;

    // Bulk SET lock: vendor_dispatch_set releases it on OK; we release on any
    // non-OK result (including a timed-out BUSY that never touched the lock).
    if (!pending.is_get) {
        if (parser_holds_bulk && res != CTRL_DISPATCH_OK)
            vendor_bulk_release(CTRL_SOURCE_UART);
        parser_holds_bulk = false;
    }

    pending.active = false;

    if (pending.is_get && res == CTRL_DISPATCH_OK) {
        if (pending.bReq == REQ_GET_ALL_PARAMS) {
            // Zero-copy stream from bulk_param_buf; the dispatcher handed us the
            // bulk lock, released when the last CRC byte goes out.
            start_tx(resp_type, CTRL_STATUS_OK, rd, rl, true);
        } else {
            // The dispatcher's static buffer is only valid until the next
            // dispatch from any transport, so snapshot it now.
            uint16_t n = (rl > TX_COPY_MAX) ? TX_COPY_MAX : rl;
            memcpy(tx_copy, rd, n);
            start_tx(resp_type, CTRL_STATUS_OK, tx_copy, n, false);
        }
    } else {
        start_tx_status(resp_type, status_from_dispatch(res));
    }
}

// ---------------------------------------------------------------------------
// TX pump (main-loop only; never waits on the FIFO)
// ---------------------------------------------------------------------------

static void pump_tx(void) {
    while (tx.active && uart_is_writable(g_uart)) {
        uint8_t out;
        switch (tx.state) {
            case TX_SYNC:
                out = UART_SYNC;  // sync byte is outside the CRC
                tx.state = TX_TYPE;
                break;
            case TX_TYPE:
                out = tx.type;    tx.crc = crc16_step(tx.crc, out); tx.state = TX_STATUS; break;
            case TX_STATUS:
                out = tx.status;  tx.crc = crc16_step(tx.crc, out); tx.state = TX_LEN_L; break;
            case TX_LEN_L:
                out = (uint8_t)(tx.len & 0xFF); tx.crc = crc16_step(tx.crc, out); tx.state = TX_LEN_H; break;
            case TX_LEN_H:
                out = (uint8_t)(tx.len >> 8);   tx.crc = crc16_step(tx.crc, out);
                tx.pos = 0;
                tx.state = tx.len ? TX_PAYLOAD : TX_CRC_L;
                break;
            case TX_PAYLOAD:
                out = tx.payload[tx.pos];  tx.crc = crc16_step(tx.crc, out); tx.pos++;
                if (tx.pos == tx.len) tx.state = TX_CRC_L;
                break;
            case TX_CRC_L:
                out = (uint8_t)(tx.crc & 0xFF); tx.state = TX_CRC_H; break;
            case TX_CRC_H:
                out = (uint8_t)(tx.crc >> 8);
                uart_get_hw(g_uart)->dr = out;   // last byte of the frame
                tx.active = false;
                if (tx.bulk) { vendor_bulk_release(CTRL_SOURCE_UART); tx.bulk = false; }
                continue;
            default:
                tx.active = false;
                continue;
        }
        uart_get_hw(g_uart)->dr = out;
    }
}

// ---------------------------------------------------------------------------
// Bring-up / teardown
// ---------------------------------------------------------------------------

static void up(void) {
    uint8_t tx_pin = g_live.tx_pin;
    uint8_t rx_pin = g_live.rx_pin;
    uint8_t idx = uart_instance_index(tx_pin);
    g_uart = idx ? uart1 : uart0;
    g_irq  = idx ? UART1_IRQ : UART0_IRQ;

    uart_init(g_uart, g_live.baud);
    uart_set_format(g_uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(g_uart, false, false);
    uart_set_fifo_enabled(g_uart, true);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin);   // keep RX idle-high when no external MCU is attached

    while (uart_is_readable(g_uart)) (void)uart_get_hw(g_uart)->dr;  // drain stale RX

    reset_proto_state();

    irq_set_exclusive_handler(g_irq, uart_ctrl_irq);
    irq_set_priority(g_irq, 0xC0);
    irq_set_enabled(g_irq, true);
    g_irq_installed = true;
    uart_set_irq_enables(g_uart, true, false);  // RX + RX-timeout, no TX IRQ

    g_is_live = true;
}

static void down(void) {
    if (g_irq_installed) {
        uart_set_irq_enables(g_uart, false, false);
        irq_set_enabled(g_irq, false);
        irq_remove_handler(g_irq, uart_ctrl_irq);
        g_irq_installed = false;
    }
    if (g_uart) uart_deinit(g_uart);

    gpio_set_function(g_live.tx_pin, GPIO_FUNC_NULL);
    gpio_disable_pulls(g_live.tx_pin);
    gpio_set_function(g_live.rx_pin, GPIO_FUNC_NULL);
    gpio_disable_pulls(g_live.rx_pin);

    reset_proto_state();
    g_is_live = false;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

uint8_t uart_ctrl_validate(const UartCtrlConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->enabled > 1) return PIN_CONFIG_INVALID_PARAM;
    if (cfg->baud < UART_CTRL_BAUD_MIN || cfg->baud > UART_CTRL_BAUD_MAX)
        return PIN_CONFIG_INVALID_PARAM;
    if (!is_valid_gpio_pin(cfg->tx_pin) || !is_valid_gpio_pin(cfg->rx_pin))
        return PIN_CONFIG_INVALID_PIN;

    if (!cfg->enabled) return PIN_CONFIG_SUCCESS;

    // Enabled: pins must land on the correct UART mux and the same instance.
    if ((cfg->tx_pin & 3u) != 0u) return PIN_CONFIG_INVALID_PIN;   // TX at pin%4==0
    if ((cfg->rx_pin & 3u) != 1u) return PIN_CONFIG_INVALID_PIN;   // RX at pin%4==1
    if (uart_instance_index(cfg->tx_pin) != uart_instance_index(cfg->rx_pin))
        return PIN_CONFIG_INVALID_PIN;
    if (is_pin_in_use(cfg->tx_pin, 0xFF) || is_pin_in_use(cfg->rx_pin, 0xFF))
        return PIN_CONFIG_PIN_IN_USE;
    return PIN_CONFIG_SUCCESS;
}

void uart_ctrl_init(const UartCtrlConfig *cfg) {
    g_irq_installed = false;
    g_is_live = false;
    g_uart = NULL;
    tx.bulk = false;
    tx.active = false;
    parser_holds_bulk = false;
    reset_proto_state();

    if (!cfg) return;
    g_live = *cfg;

    if (uart_ctrl_validate(cfg) == PIN_CONFIG_SUCCESS && cfg->enabled)
        up();
}

uint8_t uart_ctrl_apply(const UartCtrlConfig *cfg) {
    if (!cfg) return PIN_CONFIG_INVALID_PARAM;

    UartCtrlConfig prev = g_live;
    bool prev_up = g_is_live;

    if (g_is_live) down();  // free our own pins before validating the new config

    uint8_t st = uart_ctrl_validate(cfg);
    if (st == PIN_CONFIG_SUCCESS) {
        g_live = *cfg;
        if (cfg->enabled) up();
        return st;
    }

    // Validation failed: best-effort restore the previous live config.
    if (prev_up) {
        g_live = prev;
        up();
    }
    return st;
}

void uart_ctrl_poll(void) {
    if (!g_is_live) return;

    // Keep the bulk lock fresh whenever we hold it (receiving, pending, or TX).
    if (parser_holds_bulk || tx.bulk)
        vendor_bulk_touch(CTRL_SOURCE_UART);

    // Inter-byte timeout: drop a stalled partial frame, send nothing.
    if (rx_state != RX_WAIT_SYNC &&
        (uint32_t)(time_us_32() - last_byte_time) > FRAME_TIMEOUT_US)
        abort_frame();

    if (tx.active) pump_tx();
    if (pending.active) try_dispatch();

    // Parse new bytes only when the transport is otherwise idle.
    if (!tx.active && !pending.active) parse_ring();
}

bool uart_ctrl_owns_pin(uint8_t pin) {
    return g_is_live && (pin == g_live.tx_pin || pin == g_live.rx_pin);
}

bool uart_ctrl_is_live(void) {
    return g_is_live;
}

void uart_ctrl_get_live_config(UartCtrlConfig *out) {
    if (out) *out = g_live;
}
