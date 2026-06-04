/* This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (c) 2015 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* Define this to run this code on a PC instead of the NIOS II */
//#define BLADERF_NULL_HARDWARE

/* Will send debug alt_printf info to JTAG console while running on NIOS.
 * This can slow performance and cause timing issues... be careful. */
//#define BLADERF_NIOS_DEBUG

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include "devices.h"
#include "pkt_handler.h"
#include "pkt_8x8.h"
#include "pkt_8x16.h"
#include "pkt_8x32.h"
#include "pkt_8x64.h"
#include "pkt_16x64.h"
#include "pkt_32x32.h"
#include "pkt_retune2.h"
#include "pkt_legacy.h"
#include "debug.h"

/* HPSDR command-mailbox dispatcher + autonomous bring-up.  Fenced on
 * HPSDR_CMD_OP_BASE, which the BSP only defines for the hpsdr revision
 * (the hpsdr_cmd_* PIOs exist only in that revision's Qsys), so this
 * entire block compiles out for every other micro revision that shares
 * this source file. */
#if defined(BLADERF_NIOS_LIBAD936X) && defined(HPSDR_CMD_OP_BASE)
#  include "altera_avalon_pio_regs.h"
#  include "devices_rfic.h"
/* hpsdr_cmd_op PIO bit-layout (matches hpsdr_cmd_mux.vhd):
 *   [7:0]   opcode  (BLADERF_RFIC_COMMAND_* from bladerf2_common.h)
 *   [10:8]  channel (0=RX0, 1=RX1, 2=TX0, 3=TX1, 7=SYSTEM)
 *   [11]    rw      (0=write, 1=read)
 *   [15:12] seq     (strobe; fabric increments on every commit)
 *
 * hpsdr_cmd_status PIO bit-layout (driven by this code):
 *   [3:0]   done_seq -- the seq we just serviced
 *   [4]     err      -- the rfic_command_* call returned false
 *   [7:5]   reserved
 */
#  define HPSDR_CMD_OP_OPCODE(x)  ((uint8_t)((x) & 0xFF))
#  define HPSDR_CMD_OP_CHANNEL(x) ((uint8_t)(((x) >> 8) & 0x7))
#  define HPSDR_CMD_OP_RW(x)      ((((x) >> 11) & 0x1) != 0)
#  define HPSDR_CMD_OP_SEQ(x)     ((uint8_t)(((x) >> 12) & 0xF))
#  define HPSDR_CMD_STATUS_ERR    (1u << 4)

/* Map the 3-bit hpsdr_cmd_op channel field to a bladerf_channel handle. */
static inline bladerf_channel hpsdr_decode_channel(uint8_t ch)
{
    switch (ch) {
        case 0:  return BLADERF_CHANNEL_RX(0);
        case 1:  return BLADERF_CHANNEL_RX(1);
        case 2:  return BLADERF_CHANNEL_TX(0);
        case 3:  return BLADERF_CHANNEL_TX(1);
        case 7:  return RFIC_SYSTEM_CHANNEL;
        default: return RFIC_SYSTEM_CHANNEL;
    }
}

/* Virtual-transverter LO offset applied to FREQUENCY writes (see
 * HPSDR_LO_OFFSET_HZ below).  Discovery advertises FREQ_PHASE=0x01 (Orion
 * MkII), so cmd_data_in is a 32-bit NCO phase word, NOT Hz.  Convert with
 * freq_Hz = phase * 122.88e6 / 2^32 (122.88 MHz = Orion MkII DSP clock),
 * then add the transverter LO.  Clamp negative results to 0; the RFIC
 * layer rejects sub-70 MHz anyway via _modify_spdt_bits_by_freq. */
static inline uint64_t hpsdr_phase_to_tune_hz(uint32_t phase, int32_t lo_offset)
{
    uint32_t if_hz       = (uint32_t)(((uint64_t)phase * 122880000ULL) >> 32);
    int64_t  signed_tune = (int64_t)if_hz + (int64_t)lo_offset;
    return (signed_tune < 0) ? 0u : (uint64_t)signed_tune;
}
/* Bring RX0 online with no host / bladeRF-cli session.  Set to 0 to revert to
 * purely host-driven bring-up for bench debugging.  The bring-up runs from the
 * main loop (NOT before it) and is triggered by an HPSDR client discovery (the
 * hpsdr_status host_valid bit), which only happens in standalone Ethernet
 * operation -- so it never blocks the host's post-load FPGA-version handshake
 * and never fights libbladeRF's own RFIC init during a USB FPGA load.  There are
 * seconds of slack between Thetis's discovery and its run=1. */
#  ifndef HPSDR_AUTONOMOUS_RX_INIT
#    define HPSDR_AUTONOMOUS_RX_INIT 1
#  endif
/* hpsdr_status PIO bits (engagement state from the FPGA fabric). */
#  define HPSDR_STATUS_HOST_VALID (1u << 0)  /* a Thetis discovery committed */
#  define HPSDR_STATUS_HOST_RUN   (1u << 1)  /* HP Command run=1 (reserved) */
/* Native AD9361 RX rate: matches U_hpsdr_ddc DECIMATION=256 -> 48 kHz, and is
 * above the AD9361 decimation-FIR floor so a single SAMPLERATE command works. */
#  define HPSDR_RX_SAMPLERATE 12288000u
/* Virtual-transverter LO offset (Hz, signed).  Added to the IF frequency Thetis
 * sends in the HP Command before commanding the AD9361, so the radio behaves as
 * if a transverter sits between it and the antenna.  Lets one work above the
 * 61.44 MHz Thetis NCO half-Nyquist cap, and around the 70 MHz bladeRF RX
 * floor.  Thetis's own Setup -> Transverter dialog must be set to the same LO
 * so its dial display reads the true RF frequency.  Set to 0 to disable.
 *
 * Examples (Thetis dial range 0..61.43 MHz):
 *   HPSDR_LO_OFFSET_HZ =  100000000 -> covers 100..161.44 MHz (2 m band)
 *   HPSDR_LO_OFFSET_HZ =  400000000 -> covers 400..461.44 MHz (70 cm band)
 *   HPSDR_LO_OFFSET_HZ = 1090000000 -> covers 1090..1151.44 MHz (ADS-B / 23 cm)
 *
 * The post-offset frequency still has to land in the AD9361's 70 MHz..6 GHz
 * range or _modify_spdt_bits_by_freq() returns BLADERF_ERR_INVAL. */
#  ifndef HPSDR_LO_OFFSET_HZ
#    define HPSDR_LO_OFFSET_HZ 400000000
#  endif
#endif

#define BLADERF_DEVICE_NAME "Nuand bladeRF 2.0 Micro"

#ifdef BLADERF_NIOS_PC_SIMULATION
    extern bool run_nios;
#   define HAVE_REQUEST() ({ \
        have_request = !have_request; \
        if (have_request) { \
            command_uart_read_request( (uint8_t*) pkt.req); \
        } \
        have_request; \
    })

    /* We need to reset the response buffer to known values so we can
     * compare against expected test case responses */
#   define RESET_RESPONSE_BUF

#else
#   define run_nios true
#   define HAVE_REQUEST() (pkt.ready == true)
#endif

#ifdef RESET_RESPONSE_BUF
#   undef RESET_RESPONSE_BUF
#   define RESET_RESPONSE_BUF() do { \
        memset(pkt.resp, 0xff, NIOS_PKT_LEN); \
    } while (0)

#else
#   define RESET_RESPONSE_BUF() do {} while (0)
#endif

/* When adding packet handlers here, you must also ensure that you update
 * the bladeRF/hdl/fpga/ip/nuand/command_uart/vhdl/command_uart.vhd
 * to include the magic header byte value in the `magics` array.
 */
static const struct pkt_handler pkt_handlers[] = {
    PKT_RETUNE2,
    PKT_8x8,
    PKT_8x16,
    PKT_8x32,
    PKT_8x64,
    PKT_16x64,
    PKT_32x32,
    PKT_LEGACY,
};

/* A structure that represents a point on a line. Used for calibrating
 * the VCTCXO */
typedef struct point {
    int32_t  x; // Error counts
    uint16_t y; // DAC count
} point_t;

typedef struct line {
    point_t  point[2];
    int32_t  slope;
    uint16_t y_intercept; // in DAC counts
} line_t;

/* State machine for VCTCXO tuning */
typedef enum state {
    COARSE_TUNE_MIN,
    COARSE_TUNE_MAX,
    COARSE_TUNE_DONE,
    FINE_TUNE,
    DO_NOTHING
} state_t;

int main(void)
{
    DBG(BLADERF_DEVICE_NAME " FPGA v%x.%x.%x\n",
        FPGA_VERSION_MAJOR, FPGA_VERSION_MINOR, FPGA_VERSION_PATCH);
    DBG("Built " __DATE__ " " __TIME__ " with love <3\n");
#ifdef BLADERF_NIOS_LIBAD936X
    DBG("libad936x found: This FPGA image has magic transgirl powers\n");
#endif  // BLADERF_NIOS_LIBAD936X

    uint8_t i;

    /* Pointer to currently active packet handler */
    const struct pkt_handler *handler;

    struct pkt_buf pkt;
    struct vctcxo_tamer_pkt_buf vctcxo_tamer_pkt;

    /* Marked volatile to ensure we actually read the byte populated by
     * the UART ISR */
    const volatile uint8_t *magic = &pkt.req[PKT_MAGIC_IDX];

    volatile bool have_request = false;

#if defined(BLADERF_NIOS_LIBAD936X) && defined(HPSDR_CMD_OP_BASE)
    /* HPSDR command-mailbox dispatcher state.  prev_op = previous raw PIO
     * sample (for the 2-poll tearing filter); last_seq = the seq value of
     * the most recently dispatched request (initialised to 0 -- matches the
     * fabric's reset state, so the first real seq=1 from the mux trips the
     * delta and dispatches). */
    uint16_t hpsdr_cmd_prev_op = 0;
    uint8_t  hpsdr_cmd_last_seq = 0;
#  if HPSDR_AUTONOMOUS_RX_INIT
    bool     hpsdr_brought_up   = false;  /* autonomous bring-up done (one-shot) */
#  endif
#endif

#ifdef BLADERF_NIOS_DEBUG
    // Twiddler: gratuitous screen placebo
    size_t const TWIDDLE_DELAY_CONSTANT = 100000;
    size_t twiddle_count = 0;
    enum {
        TWIDDLE_STATE_PIPE = '|',
        TWIDDLE_STATE_FWD_SLASH = '/',
        TWIDDLE_STATE_HYPHEN = '-',
        TWIDDLE_STATE_BACKSLASH = '\\',
    } twiddle_state = TWIDDLE_STATE_PIPE;
#endif

    // Trim DAC constants
    const uint16_t trimdac_min       = 0x28F5;
    const uint16_t trimdac_max       = 0xF5C3;

    // Trim DAC calibration line
    line_t trimdac_cal_line;

    // VCTCXO Tune State machine
    state_t tune_state = COARSE_TUNE_MIN;

    // Set the known/default values of the trim DAC cal line
    trimdac_cal_line.point[0].x  = 0;
    trimdac_cal_line.point[0].y  = trimdac_min;
    trimdac_cal_line.point[1].x  = 0;
    trimdac_cal_line.point[1].y  = trimdac_max;
    trimdac_cal_line.slope       = 0;
    trimdac_cal_line.y_intercept = 0;

    /* Sanity check */
    ASSERT(PKT_MAGIC_IDX == 0);

    memset(&pkt, 0, sizeof(pkt));
    pkt.ready = false;
    bladerf_nios_init(&pkt, &vctcxo_tamer_pkt);

    /* Initialize packet handlers */
    for (i = 0; i < ARRAY_SIZE(pkt_handlers); i++) {
        if (pkt_handlers[i].init != NULL) {
            pkt_handlers[i].init();
        }
    }

    /* ====================
     * AD9361 SPI TESTS
     * ==================== */
    #ifdef BLADERF_NIOS_AD9361_SPI_TESTS
        uint16_t adi_spi_addr;
        uint64_t adi_spi_data;

        while ( 1 ) {
            //             W/Rb        | NB2:0       | A[9:0]
            adi_spi_addr = (0x0 << 15) | (0x0 << 12) | (0x000 << 0);
            adi_spi_data = UINT64_C(0x0);

            // Read AD9361 registers 0x045-0x04c
            for( i=0x45; i < 0x4d; i++ ) {
                adi_spi_addr = (adi_spi_addr & 0xfc00) | i;
                adi_spi_data = adi_spi_read(adi_spi_addr);
            }

            // Read 0x028
            adi_spi_addr = (adi_spi_addr & 0xfc00) | 0x28;
            adi_spi_data = adi_spi_read(adi_spi_addr);

            // Write 0x5a to 0x028
            adi_spi_addr = (adi_spi_addr & 0xfc00) | 0x28;
            adi_spi_data = (UINT64_C(0x5a) << (64-8));
            adi_spi_write((adi_spi_addr | 0x8000), adi_spi_data);

            // Read 0x028
            adi_spi_addr = (adi_spi_addr & 0xfc00) | 0x28;
            adi_spi_data = adi_spi_read(adi_spi_addr);

            // Write 0x5a to 0x028
            adi_spi_addr = (adi_spi_addr & 0xfc00) | 0x28;
            adi_spi_data = (UINT64_C(0x0) << (64-8));
            adi_spi_write((adi_spi_addr | 0x8000), adi_spi_data);

            usleep(100);
        }
    #endif


    /* ====================
     * AD5621 SPI TESTS
     * ==================== */
    #ifdef BLADERF_NIOS_AD5621_SPI_TESTS
        uint16_t dac_val;

        // Disable the ADF400x
        control_reg_write( 0x0 );
        usleep(2000000);
        while( 1 ) {
            for( i = 0; i < 6; i++ ) {
                // Calculate DAC value
                dac_val = ((((uint16_t)i) * 0x333) << 2) & 0x3fff;
                // Write DAC value
                ad56x1_vctcxo_trim_dac_write(dac_val);
                usleep(2000000);
                // Tristate the DAC, keeping same DAC value in register
                ad56x1_vctcxo_trim_dac_write(dac_val | 0xc000 );
                usleep(2000000);
            }
        }
    #endif


    /* ====================
     * ADF4001 SPI TESTS
     * ==================== */
    #ifdef BLADERF_NIOS_ADF4001_SPI_TESTS
        // Tristate the DAC
        ad56x1_vctcxo_trim_dac_write( 0xc000 );
        while( 1 ) {

            // Enable the ADF400x
            control_reg_write( 0x1 << 11 );
            usleep(2000000);

            // 0x000003 -- MUXOUT = 'Z'; CP = Normal
            adf400x_spi_write(0x3);
            usleep(2000000);

            // 0x000137 -- MUXOUT = AVDD (3.3 V); CP = 'Z'
            adf400x_spi_write(0x137);
            usleep(2000000);

            // 0x000177 -- MUXOUT = DGND; CP = 'Z'
            adf400x_spi_write(0x177);
            usleep(2000000);

            // 0x000167 -- MUXOUT = SDO; CP = 'Z'
            //adf400x_spi_write(0x167);
            //usleep(2000000);

            for( i = 0; i < 4; i++ ) {
                // Read all the registers
                adf400x_spi_read((uint32_t)i);
                usleep(100);
            }

            // Disable the ADF400x
            control_reg_write( 0x0 );
            usleep(2000000);
        }
    #endif

    DBG("=== System Ready ===\n");

    while (run_nios) {
        have_request = HAVE_REQUEST();

#ifdef BLADERF_NIOS_DEBUG
        if (have_request) {
            twiddle_count = 0;
        } else if (TWIDDLE_DELAY_CONSTANT == ++twiddle_count) {
            DBG("%c\b", twiddle_state);

            switch (twiddle_state) {
                case TWIDDLE_STATE_PIPE:
                    twiddle_state = TWIDDLE_STATE_FWD_SLASH;
                    break;
                case TWIDDLE_STATE_FWD_SLASH:
                    twiddle_state = TWIDDLE_STATE_HYPHEN;
                    break;
                case TWIDDLE_STATE_HYPHEN:
                    twiddle_state = TWIDDLE_STATE_BACKSLASH;
                    break;
                case TWIDDLE_STATE_BACKSLASH:
                    twiddle_state = TWIDDLE_STATE_PIPE;
                    break;
            }

            twiddle_count = 0;
        }
#endif

        /* We have a command in the UART */
        if (have_request) {
            pkt.ready = false;
            handler = NULL;

            /* Determine which packet handler should receive this message */
            for (i = 0; i < ARRAY_SIZE(pkt_handlers); i++) {
                if (pkt_handlers[i].magic == *magic) {
                    handler = &pkt_handlers[i];
                }
            }

            if (handler == NULL) {
                /* We somehow got out of sync. Throw away request data until
                 * we hit a magic value */
                DBG("Got invalid magic value: 0x%x\n", pkt.req[PKT_MAGIC_IDX]);
                continue;
            }

            print_bytes("Request data:", pkt.req, NIOS_PKT_LEN);

            /* If building with RESET_RESPONSE_BUF defined, reset response buffer
             * contents to ensure unused values are known values. */
            RESET_RESPONSE_BUF();

            /* Process data and execute requested actions */
            handler->exec(&pkt);

            /* Write response to host */
            command_uart_write_response(pkt.resp);
        } else {

            /* Temporarily putting the VCTCXO Calibration stuff here. */
            if( vctcxo_tamer_pkt.ready ) {

                vctcxo_tamer_pkt.ready = false;

                switch(tune_state) {

                case COARSE_TUNE_MIN:

                    /* Tune to the minimum DAC value */
                    vctcxo_trim_dac_write( 0x08, trimdac_min );

                    /* State to enter upon the next interrupt */
                    tune_state = COARSE_TUNE_MAX;

                    break;

                case COARSE_TUNE_MAX:

                    /* We have the error from the minimum DAC setting, store it
                     * as the 'x' coordinate for the first point */
                    trimdac_cal_line.point[0].x = vctcxo_tamer_pkt.pps_1s_error;

                    /* Tune to the maximum DAC value */
                    vctcxo_trim_dac_write( 0x08, trimdac_max );

                    /* State to enter upon the next interrupt */
                    tune_state = COARSE_TUNE_DONE;

                    break;

                case COARSE_TUNE_DONE:

                    /* We have the error from the maximum DAC setting, store it
                     * as the 'x' coordinate for the second point */
                    trimdac_cal_line.point[1].x = vctcxo_tamer_pkt.pps_1s_error;

                    /* We now have two points, so we can calculate the equation
                     * for a line plotted with DAC counts on the Y axis and
                     * error on the X axis. We want a PPM of zero, which ideally
                     * corresponds to the y-intercept of the line. */
                    trimdac_cal_line.slope = ( (trimdac_cal_line.point[1].y - trimdac_cal_line.point[0].y) /
                                               (trimdac_cal_line.point[1].x - trimdac_cal_line.point[0].x) );
                    trimdac_cal_line.y_intercept = ( trimdac_cal_line.point[0].y -
                                                     (trimdac_cal_line.slope * trimdac_cal_line.point[0].x) );

                    /* Set the trim DAC count to the y-intercept */
                    vctcxo_trim_dac_write( 0x08, trimdac_cal_line.y_intercept );

                    /* State to enter upon the next interrupt */
                    tune_state = FINE_TUNE;

                    break;

                case FINE_TUNE:

                    /* We should be extremely close to a perfectly tuned
                     * VCTCXO, but some minor adjustments need to be made */

                    /* Check the magnitude of the errors starting with the
                     * one second count. If an error is greater than the maxium
                     * tolerated error, adjust the trim DAC by the error (Hz)
                     * multiplied by the slope (in counts/Hz) and scale the
                     * result by the precision interval (e.g. 1s, 10s, 100s). */
                    if( vctcxo_tamer_pkt.pps_1s_error_flag ) {
                        vctcxo_trim_dac_write( 0x08, (vctcxo_trim_dac_value -
                            ((vctcxo_tamer_pkt.pps_1s_error * trimdac_cal_line.slope)/1)) );
                    } else if( vctcxo_tamer_pkt.pps_10s_error_flag ) {
                        vctcxo_trim_dac_write( 0x08, (vctcxo_trim_dac_value -
                            ((vctcxo_tamer_pkt.pps_10s_error * trimdac_cal_line.slope)/10)) );
                    } else if( vctcxo_tamer_pkt.pps_100s_error_flag ) {
                        vctcxo_trim_dac_write( 0x08, (vctcxo_trim_dac_value -
                            ((vctcxo_tamer_pkt.pps_100s_error * trimdac_cal_line.slope)/100)) );
                    }

                    break;

                default:
                    break;

                } /* switch */

                /* Take PPS counters out of reset */
                vctcxo_tamer_reset_counters( false );

                /* Enable interrupts */
                vctcxo_tamer_enable_isr( true );

            } /* VCTCXO Tamer interrupt */

            for (i = 0; i < ARRAY_SIZE(pkt_handlers); i++) {
                if (pkt_handlers[i].do_work != NULL) {
                    pkt_handlers[i].do_work();
                }
            }

#if defined(BLADERF_NIOS_LIBAD936X) && defined(HPSDR_CMD_OP_BASE) && HPSDR_AUTONOMOUS_RX_INIT
            /* Autonomous RX0 bring-up, one-shot, triggered by an HPSDR client
             * discovery (hpsdr_status host_valid).  Runs here -- in the main
             * loop, not before it -- so the command UART stays responsive for a
             * USB host's post-load FPGA-version read.  Discovery only occurs in
             * standalone Ethernet operation, so a USB libbladeRF session never
             * trips this and we never fight its RFIC init.  No FREQUENCY: the
             * radio parks at the AD9361 init default until Thetis's first HP
             * Command, which the dispatcher below then applies. */
            if (!hpsdr_brought_up &&
                (IORD_ALTERA_AVALON_PIO_DATA(HPSDR_STATUS_BASE) &
                 HPSDR_STATUS_HOST_VALID)) {
                bool ok;
                DBG("HPSDR: discovery seen, RX0 bring-up start\n");

                ok = rfic_command_write_immed(BLADERF_RFIC_COMMAND_INIT,
                                              RFIC_SYSTEM_CHANNEL,
                                              BLADERF_RFIC_INIT_STATE_ON);
                DBG("HPSDR:  INIT       -> %s\n", ok ? "ok" : "FAIL");

                ok = rfic_command_write_immed(BLADERF_RFIC_COMMAND_SAMPLERATE,
                                              BLADERF_CHANNEL_RX(0),
                                              HPSDR_RX_SAMPLERATE);
                DBG("HPSDR:  SAMPLERATE -> %s\n", ok ? "ok" : "FAIL");

                ok = rfic_command_write_immed(BLADERF_RFIC_COMMAND_GAINMODE,
                                              BLADERF_CHANNEL_RX(0),
                                              BLADERF_GAIN_MGC);
                DBG("HPSDR:  GAINMODE   -> %s\n", ok ? "ok" : "FAIL");

                ok = rfic_command_write_immed(BLADERF_RFIC_COMMAND_GAIN,
                                              BLADERF_CHANNEL_RX(0), 40);
                DBG("HPSDR:  GAIN       -> %s\n", ok ? "ok" : "FAIL");

                ok = rfic_command_write_immed(BLADERF_RFIC_COMMAND_ENABLE,
                                              BLADERF_CHANNEL_RX(0), 1);
                DBG("HPSDR:  ENABLE     -> %s\n", ok ? "ok" : "FAIL");

                hpsdr_brought_up = true;
                DBG("HPSDR: bring-up done\n");
            }
#endif

#if defined(BLADERF_NIOS_LIBAD936X) && defined(HPSDR_CMD_OP_BASE)
            /* HPSDR command-mailbox dispatcher.  The fabric (hpsdr_cmd_mux)
             * encodes one libbladeRF RFIC request per write to hpsdr_cmd_op,
             * incrementing a 4-bit seq strobe so duplicate opcode/data still
             * fire.  Both cmd_op and cmd_data_in are crossed bit-by-bit out
             * of fx3_pclk_pll into sys_clock, so a multi-bit sample can tear
             * for a couple of cycles around a write.  Act only on a sample
             * stable across two consecutive reads AND with a seq value we
             * haven't serviced -- combined this absorbs both the per-bit
             * tearing and the duplicate-op case (e.g. user re-applies the
             * same gain).
             *
             * Writes: cmd_data_in is the raw payload.  FREQUENCY is special
             * (32b NCO phase word, see hpsdr_phase_to_tune_hz); everything
             * else passes straight through as a uint64_t.  Reads: stash the
             * 64b result into cmd_data_lo/_hi before raising done_seq, so
             * the fabric sees stable data when it samples on the seq match.
             */
            {
                uint16_t op_word = IORD_ALTERA_AVALON_PIO_DATA(HPSDR_CMD_OP_BASE);
                uint8_t  seq     = HPSDR_CMD_OP_SEQ(op_word);

                if (op_word == hpsdr_cmd_prev_op && seq != hpsdr_cmd_last_seq) {
                    uint8_t  op  = HPSDR_CMD_OP_OPCODE(op_word);
                    uint8_t  ch  = HPSDR_CMD_OP_CHANNEL(op_word);
                    bool     rw  = HPSDR_CMD_OP_RW(op_word);
                    uint32_t din = IORD_ALTERA_AVALON_PIO_DATA(HPSDR_CMD_DATA_IN_BASE);
                    bladerf_channel bch = hpsdr_decode_channel(ch);
                    bool     ok  = false;
                    uint8_t  status_byte;

                    if (!rw) {
                        uint64_t value = (op == BLADERF_RFIC_COMMAND_FREQUENCY)
                            ? hpsdr_phase_to_tune_hz(din,
                                                     (int32_t)HPSDR_LO_OFFSET_HZ)
                            : (uint64_t)din;
                        ok = rfic_command_write_immed(
                                 (bladerf_rfic_command)op, bch, value);
                        DBG("HPSDR: W op=%x ch=%x din=%x val=%x:%x %s\n",
                            op, ch, din,
                            (uint32_t)(value >> 32), (uint32_t)value,
                            ok ? "ok" : "FAIL");
                    } else {
                        uint64_t value = 0;
                        ok = rfic_command_read_immed(
                                 (bladerf_rfic_command)op, bch, &value);
                        /* Data first, status last -- the fabric waits for
                         * the seq match before sampling, so updates to
                         * data_lo/_hi land before done_seq propagates. */
                        IOWR_ALTERA_AVALON_PIO_DATA(HPSDR_CMD_DATA_LO_BASE,
                                                    (uint32_t)value);
                        IOWR_ALTERA_AVALON_PIO_DATA(HPSDR_CMD_DATA_HI_BASE,
                                                    (uint32_t)(value >> 32));
                        DBG("HPSDR: R op=%x ch=%x val=%x:%x %s\n",
                            op, ch,
                            (uint32_t)(value >> 32), (uint32_t)value,
                            ok ? "ok" : "FAIL");
                    }

                    status_byte = (uint8_t)(seq & 0xF);
                    if (!ok) {
                        status_byte |= HPSDR_CMD_STATUS_ERR;
                    }
                    IOWR_ALTERA_AVALON_PIO_DATA(HPSDR_CMD_STATUS_BASE,
                                                status_byte);
                    hpsdr_cmd_last_seq = seq;
                }

                hpsdr_cmd_prev_op = op_word;
            }
#endif
        }
    }

    return 0;
}
