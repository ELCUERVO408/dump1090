// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_airspy.c: Airspy dongle support
//
// Copyright (c) 2014-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
//
// This file is free software: you may copy, redistribute and/or modify it  
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your  
// option) any later version.  
//
// This file is distributed in the hope that it will be useful, but  
// WITHOUT ANY WARRANTY; without even the implied warranty of  
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License  
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and  
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"
#include "sdr_airspy.h"

#include <airspy.h>
#include <inttypes.h>

#define NTHREADS 4

static struct {
    struct airspy_device *dev;
    airspy_model model;
    double sample_rate;
    uint64_t serial_number;
    bool enable_lna_agc;
    bool enable_mixer_agc;
    bool enable_linearity;
    bool enable_sensitivity;
    bool enable_airspy_biast;
    bool enable_bit_packing;
    uint8_t linearity_gain;
    uint8_t sensitivity_gain;
    uint8_t mixer_gain;
    uint8_t lna_gain;
    uint8_t vga_gain;
    uint16_t *samples;
    unsigned num_samples;
    uint16_t *magbuf;
    iq_convert_fn converter;
    struct converter_state *converter_state;
} AIRSPY;

//
// =============================== AIRSPY handling ==========================
//

void airspyInitConfig()
{
    AIRSPY.dev = NULL;
    AIRSPY.model = BADMODEL;
    AIRSPY.sample_rate = 0;
    AIRSPY.serial_number = 0;
    AIRSPY.enable_lna_agc = false;
    AIRSPY.enable_mixer_agc = false;
    AIRSPY.enable_linearity = false;
    AIRSPY.enable_sensitivity = false;
    AIRSPY.enable_airspy_biast = false;
    AIRSPY.enable_bit_packing = false;
    AIRSPY.linearity_gain = 0;
    AIRSPY.sensitivity_gain = 0; 
    AIRSPY.mixer_gain = 8;
    AIRSPY.lna_gain = 13;
    AIRSPY.vga_gain = 5;
    AIRSPY.samples = NULL;
    AIRSPY.num_samples = 0;
    AIRSPY.magbuf = NULL;
    AIRSPY.converter = NULL;
    AIRSPY.converter_state = NULL;
}

static bool parse_uintt(char *s, uint64_t *const value, int utype)
{
    char *s_end;
    uint64_t u64_val;

    s_end = s;

    u64_val = strtoull(s, &s_end, 0);
    if( (s != s_end) && (*s_end == 0) ) {
        switch(utype) {
            case 8:
            {
                if (u64_val > UINT8_MAX)
                    return false;
                uint8_t *u8_ptr = (uint8_t *)value;
                *u8_ptr = (uint8_t)u64_val;
                break;
            }
            case 16:
            {
                if (u64_val > UINT16_MAX)
                    return false;
                uint16_t *u16_ptr = (uint16_t *)value;
                *u16_ptr = (uint16_t)u64_val;
                break;
            }
            case 32:
            {
                if (u64_val > UINT32_MAX)
                    return false;
                uint32_t *u32_ptr = (uint32_t *)value;
                *u32_ptr = (uint32_t)u64_val;
                break;
            }
            case 64:
                if (u64_val > UINT64_MAX)
                    return false;
                *value = u64_val;
                break;
            default:
                return false;
                break;
        }
            return true;
    } else {
        return false;
    }
}

void airspyShowHelp()
{
    printf("      airspy-specific options (use with --device-type airspy)\n");
    printf("\n");
    printf("--model [mini | r2]      Specify Airspy model to use (required)\n");
    printf("--serial-number <0x...>  Serial number (in hex) of Airspy to open (optional)\n");
    printf("--enable-lna-agc         Enable LNA AGC (default off)\n");
    printf("--enable-mixer-agc       Enable MIXER AGC (default off)\n");
    printf("--linearity-gain <n>     Set linearity simplified gain (0-21)\n");
    printf("--sensitivity-gain <n>   Set sensitivity simplified gain (0-21)\n");
    printf("--mixer-gain <n>         Set MIXER gain (0-15, default 8)\n");
    printf("--lna-gain <n>           Set LNA gain (0-14, default 13)\n");
    printf("--vga-gain <n>           Set VGA gain (0-15, default 5)\n");
    printf("--enable-airspy-biast    Set bias tee supply on (default off)\n");
    printf("--enable-bit-packing     Enable USB bit packing (default off)\n");
    printf("\n");
}

bool airspyHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool result = true;
    bool more = (j + 1 < argc);

    if (!strcmp(argv[j], "--serial-number") && more) {
        result = parse_uintt(argv[++j], &AIRSPY.serial_number, 64);
    } else if (!strcmp(argv[j], "--model") && more) {
        ++j;
        if (!strcasecmp(argv[j], "mini")) {
            AIRSPY.model = MINI;
        } else if (!strcasecmp(argv[j], "r2")) {
            AIRSPY.model = R2;
        } else {
            fprintf(stderr, "Invalid Airspy model '%s' (supported values: mini, r2)\n",
                    argv[j]);
            return false;
        }
    } else if (!strcmp(argv[j], "--enable-lna-agc")) {
        AIRSPY.enable_lna_agc = true;
    } else if (!strcmp(argv[j], "--enable-mixer-agc")) {
        AIRSPY.enable_mixer_agc = true;
    } else if (!strcmp(argv[j], "--enable-airspy-biast")) {
        AIRSPY.enable_airspy_biast = true;
    } else if (!strcmp(argv[j], "--enable-bit-packing")) {
        AIRSPY.enable_bit_packing = true;
    } else if (!strcmp(argv[j], "--linearity-gain") && more) {
        result = parse_uintt(argv[++j], (uint64_t *)&AIRSPY.linearity_gain, 8);
        AIRSPY.enable_linearity = true;
    } else if (!strcmp(argv[j], "--sensitivity-gain") && more) {
        result = parse_uintt(argv[++j], (uint64_t *)&AIRSPY.sensitivity_gain, 8);
        AIRSPY.enable_sensitivity = true;
    } else if (!strcmp(argv[j], "--mixer-gain") && more) {
        result = parse_uintt(argv[++j], (uint64_t *)&AIRSPY.mixer_gain, 8);
    } else if (!strcmp(argv[j], "--lna-gain") && more) {
        result = parse_uintt(argv[++j], (uint64_t *)&AIRSPY.lna_gain, 8);
    } else if (!strcmp(argv[j], "--vga-gain") && more) {
        result = parse_uintt(argv[++j], (uint64_t *)&AIRSPY.vga_gain, 8);
    } else {
        return false;
    }

    if (!result)
        return false;

    if (AIRSPY.enable_linearity && AIRSPY.enable_sensitivity) {
        fprintf(stderr,
            "Airspy config: --linearity-gain and --sensitivity-gain cannot be used together.\n");
        return false;
    }

    if ((AIRSPY.enable_linearity || AIRSPY.enable_sensitivity) && (AIRSPY.enable_lna_agc || AIRSPY.enable_mixer_agc)) {
        fprintf(stderr,
            "Airspy config: --linearity-gain/--sensitivity-gain cannot be used with --enable-lna-agc/--enable-mixer-agc.\n");
        return false;
    }

    *jptr = j;
    return true;
}

bool airspyOpen(void) {
    #define AIRSPY_STATUS(status, message) \
        if (status != 0) { \
        fprintf(stderr, "%s\n", message); \
        airspyClose(); \
        return false; \
        } \

    int status;
    uint32_t count, *samplerates;
    static const char *aspymod[] = { "MINI", "R2" };

    if (AIRSPY.model == BADMODEL) {
        fprintf(stderr, "Airspy error: You must specify an Airspy model with --model\n");
        return false;
    }

    if (Modes.mode_ac) {
        fprintf(stderr,
            "\nWarning: --modeac ignored (Airspy support not currently implemented);\n"
            "         no ModeA/C messages will be decoded.\n\n");
    }

    if (AIRSPY.serial_number) {
        status = airspy_open_sn(&AIRSPY.dev, AIRSPY.serial_number);
        AIRSPY_STATUS(status, "airspy_open_sn failed");
        fprintf(stderr, "Airspy %s with serial number 0x%" PRIX64 " found, ", aspymod[AIRSPY.model], AIRSPY.serial_number);
    } else {
        status = airspy_open(&AIRSPY.dev);
        AIRSPY_STATUS(status, "No Airspy compatible devices found");
        fprintf(stderr, "Airspy %s found, ", aspymod[AIRSPY.model]);
    }

    fprintf(stderr, "configuring...\n");

    status = airspy_get_samplerates(AIRSPY.dev, &count, 0);
    AIRSPY_STATUS(status, "airspy_get_samplerates failed");
    samplerates = (uint32_t *)malloc(count * sizeof(uint32_t));
    airspy_get_samplerates(AIRSPY.dev, samplerates, count);
    // Set the samplerate to the highest value available
    // mini = 6 Msps, r2 = 10 Msps
    status = airspy_set_samplerate(AIRSPY.dev, samplerates[0]);
    AIRSPY_STATUS(status, "airspy_set_samplerate failed");
    AIRSPY.sample_rate = samplerates[0];
    free(samplerates);

    // We'll do our own real->mag conversion vs. doing it in libairspy
    status = airspy_set_sample_type(AIRSPY.dev, AIRSPY_SAMPLE_INT16_REAL);
    AIRSPY_STATUS(status, "airspy_set_sample_type failed");

    status = airspy_set_freq(AIRSPY.dev, Modes.freq);
    AIRSPY_STATUS(status, "airspy_set_freq failed");

    if (AIRSPY.enable_linearity) {
        status = airspy_set_linearity_gain(AIRSPY.dev, AIRSPY.linearity_gain);
        AIRSPY_STATUS(status, "airspy_set_linearity_gain failed");
        fprintf(stderr, "Linearity Mode Gain: %i, ", AIRSPY.linearity_gain);
    }
    else if (AIRSPY.enable_sensitivity) {
        status = airspy_set_sensitivity_gain(AIRSPY.dev, AIRSPY.sensitivity_gain);
        AIRSPY_STATUS(status, "airspy_set_sensitivity_gain failed");
        fprintf(stderr, "Sensitivity Mode Gain: %i, ", AIRSPY.sensitivity_gain);
    }
    else {
        if (AIRSPY.enable_mixer_agc) {
            status = airspy_set_mixer_agc(AIRSPY.dev, 1);
            AIRSPY_STATUS(status, "airspy_set_mixer_agc failed");
            fprintf(stderr, "MIXER Gain: AGC, ");
        } 
        else {
            status = airspy_set_mixer_gain(AIRSPY.dev, AIRSPY.mixer_gain);
            AIRSPY_STATUS(status, "airspy_set_mixer_gain failed");
            fprintf(stderr, "MIXER Gain: %i, ", AIRSPY.mixer_gain);
        }

        if (AIRSPY.enable_lna_agc) {
            status = airspy_set_lna_agc(AIRSPY.dev, 1);
            AIRSPY_STATUS(status, "airspy_set_lna_agc failed");
            fprintf(stderr, "LNA Gain: AGC, ");
        } 
        else {
            status = airspy_set_lna_gain(AIRSPY.dev, AIRSPY.lna_gain);
            AIRSPY_STATUS(status, "airspy_set_lna_gain failed");
            fprintf(stderr, "LNA Gain: %i, ", AIRSPY.lna_gain);
        }

        status = airspy_set_vga_gain(AIRSPY.dev, AIRSPY.vga_gain);
        AIRSPY_STATUS(status, "airspy_set_vga_gain failed");
        fprintf(stderr, "VGA Gain: %i, ", AIRSPY.vga_gain);
    }

    if (AIRSPY.enable_airspy_biast) {
        status = airspy_set_rf_bias(AIRSPY.dev, 1);
        AIRSPY_STATUS(status, "airspy_set_rf_bias (on) failed");
        fprintf(stderr, "Bias-t: On, ");
    }
    else {
        status = airspy_set_rf_bias(AIRSPY.dev, 0);
        AIRSPY_STATUS(status, "airspy_set_rf_bias (off) failed");
        fprintf(stderr, "Bias-t: Off, ");
    }

    if (AIRSPY.enable_bit_packing) {
        status = airspy_set_packing(AIRSPY.dev, 1);
        AIRSPY_STATUS(status, "airspy_set_packing (on) failed");
        fprintf(stderr, "Packing: On\n");
    }
    else {
        status = airspy_set_packing(AIRSPY.dev, 0);
        AIRSPY_STATUS(status, "airspy_set_packing (off) failed");
        fprintf(stderr, "Packing: Off\n");
    }

    AIRSPY.converter = init_converter(INPUT_UC8,
                                      AIRSPY.sample_rate,
                                      Modes.dc_filter,
                                      &AIRSPY.converter_state);

    if (!AIRSPY.converter) {
        fprintf(stderr, "airspy: can't initialize sample converter\n");
        airspyClose();
        return false;
    }

    fprintf(stderr, "Airspy successfully initialized\n\n");

    return true;
}

void *real_converter(void *tid) {
    int16_t queue[QUEUESIZE];
    int queueidx = HILBERT_SIZE - 1;
    unsigned i, start, mytid, end;

    // Configure this threads sample block to process
    mytid = * (unsigned *)tid;
    start = mytid * (AIRSPY.num_samples / NTHREADS);
    end = start + (AIRSPY.num_samples / NTHREADS);
    if ((mytid == NTHREADS - 1) && (end != AIRSPY.num_samples))
        end = AIRSPY.num_samples;

    uint16_t *in = AIRSPY.samples + start;
    uint16_t *out = AIRSPY.magbuf + start;

    for (i = start; i < end; ++i) {
        int j;
        float I, Q;

        queue[queueidx] = (int16_t)le16toh(*in++);

        // nb: this gets the signs of I/Q wrong sometimes,
        // but it doesn't matter because we're just going
        // to square them anyway

        I = queue[queueidx - HILBERT_SIZE/2] * 0.5;

        Q = 0;
        for (j = 0; j < HILBERT_SIZE; j += 2) {
            Q += queue[queueidx - HILBERT_SIZE + j + 1] * hilbert[j];
        }

        ++queueidx;
        if (queueidx >= QUEUESIZE) {
            memcpy(&queue[0], &queue[queueidx - HILBERT_SIZE], HILBERT_SIZE * sizeof(queue[0]));
            queueidx = HILBERT_SIZE;
        }

        float mag = sqrtf(I*I + Q*Q) * (65536.0 / 32768.0);
        if (mag > 65535)
            mag = 65535;

        *out++ = (uint16_t)mag;
    }

    pthread_exit(NULL); 
}

static struct timespec airspy_thread_cpu;

int airspyCallback(airspy_transfer *transfer) {
    size_t slen = transfer->sample_count;
    struct mag_buf *outbuf;
    struct mag_buf *lastbuf;
    unsigned next_free_buffer;
    unsigned free_bufs;
    unsigned block_duration;
    int i, tids[NTHREADS];
    pthread_t threads[NTHREADS];
    static int dropping = 0;
    static uint64_t sampleCounter = 0;
    //uint64_t dropped_samps = transfer->dropped_samples;

    if (Modes.exit) {
        return -1;
    }

    // Lock the data buffer variables before accessing them
    pthread_mutex_lock(&Modes.data_mutex);

    next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
    outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
    lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
    free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;
    AIRSPY.samples = (uint16_t *)transfer->samples;
    AIRSPY.num_samples = transfer->sample_count;

    pthread_mutex_unlock(&Modes.data_mutex);

    //fprintf(stderr, "dropped samples: %" PRIu64 ", number of samples this callback: %zu\n", dropped_samps, slen);

    if (free_bufs == 0 || (dropping && free_bufs < MODES_MAG_BUFFERS/2)) {
        // FIFO is full. Drop this block.
        dropping = 1;
        outbuf->dropped += slen;
        sampleCounter += slen;
        return 0;
    }

    dropping = 0;

    // Compute the sample timestamp and system timestamp for the start of the block
    outbuf->sampleTimestamp = sampleCounter + 12e6 / AIRSPY.sample_rate;
    sampleCounter += slen;
    block_duration = 1e9 * slen / AIRSPY.sample_rate;

    // Get the approx system time for the start of this block
    clock_gettime(CLOCK_REALTIME, &outbuf->sysTimestamp);
    outbuf->sysTimestamp.tv_nsec -= block_duration;
    normalize_timespec(&outbuf->sysTimestamp);

    // Copy trailing data from last block (or reset if not valid)
    if (outbuf->dropped == 0) {
        memcpy(outbuf->data, lastbuf->data + lastbuf->length, Modes.trailing_samples * sizeof(uint16_t));
    } else {
        memset(outbuf->data, 0, Modes.trailing_samples * sizeof(uint16_t));
    }

    // Convert the new data
    outbuf->length = slen;

    AIRSPY.magbuf = &outbuf->data[Modes.trailing_samples];

    for (i = 0; i < NTHREADS; i++)
    {
        tids[i] = i;
        if ( pthread_create(&threads[i], NULL, real_converter, (void *)&tids[i]) ) {
            fprintf(stderr, "Cannot create converter threads");
            return (-1);
        }
    }

    for (i = 0; i < NTHREADS; i++)
        pthread_join(threads[i], NULL);

    // Push the new data to the demodulation thread
    pthread_mutex_lock(&Modes.data_mutex);

    Modes.mag_buffers[next_free_buffer].dropped = 0;
    Modes.mag_buffers[next_free_buffer].length = 0;  // just in case
    Modes.first_free_buffer = next_free_buffer;

    // Accumulate CPU while holding the mutex, and restart measurement
    end_cpu_timing(&airspy_thread_cpu, &Modes.reader_cpu_accumulator);
    start_cpu_timing(&airspy_thread_cpu);

    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);

    return (0);
}

void airspyRun()
{
    if (!AIRSPY.dev) {
        return;
    }

    start_cpu_timing(&airspy_thread_cpu);

    airspy_start_rx(AIRSPY.dev, airspyCallback, NULL);

    while ((airspy_is_streaming(AIRSPY.dev) == AIRSPY_TRUE) && (!Modes.exit)) {
        usleep(5000);
    }
}

void airspyDemod(struct mag_buf *buf, int acFlag)
{
    MODES_NOTUSED(acFlag);

    if (AIRSPY.model == MINI)
       demodulate12m(buf);
    else
       demodulate20m(buf);
}

void airspyClose()
{
    if (AIRSPY.dev) {
        if (airspy_is_streaming(AIRSPY.dev) == AIRSPY_TRUE)
            airspy_stop_rx(AIRSPY.dev);
        airspy_close(AIRSPY.dev);
        AIRSPY.dev = NULL;
    }

    if (AIRSPY.converter) {
        cleanup_converter(AIRSPY.converter_state);
        AIRSPY.converter = NULL;
        AIRSPY.converter_state = NULL;
    }
}
