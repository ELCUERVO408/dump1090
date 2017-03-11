// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// demod_12m.c: 12MHz Mode S demodulator.
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
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

#include "dump1090.h"

//
// Given 'mlen' magnitude samples in 'm', sampled at 12MHz,
// try to demodulate some Mode S messages.
//
void demodulate12m(struct mag_buf *mag)
{
    static struct modesMessage zeroMessage;
    struct modesMessage mm;
    unsigned char msg[MODES_LONG_MSG_BYTES];
    uint32_t j;
    int preamble[10] = {1, -1, 1, 0, 0, 0, 0, 1, -1, 1};

    uint16_t *m = mag->data;
    uint32_t mlen = mag->length;

    memset(&mm, 0, sizeof(mm));

    for (j = 0; j < mlen; j++) {
        int i;
        int bit;
        int phase, bestscore;
        int peak;
        int msglen;

        // Look for a rising edge
        if (m[j] >= m[j+3])
            continue;

        // Pulses in the right places?
        if (m[j+3] < m[j+9])
            continue;
        if (m[j+9] > m[j+15])
            continue;
        if (m[j+15] < m[j+21])
            continue;
        if (m[j+39] > m[j+45])
            continue;
        if (m[j+45] < m[j+51])
            continue;
        if (m[j+51] > m[j+57])
            continue;
        if (m[j+57] < m[j+63])
            continue;

        // Correlate.
        // (yes, this could be cleverer)
        phase = -1;
        bestscore = -1;
        for (i = 0; i < 6; ++i) {
            int score = 0;
            int k;
            for (bit = 0; bit < 6; ++bit) {
                if (preamble[bit] == 0)
                    continue;
                for (k = 0; k < 6; ++k) {
                    score += m[j + bit*6 + i + k] * preamble[bit];
                }
            }

            if (phase == -1 || score > bestscore) {
                phase = i;
                bestscore = score;
            }
        }

        j += phase;
            
        // Check quiet zones
        peak = m[j+3];
        if (m[j+15] > peak)
            peak = m[j+15];
        if (m[j+45] > peak)
            peak = m[j+45];
        if (m[j+57] > peak)
            peak = m[j+57];
        
        peak = peak * 2/3;
        if (m[j+9] > peak ||
            m[j+21] > peak ||
            m[j+27] > peak ||
            m[j+33] > peak ||
            m[j+39] > peak ||
            m[j+51] > peak ||
            m[j+63] > peak ||
            m[j+69] > peak ||
            m[j+75] > peak ||
            m[j+81] > peak ||
            m[j+87] > peak ||
            m[j+93] > peak)
            continue;

        // OK, looks plausible. demodulate.
        Modes.stats_current.demod_preambles++;

        for (i = 0; i < MODES_LONG_MSG_BYTES; ++i) {
            uint16_t *b = &m[j+96+i*96];
#define SLICE(x) ((b[x+2] + b[x+3]) > (b[x+8] + b[x+9]))
            msg[i] =
                (SLICE(0) ? 0x80 : 0) |
                (SLICE(12) ? 0x40 : 0) |
                (SLICE(24) ? 0x20 : 0) |
                (SLICE(36) ? 0x10 : 0) |
                (SLICE(48) ? 0x08 : 0) |
                (SLICE(60) ? 0x04 : 0) |
                (SLICE(72) ? 0x02 : 0) |
                (SLICE(84) ? 0x01 : 0);
        }

        msglen = modesMessageLenByType(msg[0] >> 3);

        // Set initial mm structure details
        mm = zeroMessage;
        //mm.timestampMsg = mag->sampleTimestamp + j * 20 / 12;
        mm.timestampMsg = mag->sampleTimestamp + j;

        // compute message receive time as block-start-time + difference in the 12MHz clock
        mm.sysTimestampMsg = mag->sysTimestamp; // start of block time
        mm.sysTimestampMsg.tv_nsec += receiveclock_ns_elapsed(mag->sampleTimestamp, mm.timestampMsg);
        normalize_timespec(&mm.sysTimestampMsg);

        //mm.bFlags = mm.correctedbits   = 0;
        mm.correctedbits   = 0;

        // Decode the received message
        {
            int result = decodeModesMessage(&mm, msg);
            if (result < 0) {
                if (result == -1)
                    Modes.stats_current.demod_rejected_unknown_icao++;
                else
                    Modes.stats_current.demod_rejected_bad++;
                continue;
            } else {
                Modes.stats_current.demod_accepted[mm.correctedbits]++;
            }
        }

        // Pass data to the next layer
        useModesMessage(&mm);

        // Skip over the message:
        // (we actually skip to 8 bits before the end of the message,
        //  because we can often decode two messages that *almost* collide,
        //  where the preamble of the second message clobbered the last
        //  few bits of the first message, but the message bits didn't
        //  overlap)

        //j += (msglen-8) * 20;
        j += (msglen-8) * 12;
    }
}

