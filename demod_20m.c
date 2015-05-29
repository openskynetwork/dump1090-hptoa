// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// demod_20m.c: 20MHz Mode S demodulator.
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
// Given 'mlen' magnitude samples in 'm', sampled at 20MHz,
// try to demodulate some Mode S messages.
//
void demodulate20m(struct mag_buf *mag)
{
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
        if (m[j] >= m[j+5])
            continue;

        // Pulses in the right places?
        if (m[j+5] < m[j+15])
            continue;
        if (m[j+15] > m[j+25])
            continue;
        if (m[j+25] < m[j+35])
            continue;
        if (m[j+65] > m[j+75])
            continue;
        if (m[j+75] < m[j+85])
            continue;
        if (m[j+85] > m[j+95])
            continue;
        if (m[j+95] < m[j+105])
            continue;

        // Correlate.
        // (yes, this could be cleverer)
        phase = -1;
        bestscore = -1;
        for (i = 0; i < 10; ++i) {
            int score = 0;
            int k;
            for (bit = 0; bit < 10; ++bit) {
                if (preamble[bit] == 0)
                    continue;
                for (k = 0; k < 10; ++k) {
                    score += m[j + bit*10 + i + k] * preamble[bit];
                }
            }

            if (phase == -1 || score > bestscore) {
                phase = i;
                bestscore = score;
            }
        }

        j += phase;
            
        // Check quiet zones
        peak = m[j+5];
        if (m[j+25] > peak)
            peak = m[j+25];
        if (m[j+75] > peak)
            peak = m[j+25];
        if (m[j+95] > peak)
            peak = m[j+95];
        
        peak = peak * 2/3;
        if (m[j+15] > peak ||
            m[j+35] > peak ||
            m[j+45] > peak ||
            m[j+55] > peak ||
            m[j+65] > peak ||
            m[j+85] > peak ||
            m[j+105] > peak ||
            m[j+115] > peak ||
            m[j+125] > peak ||
            m[j+135] > peak ||
            m[j+145] > peak ||
            m[j+155] > peak)
            continue;

        // OK, looks plausible. demodulate.
        Modes.stats_current.demod_preambles++;

        for (i = 0; i < MODES_LONG_MSG_BYTES; ++i) {
            uint16_t *b = &m[j+160+i*160];
#define SLICE(x) ((b[x+4] + b[x+5] + b[x+6]) > (b[x+14] + b[x+15] + b[x+16]))
            msg[i] =
                (SLICE(0) ? 0x80 : 0) |
                (SLICE(20) ? 0x40 : 0) |
                (SLICE(40) ? 0x20 : 0) |
                (SLICE(60) ? 0x10 : 0) |
                (SLICE(80) ? 0x08 : 0) |
                (SLICE(100) ? 0x04 : 0) |
                (SLICE(120) ? 0x02 : 0) |
                (SLICE(140) ? 0x01 : 0);
        }

        msglen = modesMessageLenByType(msg[0] >> 3);

        // Set initial mm structure details
        mm.timestampMsg = mag->sampleTimestamp + j * 20 / 12;

        // compute message receive time as block-start-time + difference in the 12MHz clock
        mm.sysTimestampMsg = mag->sysTimestamp; // start of block time
        mm.sysTimestampMsg.tv_nsec += receiveclock_ns_elapsed(mag->sampleTimestamp, mm.timestampMsg);
        normalize_timespec(&mm.sysTimestampMsg);

        mm.bFlags = mm.correctedbits   = 0;

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

        j += (msglen-8) * 20;
    }
}

