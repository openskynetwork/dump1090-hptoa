/*
 * Copyright (C) IMDEA Networks Institute
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * 	Author: 	Roberto Calvo-Palomino <roberto.calvo@imdea.org>
 *
 */

/*
 *
 * This code is based on the work named
 * 'Nanosecond-precision Time-of-Arrival Estimation for Aircraft Signals with low-cost SDR Receivers'
 * (http://eprints.networks.imdea.org/1768/) published in ACM/IPSN 2018 conference
 * and made by:
 * Roberto Calvo-Palomino, Fabio Ricciato, Blaz Repas, Domenico Giustiniano, Vincent Lenders.
 *
 */

#ifndef HP_TOA_TIMESTAMP_H
#define HP_TOA_TIMESTAMP_H

#ifdef __cplusplus

#include "readerwriterqueue.h"
#include "../dump1090.h"
#include <mutex>
#include <thread>
#include <algorithm>
#include <complex>
#include <vector>

extern "C" {
#include "gpu_fft_3/gpu_fft.h"
};

using namespace moodycamel;

struct hptoaMessage {
    struct queueMessage* qMessage;
    GPU_FFT_COMPLEX* signal_upsampled;
    unsigned int signal_upsampled_len;
};

class HPTOA {
public:

    void init_queue();
    void push_message(struct hptoaMessage *message);
    void end_queue();

private:

    void thread();

    double get_hptoa_peak_pulse (GPU_FFT_COMPLEX* signal_upsampled, unsigned int signal_upsampled_len,
                                 unsigned int signal_front_margin, unsigned int init_packet_sample,
                                 double up_factor, int bits_decoded[], int bits_len);

    double get_hptoa_corr_pulse (GPU_FFT_COMPLEX* signal_upsampled, unsigned int signal_upsampled_len,
                                 unsigned int signal_front_margin, unsigned int init_packet_sample,
                                 double up_factor, int bits_decoded[], int bits_len);

    int cross_correlation (double* signal1, double* signal2, unsigned int length);


    ReaderWriterQueue<hptoaMessage*> *mHPTOAQueue;
    std::thread *mHPTOAThread;
    bool mRunning = true;

    std::mutex m_mutex, q_mutex;

    const double SAMPLING_RATE = 2.4;
};


#endif

#endif