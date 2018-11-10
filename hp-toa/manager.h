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

#ifndef MANAGER_H
#define MANAGER_H

#include "../dump1090.h"

#ifdef __cplusplus

#include <stdio.h>
#include <iostream>
#include <complex>
#include <chrono>
#include "readerwriterqueue.h"
#include "upsampling_fft.h"
#include "hp-toa-timestamp.h"
#include "types.h"
#include <mutex>
#include <thread>

    using namespace moodycamel;

    extern "C" void init_hp_queue ();
    extern "C" void push_message (struct queueMessage* message);
    extern "C" void end_hp_queue ();
    extern "C" void thread(ReaderWriterQueue<queueMessage*> *queue);

    ReaderWriterQueue<queueMessage*> *mFFTQueue1 , *mFFTQueue2;
    std::thread *mUpsamplingThread1, *mUpsamplingThread2;

    HPTOA* mHPtimestamp;
    bool mRunning = true;
    bool mBalance = true;

    std::mutex m_mutex;

#else

    // Headers for the C files that call this library
    void init_hp_queue ();
    void end_hp_queue ();
    void push_message (struct queueMessage* message);

#endif


#endif
