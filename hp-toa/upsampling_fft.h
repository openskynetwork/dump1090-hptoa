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

#ifndef UPSAMPLING_H
#define UPSAMPLING_H

#ifdef __cplusplus

#include <complex>
#include <math.h>
#include <string.h>

extern "C" {
#include "gpu_fft_3/gpu_fft.h"
};

#ifndef FFT_GPU
    #include <liquid/liquid.h>
#else
extern "C"{
    #include "gpu_fft_3/mailbox.h"
};
#endif

#include <mutex>
#include <vector>

#define  UPSAMPLING_FACTOR  16.0

GPU_FFT_COMPLEX* upsampling (std::complex<float>* signal, unsigned int signal_len, double up_factor);
void release_upsampling();


#endif
#endif
