/*
 * Copyright (C) OpenSky Network 2018
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


#include "HP-TOA.h"

// Un-comment to enable the debug
// #define DEBUG_HPTOA

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))


resamp_crcf q;
unsigned int init = 0;


double get_hp_toa (float complex signal[], unsigned int signal_len, int signal_front_margin,
                  unsigned int init_packet_sample,
                  double up_factor,
                  int bits_decoded[], unsigned int bits_len) {


    double SAMPLING_RATE = 2.4;

    double chip_p = 0.5; // microseconds
    float bw = 0.45f;  // resampling filter bandwidth

    int new_samplingrate = up_factor * SAMPLING_RATE;
    int s_chip = chip_p * new_samplingrate;

    unsigned int m = 10;    // delay
    float As = 60.0f;       // resampling filter stop-band attenuation [dB
    unsigned int npfb = 64; // number of filters in bank (timing resolution)

    unsigned int nx = signal_len;


#ifdef DEBUG_HPTOA

    for (unsigned int i=0; i<nx; i++) {
        printf("%.2f, ", cabs(signal[i]));
    }
    printf("\n");

#endif

    unsigned int y_len = (unsigned int) ceilf(1.1 * nx * up_factor) + 4;

    float complex
    y[y_len];

    // create resampler
    // This funcion can be reused, it has to be called just once.
    if (init==0) {
        q = resamp_crcf_create((float)up_factor, m, bw, As, npfb);
        init = 1;
    }

    unsigned int ny = 0;
    resamp_crcf_execute_block(q, signal, nx, y, &ny);

#ifdef DEBUG_HPTOA

    printf("Size original signal: %d\n",nx);
    printf("Size upsampled signal: %d\n",ny);

    // run FFT and ensure that carrier has moved and that image
    // frequencies and distortion have been adequately suppressed

    unsigned int nfft = 1 << liquid_nextpow2(ny);
    float complex
    yfft[nfft];   // fft input
    float complex
    Yfft[nfft];   // fft output
    for (unsigned int i = 0; i < nfft; i++)
        yfft[i] = i < ny ? y[i] : 0.0f;

    fft_run(nfft, yfft, Yfft, LIQUID_FFT_FORWARD, 0);
    fft_shift(Yfft, nfft);      // run FFT shift

    FILE *fd = fopen("/tmp/signal_up.bin", "wb");
    float mI,mQ;

    for (unsigned int o=1; o<ny; o++)
    {
        mI = crealf(y[o]);
        mQ = cimagf(y[o]);
        fwrite(&mI, sizeof(float), 1, fd);
        fwrite(&mQ, sizeof(float), 1, fd);
    }
    fclose(fd);

#endif


    // Bits-chips conversion
    int chips[bits_len*2];
    int pointer = 0;

    for (unsigned int k=0; k<bits_len; k=k+2) {
        if (bits_decoded[k]==0 && bits_decoded[k+1]==0) {
            chips[pointer]=0; chips[pointer+1]=1; chips[pointer+2]=0; chips[pointer+3]=1;
            pointer += 4;
        }
        else if (bits_decoded[k]==0 && bits_decoded[k+1]==1) {
            chips[pointer]=0; chips[pointer+1]=1; chips[pointer+2]=1; chips[pointer+3]=0;
            pointer += 4;
        }
        else if (bits_decoded[k]==1 && bits_decoded[k+1]==0) {
            chips[pointer]=1; chips[pointer+1]=0; chips[pointer+2]=0; chips[pointer+3]=1;
            pointer += 4;
        }
        else if (bits_decoded[k]==1 && bits_decoded[k+1]==1) {
            chips[pointer]=1; chips[pointer+1]=0; chips[pointer+2]=1; chips[pointer+3]=0;
            pointer += 4;
        }
    }

    // Remove Type-II pulses
    for (unsigned int k=0; k<NELEMS(chips); k=k+1){
        if (chips[k]==0 && chips[k+1]==1 && chips[k+2]==1 && chips[k+3]==0) {
            chips[k+1]=0; chips[k+2]=0;
        }
    }

    // Detect type_I pulses
    double payload_start = 8.0;     // microsecond
    double peaks_I[200];            // max
    unsigned int peaks_size=0;

    // Nominal pulses for the preamble
    peaks_I[0]=0; peaks_I[1]=1.0; peaks_I[2]=3.5; peaks_I[3]=4.5;       // from the standard
    peaks_size +=4;

    for (unsigned int k=0; k<NELEMS(chips); k++){
        if (chips[k]==1) {
            peaks_I[peaks_size]=payload_start+k*chip_p;
            peaks_size++;
        }
    }

#ifdef DEBUG_HPTOA

    for (unsigned int i=0; i<peaks_size; i++)
        printf("%.2f\n", peaks_I[i]);

    printf("***\n");

#endif

    int start_packet = signal_front_margin*up_factor + (0*new_samplingrate) + (m*up_factor);

    // Nominal peaks
    int nominal_peaks_samples[peaks_size];

    // Estimated peaks
    int real_peaks_samples[peaks_size];
    int sample_correction = 0;
    int slice_begin, slice_end, amp_vector_size, max_pos;
    double max_value;
    int delta_peaks=0;

    // peaks_I -> Nominal time of the peaks
    // nominal_peaks_samples -> Nominal peaks samples counting from

    // We try to do almost everything in the same for
    for (unsigned int k=0; k<peaks_size; k++) {

        // Nominal pulses
        if (k == 0) {
            nominal_peaks_samples[k] = start_packet + (peaks_I[k] * new_samplingrate) + s_chip;
            slice_begin = nominal_peaks_samples[k] - s_chip;
            slice_end = nominal_peaks_samples[k] + s_chip;
        }
            // Delta peak (k==1) correction is applied to all
            // nominal peaks.
        else {
            nominal_peaks_samples[k] = sample_correction + (peaks_I[k] * new_samplingrate);
            slice_begin = nominal_peaks_samples[k] - s_chip / 2;
            slice_end = nominal_peaks_samples[k] + s_chip / 2;
        }

        amp_vector_size = slice_end - slice_begin;
        double amp_vector[amp_vector_size];



        // Compute the amplitude and get the max
        max_value = 0.0;
        max_pos = 0;

        // Compute the amplitude
        for (int l = 1; l < amp_vector_size; l++) {
            amp_vector[l] = cabs(y[slice_begin + l]);
        }


        // Find peaks, p should be higher than p-1 and p+1
        for (int l = 2; l < amp_vector_size - 2; l++) {
            if (amp_vector[l] >= amp_vector[l + 1] && amp_vector[l] >= amp_vector[l + 2] &&
                amp_vector[l] >= amp_vector[l - 1] && amp_vector[l] >= amp_vector[l - 2] &&
                amp_vector[l] >= max_value) {
                max_value = amp_vector[l];
                max_pos = slice_begin + l;
            }
        }
        real_peaks_samples[k] = max_pos + 1;


#ifdef DEBUG_HPTOA
        printf(" * [ %d  |  %d  ] ->\t ", slice_begin, slice_end);

        if (real_peaks_samples[k] == 1)
            for (int l=2; l<amp_vector_size-2; l++)
                printf("%.2f,", amp_vector[l]);
#endif

        // Once the first peak is detected, the delta peak is saved
        if (k == 0) {
            sample_correction = real_peaks_samples[k] + 1;

#ifdef DEBUG_HPTOA
            printf(" N: %d - E: %d) \t %d\n", nominal_peaks_samples[k], real_peaks_samples[k] ,
                   real_peaks_samples[k] - nominal_peaks_samples[k]);

            printf("t=%.2f\n",
                  ( (init_packet_sample*up_factor + real_peaks_samples[k]) -
                  (init_packet_sample*up_factor)));
#endif
        }

        // compute the delta peak.
        // For k=0 we assume nominal_peak = estimated_peak
        if (k > 1) {
            // real_peak == 1 means that no peak was detected,
            // we don't use it for the HP-TOA estimation
            if (real_peaks_samples[k] != 1) {
                delta_peaks = delta_peaks + (real_peaks_samples[k] - nominal_peaks_samples[k]);

#ifdef DEBUG_HPTOA
                printf(" N: %d - E: %d) \t %d\n", nominal_peaks_samples[k], real_peaks_samples[k], real_peaks_samples[k] - nominal_peaks_samples[k]);
#endif
            }


        }

    }

#ifdef DEBUG_HPTOA

    for (int unsigned i=0; i<peaks_size; i++) {
        printf("N:%.4f ", ((nominal_peaks_samples[i] - start_packet) / SAMPLING_RATE / up_factor));
        printf("S:%.4f \t", ((real_peaks_samples[i] - start_packet) / SAMPLING_RATE / up_factor));
        printf(" %.4f \n", ((real_peaks_samples[i] - start_packet) - (nominal_peaks_samples[i] - start_packet))
                           / SAMPLING_RATE / up_factor);
    }

#endif


    double dpeaks = (delta_peaks/((peaks_size-1)*1.0));


    double toa_res = (( (init_packet_sample*up_factor +
            real_peaks_samples[0] + dpeaks)
            / SAMPLING_RATE / up_factor )) *1000;


    return toa_res;
}
