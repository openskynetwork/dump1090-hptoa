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


#include "upsampling_fft.h"

// Un-comment to enable the debug
// #define DEBUG_HPTOA


#ifdef FFT_GPU
    static std::mutex mutex_up;
    static int gpu_init = 0;
    static int ioctl_mb;
    struct GPU_FFT *gpu_fft = NULL, *gpu_fft_inverse = NULL;
#endif


GPU_FFT_COMPLEX* upsampling (std::complex<float>* signal, unsigned int signal_len, double up_factor) {



    int size_upsampling = signal_len*(int)up_factor;

    GPU_FFT_COMPLEX* signal_upsampled = (GPU_FFT_COMPLEX*) malloc(sizeof(GPU_FFT_COMPLEX)*size_upsampling);

#ifndef FFT_GPU

    std::complex<float> signal_freq[signal_len];

    int flags=0;

    fftplan q_f = fft_create_plan(signal_len, signal, signal_freq, LIQUID_FFT_FORWARD, flags);
    fft_execute(q_f);

    fft_shift(signal_freq,signal_len);

#ifdef DEBUG_HPTOA

    printf("///////////////// signal_freq /////////////////\n");
        for (int s=0; s<signal_len ; s++)
            printf("%f%+fi , \n", signal_freq[s].real(), signal_freq[s].imag());
        printf("\n");

#endif

    fft_destroy_plan(q_f);

    std::complex<float> padded_spectrum[size_upsampling];

    memcpy(&padded_spectrum[(size_upsampling/2)-(signal_len/2)], signal_freq, sizeof(std::complex<float>)*signal_len);

    std::complex<float> signal_res[size_upsampling];
    fft_shift(padded_spectrum,size_upsampling);
    fftplan q_b = fft_create_plan(size_upsampling, padded_spectrum, signal_res, LIQUID_FFT_BACKWARD, flags);
    fft_execute(q_b);

    for (int i=0; i<size_upsampling; i++) {
        signal_upsampled[i].re = signal_res[i].real();
        signal_upsampled[i].im = signal_res[i].imag();
    }

#ifdef DEBUG_HPTOA

    printf("///////////////// final /////////////////\n");
        for (int s=0; s<size_upsampling ; s++)
            printf("%f%+fi , ", signal_upsampled[s].real(), signal_upsampled[s].imag());
        printf("\n");

#endif

    fft_destroy_plan(q_b);

#else

    mutex_up.lock();

    // Using GPU of RPi to perform FFT/IFFT
    // Upsampling factor and log2_N are related!!
    int _N = 0;
    if (!gpu_init) {

        int log2_N = 0, _BATCHSIZE=1;
        log2_N = 9; // 512
        _N = 1<<log2_N;
        ioctl_mb = mbox_open();
        int r = gpu_fft_prepare(ioctl_mb, log2_N, GPU_FFT_FWD, _BATCHSIZE, &gpu_fft);
        switch(r) {
              case -1:
                      fprintf(stderr, "Unable to enable V3D. Please check your firmware is up to date.\n");
                      exit(1);
              case -2:
                      fprintf(stderr, "log2_N=%d not supported. Try between 8 and 17.\n", log2_N);
                      exit(1);
              case -3:
                      fprintf(stderr, "Out of memory. Try a smaller batch or increase GPU memory.\n");
                      exit(1);
              }

        log2_N = 13;
        _N = 1<<log2_N;

        r = gpu_fft_prepare(ioctl_mb, log2_N, GPU_FFT_REV, _BATCHSIZE, &gpu_fft_inverse);
        switch(r) {
            case -1:
                fprintf(stderr, "Unable to enable V3D. Please check your firmware is up to date.\n");
                exit(1);
            case -2:
                fprintf(stderr, "log2_N=%d not supported. Try between 8 and 17.\n", log2_N);
                exit(1);
            case -3:
                fprintf(stderr, "Out of memory. Try a smaller batch or increase GPU memory.\n");
                exit(1);
        }
        gpu_init = 1;
    }


    // Prepare FFT Input
    struct GPU_FFT_COMPLEX *base;


    base = gpu_fft->in;
    for (int i=0; i<signal_len; i++)
    {
        base[i].re = signal[i].real();
        base[i].im = signal[i].imag();
    }


#ifdef DEBUG_HPTOA_GPU_FFT
    printf("///////////////// signal (time) /////////////////\n");
    for (int s=0; s<signal_len ; s++)
        printf("%f%+fi , ", base[s].re, base[s].im);
    printf("\n");
#endif

    gpu_fft_execute(gpu_fft);

    //fft_shift
    struct GPU_FFT_COMPLEX aux[signal_len/2];
    memcpy(aux,gpu_fft->out, sizeof(struct GPU_FFT_COMPLEX)*(signal_len/2));
    memcpy(gpu_fft->out,&gpu_fft->out[(signal_len/2)-1], sizeof(struct GPU_FFT_COMPLEX)*(signal_len/2));
    memcpy(&gpu_fft->out[(signal_len/2)], aux, sizeof(struct GPU_FFT_COMPLEX)*(signal_len/2));

#ifdef DEBUG_HPTOA_GPU_FFT
    printf("///////////////// signal (freq) /////////////////\n");
    for (int s=0; s<signal_len ; s++)
        printf("%f%+fi , ", gpu_fft->out[s].re, gpu_fft->out[s].im);
    printf("\n");
#endif

    unsigned int upsampling_len = signal_len*up_factor;
    memset(gpu_fft_inverse->in, 0, sizeof(struct GPU_FFT_COMPLEX)*upsampling_len);
    memcpy(&gpu_fft_inverse->in[(upsampling_len/2)-(signal_len/2)], gpu_fft->out, sizeof(struct GPU_FFT_COMPLEX)*signal_len);

#ifdef DEBUG_HPTOA_GPU_FFT
    printf("///////////////// upsampling (freq) /////////////////\n");
    for (int s=0; s<upsampling_len ; s++)
        printf("%f%+fi , ", gpu_fft_inverse->in[s].re, gpu_fft_inverse->in[s].im);
    printf("\n");
#endif

    //fft_shift

    unsigned int half_len = upsampling_len/2;
    struct GPU_FFT_COMPLEX aux2[half_len];
    memcpy(aux2,gpu_fft_inverse->in, sizeof(struct GPU_FFT_COMPLEX)*half_len);
    memcpy(gpu_fft_inverse->in,&gpu_fft_inverse->in[half_len-1], sizeof(struct GPU_FFT_COMPLEX)*half_len);
    memcpy(&gpu_fft_inverse->in[half_len], aux2, sizeof(struct GPU_FFT_COMPLEX)*half_len);



#ifdef DEBUG_HPTOA_GPU_FFT
    printf("///////////////// upsampling shift (freq) /////////////////\n");
    for (int s=0; s<upsampling_len ; s++)
        printf("%f%+fi , ", gpu_fft_inverse->in[s].re, gpu_fft_inverse->in[s].im);
    printf("\n");
#endif

    gpu_fft_execute(gpu_fft_inverse);

#ifdef DEBUG_HPTOA_GPU_FFT
    printf("///////////////// upsampling /////////////////\n");
    for (int s=0; s<upsampling_len ; s++)
        printf("%f%+fi , ", gpu_fft_inverse->out[s].re, gpu_fft_inverse->out[s].im);
    printf("\n");
#endif


    memcpy(signal_upsampled, gpu_fft_inverse->out, sizeof(GPU_FFT_COMPLEX)*upsampling_len);

    mutex_up.unlock();
#endif

    return signal_upsampled;

}

void release_upsampling() {

#ifdef FFT_GPU

    mutex_up.lock();
    if (gpu_fft) {
        gpu_fft_release(gpu_fft);
        mbox_close(ioctl_mb);
    }

    if (gpu_fft_inverse)
        gpu_fft_release(gpu_fft_inverse);

    mutex_up.unlock();

#endif

}
