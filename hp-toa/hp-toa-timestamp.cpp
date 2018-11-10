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

#include "hp-toa-timestamp.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

void HPTOA::init_queue () {

    //Initial size
    mHPTOAQueue = new ReaderWriterQueue<hptoaMessage*> (512);

    // HPTOA thread
    mHPTOAThread = new std::thread(&HPTOA::thread, this);
}

void HPTOA::end_queue () {

    // Wait until hp-toa is 0 size
    while( mHPTOAQueue->size_approx() != 0)
        ;;

    m_mutex.lock();
    mRunning = false;
    m_mutex.unlock();

    mHPTOAThread->join();

    delete(mHPTOAQueue);
    delete(mHPTOAThread);

}

void HPTOA::push_message (struct hptoaMessage* message){

    // May be using a concurrentqueue the performance will improve.

    q_mutex.lock();
    mHPTOAQueue->enqueue(message);
    q_mutex.unlock();
}

void HPTOA::thread() {

    hptoaMessage* msg;
    while(1) {

        m_mutex.lock();
        if (!mRunning) {
            m_mutex.unlock();
            break;
        }
        m_mutex.unlock();


        if (mHPTOAQueue->try_dequeue(msg)) {


            if (msg->qMessage->enable_hptoa) {


                double hp_toa=0.0;

                if (msg->qMessage->enable_hptoa == high_precision_t::PEAK_PULSE) {

                    hp_toa = this->get_hptoa_peak_pulse(msg->signal_upsampled, msg->signal_upsampled_len,
                                                        msg->qMessage->samples_front_margin,
                                                        msg->qMessage->start_packet_sample,
                                                        msg->qMessage->up_factor,
                                                        msg->qMessage->bits_decoded,
                                                        msg->qMessage->modes_message->msgbits);

                    msg->qMessage->modes_message->hpTimestampMsg = hp_toa;
                    msg->qMessage->modes_message->timestampMsg = (uint64_t) msg->qMessage->modes_message->hpTimestampMsg;

                } else if (msg->qMessage->enable_hptoa == high_precision_t::CORR_PULSE) {

                    hp_toa = this->get_hptoa_corr_pulse(msg->signal_upsampled, msg->signal_upsampled_len,
                                                        msg->qMessage->samples_front_margin,
                                                        msg->qMessage->start_packet_sample,
                                                        msg->qMessage->up_factor,
                                                        msg->qMessage->bits_decoded,
                                                        msg->qMessage->modes_message->msgbits);

                    msg->qMessage->modes_message->hpTimestampMsg = hp_toa;
                    msg->qMessage->modes_message->timestampMsg = (uint64_t) msg->qMessage->modes_message->hpTimestampMsg;
                }

                useModesMessage(msg->qMessage->modes_message);

                free(msg->qMessage->samples);
                free(msg->qMessage->modes_message);
                free(msg->qMessage->bits_decoded);
                free(msg->qMessage);
                free(msg->signal_upsampled);
                free(msg);

            }


        } else {
            usleep(0.1);
        }
    }

}




double HPTOA::get_hptoa_corr_pulse (GPU_FFT_COMPLEX* signal_upsampled, unsigned int signal_upsampled_len,
                       unsigned int signal_front_margin, unsigned int init_packet_sample,
                       double up_factor, int bits_decoded[], int bits_len) {

    std::complex<float> y[signal_upsampled_len];


    for (unsigned int i=0; i<signal_upsampled_len; i++)
    {
        y[i].real(signal_upsampled[i].re );
        y[i].imag(signal_upsampled[i].im );
    }

    double chip_p = 0.5; // microseconds

    double new_samplingrate = up_factor * SAMPLING_RATE;
    double s_chip = chip_p * new_samplingrate;

    // Bits-chips conversion
    int chips[bits_len*2];
    int pointer = 0;

    //auto start = std::chrono::steady_clock::now( );

    for (int k=0; k<bits_len; k=k+2) {
        if (bits_decoded[k]==0 && bits_decoded[k+1]==0) {
            // 0101
            chips[pointer]=0; chips[pointer+1]=1; chips[pointer+2]=0; chips[pointer+3]=1;
            pointer += 4;
        }
        else if (bits_decoded[k]==0 && bits_decoded[k+1]==1) {
            // Type-II pulses -> 0110. We don't add them.
            chips[pointer]=0; chips[pointer+1]=1; chips[pointer+2]=1; chips[pointer+3]=0;
            pointer += 4;
        }
        else if (bits_decoded[k]==1 && bits_decoded[k+1]==0) {
            // 1001
            chips[pointer]=1; chips[pointer+1]=0; chips[pointer+2]=0; chips[pointer+3]=1;
            pointer += 4;
        }
        else if (bits_decoded[k]==1 && bits_decoded[k+1]==1) {
            //1010
            chips[pointer]=1; chips[pointer+1]=0; chips[pointer+2]=1; chips[pointer+3]=0;
            pointer += 4;
        }
    }

    double payload_start = 8.0;     // microsecond
    double peaks_II[100];
    unsigned int peaks_II_size=0;


    // Detect Type_II pulses and remove them at the same time
    for (unsigned int k=0; k<NELEMS(chips)-4; k++) {

        if (chips[k]==0 && chips[k+1]==1 && chips[k+2]==1 && chips[k+3]==0) {

            peaks_II[peaks_II_size]=payload_start+k*chip_p;
            peaks_II_size++;

            // Remove Type-II pulses
            chips[k+1]=0; chips[k+2]=0;
        }
    }

    // Detect type_I pulses

    double peaks_I[200];            // max
    unsigned int peaks_I_size=0;


    // Nominal pulses for the preamble
    peaks_I[0]=0; peaks_I[1]=1.0; peaks_I[2]=3.5; peaks_I[3]=4.5;       // from the standard
    peaks_I_size +=4;

    for (unsigned int k=0; k<NELEMS(chips); k++){
        if (chips[k]==1) {
            peaks_I[peaks_I_size]=payload_start+k*chip_p;
            peaks_I_size++;
        }
    }


    int start_packet = signal_front_margin*up_factor; // + (0*new_samplingrate); // + (m*up_factor);

    // Nominal peaks
    int nominal_peaks_samples[peaks_I_size];

    // Estimated peaks
    int lag_peaks_samples[peaks_I_size];
    int sample_correction = 0;
    int slice_begin, slice_end;
    int delta_peaks=0;
    int delta_peaks_len=0;

    // peaks_I -> Nominal time of the peaks
    // nominal_peaks_samples -> Nominal peaks samples counting from


    unsigned int square_pulse_len = (int)round(up_factor*(112/83)); // int(up_factor*0.5*SAMPLING_RATE);
    if ( (square_pulse_len%2) != 0) square_pulse_len++;
    std::vector<double> square_pulse(square_pulse_len, 1.0);


    int margin_before = 1*up_factor;

    // We try to do almost everything in the same for
    for (unsigned int k=0; k<peaks_I_size; k++) {


        // Corr-Pulse method.
        if (k == 0) {
            nominal_peaks_samples[k] = start_packet + (int)round((peaks_I[k] * new_samplingrate) + s_chip);
            slice_begin = nominal_peaks_samples[k] - (int)round(square_pulse_len/2.0) - margin_before;
            slice_end = nominal_peaks_samples[k] + (int)round(square_pulse_len/2.0) + margin_before;
        }
            // Delta peak (k==1) correction is applied to all
            // nominal peaks.
        else {
            int m = (int)round(0.15*SAMPLING_RATE*up_factor);
            nominal_peaks_samples[k] = start_packet + sample_correction + (int) round((peaks_I[k] * new_samplingrate)  + s_chip);
            slice_begin = nominal_peaks_samples[k] -  (int)round(square_pulse_len/2.0) - m;
            slice_end = nominal_peaks_samples[k] +  (int)round(square_pulse_len/2.0) + m;
        }

        unsigned int corr_len = (unsigned int)(slice_end - slice_begin);
        double pulse_signal[corr_len];
        double template_signal[corr_len];

        for (int s = slice_begin; s < slice_end; s++) {
            pulse_signal[s - slice_begin] = std::abs(y[s]);
        }


        memset(template_signal, 0, sizeof(double) * corr_len);
        int pos = (int)round((corr_len / 2) - (square_pulse_len / 2));


        memcpy(&template_signal[pos], square_pulse.data(), sizeof(double) * square_pulse_len);


#ifdef DEBUG_HPTOA
        printf("*************** k = %d ************\n", k);
        printf("Size template: %d \n", corr_len);
        printf("Size pulse: %d (%d:%d)\n", slice_end-slice_begin, slice_begin, slice_end);
        printf("Nominal peak: %d\n", nominal_peaks_samples[k]);
        printf("pulse = [ ");
        for (unsigned int s=0; s<corr_len; s++)
            printf("%.2f, ", pulse_signal[s] );
        printf("]\n");
        printf("template = [ ");
        for (unsigned int s=0; s<corr_len; s++)
            printf("%.2f, ", template_signal[s] );
        printf("]\n");
#endif


        double lag = cross_correlation(pulse_signal, template_signal, corr_len);

        lag_peaks_samples[k] = lag;

        if (k == 0) {
            sample_correction = lag_peaks_samples[k];
        }

        if (k > 1) {
            delta_peaks += lag_peaks_samples[k];
            delta_peaks_len ++;
        }
    }

    // Type-II pulses
    unsigned int square_pulse_II_len = (unsigned int)round(up_factor*(212/83));
    if ( (square_pulse_len%2) != 0) square_pulse_II_len++;
    std::vector<double> square_pulse_II(square_pulse_II_len, 1.0);

    int nominal_peaks_II_samples[peaks_II_size];

    for (unsigned int k=0; k<peaks_II_size; k++) {

        int m = (int)round(0.25*new_samplingrate);

        nominal_peaks_II_samples[k] = start_packet + sample_correction + (int) round((peaks_II[k] * new_samplingrate) + 1.25*new_samplingrate);
        slice_begin = nominal_peaks_II_samples[k] - (int)round(s_chip) - m;
        slice_end = nominal_peaks_II_samples[k] + (int)round(s_chip) + m;

        unsigned int corr_len = (unsigned int) (slice_end - slice_begin);
        double pulse_signal[corr_len];
        double template_signal[corr_len];

        for (int s = slice_begin; s < slice_end; s++) {
            pulse_signal[s - slice_begin] = std::abs(y[s]);
        }

        memset(template_signal, 0, sizeof(double) * corr_len);
        int pos = (int)round((corr_len / 2) - (square_pulse_II_len / 2));


        memcpy(&template_signal[pos], square_pulse_II.data(), sizeof(double) * square_pulse_II_len);

        double lag = cross_correlation(pulse_signal, template_signal, corr_len);

        delta_peaks += lag;
        delta_peaks_len ++;



#ifdef DEBUG_HPTOA
        printf("*************** k = %d ************\n", k);
        printf("Size template: %d \n", corr_len);
        printf("Size pulse: %d (%d:%d)\n", slice_end-slice_begin, slice_begin, slice_end);
        printf("Nominal peak: %d\n", nominal_peaks_II_samples[k]);
        printf("pulse = [ ");
        for (unsigned int s=0; s<corr_len; s++)
            printf("%.2f, ", pulse_signal[s] );
        printf("]\n");
        printf("template = [ ");
        for (unsigned int s=0; s<corr_len; s++)
            printf("%.2f, ", template_signal[s] );
        printf("]\n");
#endif

    }

    double dpeaks = (delta_peaks/((delta_peaks_len)*1.0));


    double toa_res = (( (init_packet_sample*up_factor +
                         lag_peaks_samples[0] + dpeaks)
                        / SAMPLING_RATE / up_factor )) *1000;

#ifdef DEBUG_HPTOA
    printf("NPeaks = [ ");
    for (unsigned int i=0; i<peaks_I_size; i++)
        printf("%d, ", nominal_peaks_samples[i] );
    printf("];\n");

    printf("NPeaksII = [ ");
    for (unsigned int i=0; i<peaks_II_size; i++)
        printf("%d, ", nominal_peaks_II_samples[i] );
    printf("];\n");

    //Upsampled signal
    printf("up_signal = [ ");
    for (unsigned int s=0; s<signal_upsampled_len ; s++)
        printf("%f%+fi , ", y[s].real(), y[s].imag());
    printf("]\n");


#endif


    return toa_res;

}

double HPTOA::get_hptoa_peak_pulse (GPU_FFT_COMPLEX* signal_upsampled, unsigned int signal_upsampled_len,
                             unsigned int signal_front_margin, unsigned int init_packet_sample,
                             double up_factor, int bits_decoded[], int bits_len) {


    std::complex<float> y[signal_upsampled_len];

    for (unsigned int i=0; i<signal_upsampled_len; i++)
    {
        y[i].real(signal_upsampled[i].re );
        y[i].imag(signal_upsampled[i].im );
    }

    double chip_p = 0.5; // microseconds

    double new_samplingrate = up_factor * SAMPLING_RATE;
    double s_chip = chip_p * new_samplingrate;

    // Bits-chips conversion
    int chips[bits_len*2];
    int pointer = 0;

    for (int k=0; k<bits_len; k=k+2) {
        if (bits_decoded[k]==0 && bits_decoded[k+1]==0) {
            // 0101
            chips[pointer]=0; chips[pointer+1]=1; chips[pointer+2]=0; chips[pointer+3]=1;
            pointer += 4;
        }
        else if (bits_decoded[k]==0 && bits_decoded[k+1]==1) {
            // Type-II pulses -> 0110. We don't add them.
            chips[pointer]=0; chips[pointer+1]=1; chips[pointer+2]=1; chips[pointer+3]=0;
            pointer += 4;
        }
        else if (bits_decoded[k]==1 && bits_decoded[k+1]==0) {
            // 1001
            chips[pointer]=1; chips[pointer+1]=0; chips[pointer+2]=0; chips[pointer+3]=1;
            pointer += 4;
        }
        else if (bits_decoded[k]==1 && bits_decoded[k+1]==1) {
            //1010
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

    int start_packet = signal_front_margin*up_factor;

    // Nominal peaks
    int nominal_peaks_samples[peaks_size];

    // Estimated peaks
    int real_peaks_samples[peaks_size];
    int sample_correction = 0;
    int slice_begin, slice_end, amp_vector_size, max_pos;
    double max_value;
    int delta_peaks=0;
    int delta_peaks_len=0;


    unsigned int square_pulse_len = int(up_factor*0.5*SAMPLING_RATE);
    if ( (square_pulse_len%2) != 0)
        square_pulse_len++;

    std::vector<double> square_pulse(square_pulse_len, 1.0);


    int margin_before = 1*up_factor;

    // We try to do almost everything in the same for
    for (unsigned int k=0; k<peaks_size; k++) {


        // Peaks-Pulse method
        // Nominal pulses
        if (k == 0) {
            nominal_peaks_samples[k] = start_packet + (int)round((peaks_I[k] * new_samplingrate) + s_chip);
            slice_begin = nominal_peaks_samples[k] - (int)round(s_chip);
            slice_end = nominal_peaks_samples[k] + (int)round(s_chip);
        }
            // Delta peak (k==1) correction is applied to all
            // nominal peaks.
        else {
            int m = round(0.10*SAMPLING_RATE*up_factor);
            nominal_peaks_samples[k] = sample_correction + (int)round((peaks_I[k] * new_samplingrate));
            slice_begin = nominal_peaks_samples[k] - (int)round(s_chip) -m;
            slice_end = nominal_peaks_samples[k] + (int)round(s_chip) + m;
        }

        amp_vector_size = slice_end - slice_begin;
        double amp_vector[amp_vector_size];

        // Compute the amplitude and get the max
        max_value = 0.0;
        max_pos = 0;

        // Compute the amplitude
        for (int l = 1; l < amp_vector_size; l++) {
            amp_vector[l] = std::abs(y[slice_begin + l]);
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
            sample_correction = real_peaks_samples[k] +1 ;

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
            // PeakPulse
            if (real_peaks_samples[k] != 1) {
                delta_peaks = delta_peaks + (real_peaks_samples[k] - nominal_peaks_samples[k]);
                delta_peaks_len ++;

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

    double dpeaks = (delta_peaks/((delta_peaks_len)*1.0));

    double toa_res = (( (init_packet_sample*up_factor +
                         real_peaks_samples[0] + dpeaks)
                        / SAMPLING_RATE / up_factor )) *1000;

#ifdef DEBUG_HPTOA


    // Nominal Peaks
    printf("NPeaks = [ ");
    for (unsigned int i=0; i<peaks_size; i++)
        printf("%d, ", nominal_peaks_samples[i]);
    printf("];\n");

    // Estimated Peaks
    printf("SPeaks = [ ");
    for (unsigned int i=0; i<peaks_size; i++)
        printf("%d, ", real_peaks_samples[i]);
    printf("];\n");

    //Upsampled signal
    printf("up_signal = [ ");
    for (unsigned int s=0; s<signal_upsampled_len ; s++)
        printf("%f%+fi , ", y[s].real(), y[s].imag());
    printf("]\n");

    printf("start_packet=%d;\n",start_packet);

    // Outlier -> 15688065 start_time_packet
    //if ( abs(6536693812 - toa_res)<5){  // gpu
    //if (abs(6536700062 - toa_res)<5) {

    //elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>( std::chrono::steady_clock::now( ) - start );
    //std::cout << "Total (nanoseconds) " << elapsed.count( ) << '\n';

#endif

    return toa_res;
}


int HPTOA::cross_correlation (double* signal1, double* signal2, unsigned int length) {

    long double sum;

    std::vector<long double> corr_vector;

    for (unsigned int p1=0; p1<length; p1++) {
        sum=0.0;
        for (unsigned int p2=0; p2<=p1; p2++)
            sum += signal1[p2] * signal2[ (length-p1-1) + p2];
        sum=sum*sum;
        corr_vector.push_back(sum);
    }

    for (unsigned int p1=(length-2); p1>=1; p1--){
        sum=0.0;
        for (unsigned int p2=0; p2<=p1; p2++)
            sum += signal1[(length-p1-1) + p2] * signal2[p2];
        sum=sum*sum;
        corr_vector.push_back(sum);

    }

#ifdef DEBUG_HPTOA

    printf("Size vector: %lu\n", corr_vector.size());
    std::vector<long double>::iterator it = std::max_element(corr_vector.begin(), corr_vector.end());
    printf("%.3Lf\n", *it);
    printf("%ld\n", std::distance(corr_vector.begin(),it));

    printf("res_corr = [ ");
    for (unsigned int s=0; s<corr_vector.size(); s++)
        printf("%.3Lf ,", corr_vector[s]);
    printf("];\n");
#endif

    return std::distance(corr_vector.begin(), std::max_element(corr_vector.begin(), corr_vector.end())) + 1 - length ;


}
