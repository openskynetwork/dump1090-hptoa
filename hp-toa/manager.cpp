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

#include "manager.h"
#include "../mode_s.h"


void init_hp_queue ()
{
    //Initial size
    mFFTQueue1 = new ReaderWriterQueue<queueMessage*> (512);
    mFFTQueue2 = new ReaderWriterQueue<queueMessage*> (512);

    // upsampling thread
    mUpsamplingThread1 = new std::thread(&thread,mFFTQueue1);
    mUpsamplingThread2 = new std::thread(&thread,mFFTQueue2);

    // high precision thread
    mHPtimestamp = new HPTOA();
    mHPtimestamp->init_queue();

}

void end_hp_queue () {

    // Wait until queue size is 0
    while( mFFTQueue1->size_approx() != 0)
        ;;

    while( mFFTQueue2->size_approx() != 0)
        ;;

    m_mutex.lock();
    mRunning = false;
    m_mutex.unlock();


    mUpsamplingThread2->join();
    mUpsamplingThread1->join();

    release_upsampling();


    mHPtimestamp->end_queue();
    delete(mHPtimestamp);



    delete(mFFTQueue1);
    delete(mFFTQueue2);

    delete(mUpsamplingThread1);
    delete(mUpsamplingThread2);


}

void push_message (struct queueMessage* message){

    // We select the queue with less messages enqueued. In case of same size
    // we follow the round-robin strategy.


    int size1 = mFFTQueue1->size_approx();
    int size2 = mFFTQueue2->size_approx();

    if (size1 < size2)
        mBalance = true;    //queue1
    else if (size2 < size1)
        mBalance = false;   //queue2


    if (mBalance) {
        message->modes_message->queue = 1;
        mFFTQueue1->enqueue(message);
    }
    else {
        message->modes_message->queue = 2;
        mFFTQueue2->enqueue(message);
    }

    mBalance = !mBalance;
}

static inline  __attribute__((always_inline)) unsigned getbit(unsigned char *data, unsigned bitnum) {
    unsigned bi = bitnum - 1;
    unsigned by = bi >> 3;
    unsigned mask = 1 << (7 - (bi & 7));

    return (data[by] & mask) != 0;
}

// Thread performs upsampling on the original signal
void thread (ReaderWriterQueue<queueMessage*> *queue) {

    queueMessage* msg;
    while(1) {

        m_mutex.lock();
        if (!mRunning) {
            m_mutex.unlock();
            break;
        }
        m_mutex.unlock();

        if (queue->try_dequeue(msg)) {

            if (msg->enable_hptoa != high_precision_t::NONE) {

                // Get bits decoded
                //int bits[msg->modes_message->msgbits];
                msg->bits_decoded = (int*) malloc(sizeof(int)*msg->modes_message->msgbits);

                for (int i = 0; i < msg->modes_message->msgbits; i++)
                    msg->bits_decoded[i] = getbit(msg->modes_message->msg, i + 1);

                int packet_complex_len = (msg->samples_front_margin + (msg->samples_len) / 2);
                std::complex<float> packet_complex[packet_complex_len];

                memset(&packet_complex,0,sizeof(std::complex<float>)*packet_complex_len);

                int index=0;

                for (unsigned int p = 0; p < msg->samples_len; p = p + 2) {
                    index = msg->samples_front_margin+(p/2);
                    packet_complex[index] = std::complex<float>( (float)((msg->samples[p] - 127.4) / 128.0),
                                                   (float)((msg->samples[p+1] - 127.4) / 128.0));
                }

                struct hptoaMessage* msgQueue = (struct hptoaMessage*) malloc(sizeof(struct hptoaMessage));
                msgQueue->qMessage = msg;
                msgQueue->qMessage->up_factor=UPSAMPLING_FACTOR;


                msgQueue->signal_upsampled =  upsampling(packet_complex, packet_complex_len, UPSAMPLING_FACTOR);
                msgQueue->signal_upsampled_len = packet_complex_len*(int)UPSAMPLING_FACTOR;
                mHPtimestamp->push_message(msgQueue);

            }
            else {

                useModesMessage(msg->modes_message);
                free(msg->samples);
                free(msg->modes_message);
                free(msg);

            }

        } else {
            usleep(0.1);
        }
    }


}


