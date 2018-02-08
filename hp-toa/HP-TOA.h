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

#ifndef HP_TOA_H
#define HP_TOA_H

#include <complex.h>
#include <math.h>

#include <liquid/liquid.h>

#define  UPSAMPLING_FACTOR  20.0


double get_hp_toa (float complex signal[], unsigned int signal_len, int signal_front_margin,
                  unsigned int init_packet_sample,
                  double up_factor,
                  int bits_decoded[], unsigned int bits_len);


#endif
