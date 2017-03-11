// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_airspy.h: Airspy dongle support (header)
//
// Copyright (c) 2016-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
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

#ifndef SDR_AIRSPY_H
#define SDR_AIRSPY_H

#include "demod_12m.h"
#include "demod_20m.h"

#define HILBERT_SIZE 47

#define QUEUESIZE (HILBERT_SIZE * 128)

static const float hilbert[HILBERT_SIZE] =
{
	-0.000998606272947510,
	 0.000000000000000000,
	 0.001695637278417295,
	 0.000000000000000000,
	-0.003054430179754289,
	 0.000000000000000000,
	 0.005055504379767936,
	 0.000000000000000000,
	-0.007901319195893647,
	 0.000000000000000000,
	 0.011873357051047719,
	 0.000000000000000000,
	-0.017411159379930066,
	 0.000000000000000000,
	 0.025304817427568772,
	 0.000000000000000000,
	-0.037225225204559217,
	 0.000000000000000000,
	 0.057533286997004301,
	 0.000000000000000000,
	-0.102327462004259350,
	 0.000000000000000000,
	 0.317034472508947400,
	 0.500000000000000000,
	 0.317034472508947400,
	 0.000000000000000000,
	-0.102327462004259350,
	 0.000000000000000000,
	 0.057533286997004301,
	 0.000000000000000000,
	-0.037225225204559217,
	 0.000000000000000000,
	 0.025304817427568772,
	 0.000000000000000000,
	-0.017411159379930066,
	 0.000000000000000000,
	 0.011873357051047719,
	 0.000000000000000000,
	-0.007901319195893647,
	 0.000000000000000000,
	 0.005055504379767936,
	 0.000000000000000000,
	-0.003054430179754289,
	 0.000000000000000000,
	 0.001695637278417295,
	 0.000000000000000000,
	-0.000998606272947510
};

typedef enum { MINI, R2, BADMODEL } airspy_model;

void airspyInitConfig();
void airspyShowHelp();
bool airspyOpen();
void airspyRun();
void airspyDemod(struct mag_buf *buf, int acFlag);
void airspyClose();
bool airspyHandleOption(int argc, char **argv, int *jptr);

#endif
