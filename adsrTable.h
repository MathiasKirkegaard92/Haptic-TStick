#ifndef __ADSRTABLE_H__
#define __ADSRTABLE_H__

#pragma once

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

namespace adsr {
extern const int tableLength;
extern const float adsrTable[];
extern const float minScale;
class Env {
public:
	Env(float attack, float decay, float sustain, float release);

	void  setOffset(float the_offset);
	float wrap(float the_tableIdx);
	float clip(float the_tableIdx);
	float clipSection(float the_tableIdx, int section);
	void  setGains(float the_gain, float the_sustainGain);
	void  setDur(float attack, float decay, float sustain, float release);
	float update(float rate);
	float readTable(float phase, boolean normalized);
	float phase2Idx(float the_phase);

	float offset;
	float tableIdx;

	//  Normalized params
	float dur[4]; // duration scalars. Input should in range [0;1]
	float gain;
	float sustainGain;
};
}
#endif // __ADSRTABLE_H__