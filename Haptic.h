#ifndef __HAPTIC_H__
#define __HAPTIC_H__

#pragma once

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "DRV2605.h"
#include "adsrTable.h"
#include "driver/ledc.h"
#include <math.h>


class HapticStereo {
public:
	HapticStereo(DRV2605* the_drv1, DRV2605* the_drv2, int pwm1Pin, int pwm2Pin);
	DRV2605* drv[2];
	ledc_channel_config_t* pwm[2];
	ledc_channel_config_t pwm1;
	ledc_channel_config_t pwm2;
	float threshold;

	int update(float control1, float control2, float control3, int mode);
	float updateLFO(int idx, float rate, float offset, float gain);
	void setDuty(int idx, int duty);
	void play(int idx, int effect);
};


#endif