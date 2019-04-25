#include "Haptic.h"

HapticStereo::HapticStereo(DRV2605* the_drv1, DRV2605* the_drv2, int pwm1Pin, int pwm2Pin) {
	threshold = 0.2;
	drv[0] = the_drv1; drv[1] = the_drv2;
	ledc_timer_config_t ledc_timer;
	ledc_timer.duty_resolution = LEDC_TIMER_12_BIT; // resolution of PWM duty
	ledc_timer.freq_hz = 15000;                     // frequency of PWM signal
	ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;           // timer mode
	ledc_timer.timer_num = LEDC_TIMER_0;         // timer index
	// Set configuration of timer0 for high speed channels

	ledc_timer_config(&ledc_timer);
	// ledc_channel_config_t pwm1 = {};
	// 	{pwm1Pin, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0,LEDC_INTR_DISABLE, LEDC_TIMER_0, 0},
	// 	{pwm2Pin, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1,LEDC_INTR_DISABLE, LEDC_TIMER_0, 0}
	// };
	pwm1.channel    = LEDC_CHANNEL_0;
	pwm1.duty       = 0;
	pwm1.gpio_num   = pwm1Pin;
	pwm1.speed_mode = LEDC_HIGH_SPEED_MODE;
	pwm1.intr_type = LEDC_INTR_DISABLE;
	pwm1.timer_sel  = LEDC_TIMER_0;

	pwm2.channel    = LEDC_CHANNEL_1;
	pwm2.duty       = 0;
	pwm2.gpio_num   = pwm2Pin;
	pwm2.speed_mode = LEDC_HIGH_SPEED_MODE;
	pwm2.intr_type = LEDC_INTR_DISABLE;
	pwm2.timer_sel  = LEDC_TIMER_0;

	ledc_channel_config(&pwm1);
	ledc_channel_config(&pwm2);
	pwm[0] = &pwm1;
	pwm[1] = &pwm2;

};

DRV2605 *drv[2];
adsr::Env lfo[2] = {adsr::Env(0.8, 1, 1, 0.8), adsr::Env(0.8, 1, 1, 0.8)};
adsr::Env env    = adsr::Env(1, 0.2, 0.3, 0.3);
float trgLast;
float trgTime = 500; // ms
ledc_channel_config_t pwm1;
ledc_channel_config_t pwm2;
ledc_channel_config_t *pwm[2];


float threshold;
int HapticStereo::update(float control1, float control2, float control3, int mode) {
	static int trg;
	if (!mode) {
		// Handle impulsive energy - direct mapping
		if (control1 > 0.15 /*&& control3 > 0.04*/) {
			// Serial.println(control);
			float tableVal = env.readTable((control1- 0.15) / 2.0f, true);
			tableVal = max(min(tableVal * 4096.0f, 4096.0f), 4096.0f * 0.6f);
			// Serial.println(tableVal);
			setDuty(0, tableVal);
			setDuty(1, tableVal);

			trg = 1; // return true for trigger
			trgLast = millis();
		} /*else if (millis() - trgLast > trgTime) { // Enter steady state
			trg = 0;

			if (abs(control2) > 8) { // Handle LFO
				Serial.println("rainstick");
				float sqrNormalized = control2 / 90.0f * control2 / 90.0f;
				float mag = abs(control2);
				// float duty = updateLFO(1, mag, 0.5f, max((float)copysign(1.0f, control2) * sqrNormalized / 2.0f + 0.7f, 0.5f));
				float duty = max((float)copysign(1.0f, control2) * sqrNormalized / 2.0f + 0.7f, 0.3f);
				setDuty(1, duty * 4096);
				// duty = updateLFO(0, mag, 0.0f, max((float)copysign(1.0f, control2) * (sqrNormalized) / (-2.0f) + 0.7f, 0.5f));
				duty = max((float)copysign(1.0f, control2) * (sqrNormalized) / (-2.0f) + 0.7f, 0.3f);
				setDuty(0, duty * 4096);
			}
		} 	*/ else {
			setDuty(0, 0); setDuty(1, 0); // turn off both motors
		}
	} else {
		// takes normalized control input
		// lfo[0].setDur(1, control3,control3,1);
		// lfo[1].setDur(1, 1-control3,1-control3,1);

		if (abs(control2) > 5) { // Handle LFO
			// Serial.println("rainstick");
			float sqrNormalized = control2 / 90.0f * control2 / 90.0f;
			float mag = abs(control2) * 3.0f;
			float duty = updateLFO(1, mag, 0.5f, max((float)copysign(1.0f, control2) * sqrNormalized / 2.0f + 0.7f, 0.5f));

			setDuty(1, duty * 4096);
			duty = updateLFO(0, mag, 0.0f, max((float)copysign(1.0f, control2) * (sqrNormalized) / (-2.0f) + 0.7f, 0.5f));

			setDuty(0, duty * 4096);
		} else {
			setDuty(0, 0); setDuty(1, 0); // turn off both motors
		}
	}

	return trg;

};
float HapticStereo::updateLFO(int idx, float rate, float offset, float gain) {
	lfo[idx].setGains(gain, gain);
	lfo[idx].setOffset(offset);
	return lfo[idx].update(rate);
};

void HapticStereo::setDuty(int idx, int duty) {
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwm[idx]->channel, duty);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwm[idx]->channel);
};

void HapticStereo::play(int idx, int effect) {
	drv[idx]->setWaveform(0, effect);
	drv[idx]->setWaveform(1, 0);
	drv[idx]->go();
};