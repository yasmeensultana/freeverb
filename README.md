freeverb for teensy audio library

https://ccrma.stanford.edu/~jos/pasp/Freeverb.html

#include <Audio.h>
#include"effect_freeverb.h"

AudioMixer4              input_gain;
AudioMixer4              left_mix;
AudioMixer4              right_mix;
/*default values 0.5f,0.84f,0.2*/
AudioEffectFreeverb       reverbL(0.5f, 0.84f, 0.2f, false);/*allpass gain, feedback,damping, stereo pad*/
AudioEffectFreeverb       reverbR(0.5f, 0.84f, 0.2f, true);/* true adding stereo padding for right channel*/

AudioInputI2S    i2s_in;
AudioOutputI2S   i2s_out;

AudioConnection          patchCord1(i2s_in, 0, input_gain, 0);
AudioConnection          patchCord2(i2s_in, 1, input_gain, 1);
/*give the combined signal to freeverb class*/
AudioConnection          patchCord3(input_gain, reverbL);
AudioConnection          patchCord4(input_gain, reverbR);

/*Mix the left channel*/
AudioConnection          patchCord5(reverbL, 0, left_mix, 0);
AudioConnection          patchCord6(reverbR, 0, left_mix, 1);
AudioConnection          patchCord7(i2s_in, 0, left_mix, 2);

/*Mix the right channel*/
AudioConnection          patchCord8(reverbR, 0, right_mix, 0);
AudioConnection          patchCord9(reverbL, 0, right_mix, 1);
AudioConnection          patchCord10(i2s_in, 1, right_mix, 2);

/*route the final output to i2s out*/
AudioConnection          patchCord11(left_mix, 0, i2s_out, 0);
AudioConnection          patchCord12(right_mix, 0, i2s_out, 1);

void setup() {
	float wet, wet1, wet2, dry, dry1;

	const float scaleWet = 3;
	const float scaleDry = 2;
	float effectMix = 0.5;// only wet
	float width = 0.5;// complete seperation (1 no effect from the other channel)



	wet1 = scaleWet * effectMix;
	dry1 = scaleDry * (1.0 - effectMix);

	wet = wet1 / (wet1 + dry1);
	dry = dry1 / (wet1 + dry1);

	wet1 = wet * (width / 2.0 + 0.5);
	wet2 = wet * (1.0 - width) / 2.0;

	input_gain.gain(0, 0.5);
	input_gain.gain(1, 0.5);


	/*
	wet1 =1.0;
	wet2 =0;
	dry=1.0;
	*/
	/*left channel matrix*/
	left_mix.gain(0, wet1);
	left_mix.gain(1, wet2);
	left_mix.gain(2, dry);

	/*rigt channel matrix gain*/
	right_mix.gain(0, wet1);
	right_mix.gain(1, wet2);
	right_mix.gain(2, dry);
}

void loop() {
}

