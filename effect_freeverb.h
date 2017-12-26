/* Freeverb for teensy
*
* Copyright (c) 2017, Yasmeen Sultana
*
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice, development funding notice, and this permission
* notice shall be included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#ifndef effect_freeverb_
#define effect_freeverb_

#include "AudioStream.h"

/*All pass filters*/
#define APF_COUNT					    4

#define APF1_DELAY_LENGTH				225
#define APF2_DELAY_LENGTH				556
#define APF3_DELAY_LENGTH				441
#define APF4_DELAY_LENGTH				341

/*low  pass  feedback combo filters*/

#define LBCF_COUNT		8

#define LBCF1_DELAY_LENGTH				1557
#define LBCF2_DELAY_LENGTH				1617
#define LBCF3_DELAY_LENGTH				1491
#define LBCF4_DELAY_LENGTH				1422
#define LBCF5_DELAY_LENGTH				1277
#define LBCF6_DELAY_LENGTH				1356
#define LBCF7_DELAY_LENGTH				1188
#define LBCF8_DELAY_LENGTH				1116

#define RIGHT_STERO_OFFSET				23


class AudioEffectFreeverb : public AudioStream
{
public:
	AudioEffectFreeverb(void) : AudioStream(1, inputQueueArray)
	{
		/*constructor*/
		init_apf(0.5f,0);
		init_lbcf(0.84f,0.2f,0);
	
	}
	AudioEffectFreeverb(bool rightChannel) : AudioStream(1, inputQueueArray)
	{
		if (rightChannel==true)
		{
			init_apf(0.5f, RIGHT_STERO_OFFSET);
			init_lbcf(0.84f, 0.2f, RIGHT_STERO_OFFSET);
		}
		else
		{
			init_apf(0.5f, 0);
			init_lbcf(0.84f, 0.2f, 0);
		}
	}
  AudioEffectFreeverb(float apf_gain, float feedback,float damp,bool rightChannel) : AudioStream(1, inputQueueArray)
	{
		if (rightChannel==true)
		{
			init_apf(apf_gain, RIGHT_STERO_OFFSET);
			init_lbcf(feedback, damp, RIGHT_STERO_OFFSET);
		}
		else
		{
			init_apf(apf_gain, 0);
			init_lbcf(feedback, damp, 0);
		}
	}
	virtual void update(void);
	
private:
	struct allpass_filter 
	{
		int32_t   gain; 
		int32_t   *pbuffer;
		uint32_t  buf_len;
		uint32_t  delay;
		uint32_t  bufferIndex;
	};
	struct lowpass_comb_filter
	{
		int32_t   damp1; /*d*/
		int32_t   damp2; /*1-d*/
		int32_t   feedback; /*f*/
		int32_t   state_z1; /**/
		int32_t   *pbuffer;
		uint32_t  buf_len;
		uint32_t  delay;
		uint32_t   bufferIndex;
	};

	audio_block_t *inputQueueArray[1];
	
	struct allpass_filter		apf[APF_COUNT];
	struct lowpass_comb_filter	lbcf[LBCF_COUNT];
    void init_apf(float gain,int32_t offset);
	static void process_apf(struct allpass_filter *apf, int32_t *in_buf, int32_t *out_buf);
    void init_lbcf(float feedback, float damp,int32_t offset);
	static void process_lbcf(struct lowpass_comb_filter *lbcf, int32_t *in_buf, int32_t *out_buf);

	int32_t apf1_buf[APF1_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t apf2_buf[APF2_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t apf3_buf[APF3_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t apf4_buf[APF4_DELAY_LENGTH+ RIGHT_STERO_OFFSET];

	int32_t lbcf1_buf[LBCF1_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf2_buf[LBCF2_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf3_buf[LBCF3_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf4_buf[LBCF4_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf5_buf[LBCF5_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf6_buf[LBCF6_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf7_buf[LBCF7_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
	int32_t lbcf8_buf[LBCF8_DELAY_LENGTH+ RIGHT_STERO_OFFSET];
  
  int32_t q31_buf[AUDIO_BLOCK_SAMPLES];
  int32_t sum_buf[AUDIO_BLOCK_SAMPLES];
  int32_t temp_buf[AUDIO_BLOCK_SAMPLES];

};


#endif
