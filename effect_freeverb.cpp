/*
 * Copyright (c) 2017 Yasmeen Sultana
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */



#include "effect_freeverb.h"
#include "utility/dspinst.h"
#include "math_helper.h"

void
AudioEffectFreeverb::process_apf(struct allpass_filter *apf, int32_t *in_buf, int32_t *out_buf)
{
	int32_t bufout;
	int32_t input;
	int32_t z1,w;
	int32_t n;
	int32_t output;
	/*
	bufout = buffer[bufidx];
	buffer[bufidx] = input + (bufout * feedback);
	output = -buffer[bufidx] * feedback + bufout;
	*/
	for (n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
	{
		bufout = apf->pbuffer[apf->bufferIndex];
		input = in_buf[n];
		z1 = multiply_32x32_rshift32_rounded( bufout, apf->gain);
		input += (z1 << 2);
		w= multiply_32x32_rshift32_rounded(-input, apf->gain);
		output =bufout +(w << 2);
		out_buf[n] = output;
		apf->pbuffer[apf->bufferIndex] = input;
		apf->bufferIndex++;
		if (apf->bufferIndex >= apf->delay)
		{
			apf->bufferIndex = 0;
		}
	}
}


void
AudioEffectFreeverb::process_lbcf(struct lowpass_comb_filter *lbcf, int32_t *in_buf, int32_t *out_buf)
{
	int32_t bufout;
	int32_t input;
	int32_t n;
	int32_t sum=0;
	int32_t sum2 = 0;
	/*
	output = buffer[bufidx];
	filterstore = (output * damp2) + (filterstore * damp1);
	buffer[bufidx] = input + (filterstore * feedback);
	*/
	for (n = 0; n < AUDIO_BLOCK_SAMPLES; n++)
	{
		bufout = lbcf->pbuffer[lbcf->bufferIndex];
		input = in_buf[n];
		
		sum = multiply_32x32_rshift32_rounded( bufout, lbcf->damp2);
		sum2= multiply_32x32_rshift32_rounded(lbcf->state_z1, lbcf->damp1);
		lbcf->state_z1 = ((sum + sum2) << 2);
		sum = multiply_32x32_rshift32_rounded(lbcf->state_z1, lbcf->feedback);
		sum2 = input + (sum << 2);
		lbcf->pbuffer[lbcf->bufferIndex] = sum2;
		out_buf[n] = bufout;
		lbcf->bufferIndex++;
		if (lbcf->bufferIndex >= lbcf->delay)
		{
			lbcf->bufferIndex = 0;
		}
	}
}
void AudioEffectFreeverb::init_apf(float gain,int32_t offset)
{
	uint8_t n = 0;

	for (n = 0; n < APF_COUNT; n++)
	{
		apf[n].gain = (int32_t)(gain*1073741824.0); /*2.30 format*/
		apf[n].bufferIndex = 0;
	}

	memset(&apf1_buf[0], 0, sizeof(apf1_buf));
	memset(&apf2_buf[0], 0, sizeof(apf2_buf));
	memset(&apf3_buf[0], 0, sizeof(apf3_buf));
	memset(&apf4_buf[0], 0, sizeof(apf4_buf));

	apf[0].pbuffer  = &apf1_buf[0];
	apf[0].delay   = APF1_DELAY_LENGTH + offset;
	apf[0].buf_len = APF1_DELAY_LENGTH + offset;

	apf[1].pbuffer	= &apf2_buf[0];
	apf[1].delay	= APF2_DELAY_LENGTH + offset;
	apf[1].buf_len	= APF2_DELAY_LENGTH + offset;

	apf[2].pbuffer	= &apf3_buf[0];
	apf[2].delay	= APF3_DELAY_LENGTH + offset;
	apf[2].buf_len	= APF3_DELAY_LENGTH + offset;

	apf[3].pbuffer  = &apf4_buf[0];
	apf[3].delay   = APF4_DELAY_LENGTH + offset;
	apf[3].buf_len = APF4_DELAY_LENGTH + offset;



}

void AudioEffectFreeverb::init_lbcf(float feedback, float damp,int32_t offset)
{
	uint8_t n = 0;

	for (n = 0; n < LBCF_COUNT; n++)
	{
		lbcf[n].feedback	= (int32_t)(feedback*1073741824.0); /*2.30 format*/
		lbcf[n].damp1		= (int32_t)(damp*1073741824.0);		 /*2.30 format*/
		lbcf[n].damp2		= (int32_t)((1-damp)*1073741824.0);	 /*2.30 format*/
	}

	memset(&lbcf1_buf[0], 0, sizeof(lbcf1_buf));
	memset(&lbcf2_buf[0], 0, sizeof(lbcf2_buf));
	memset(&lbcf3_buf[0], 0, sizeof(lbcf3_buf));
	memset(&lbcf4_buf[0], 0, sizeof(lbcf4_buf));
	memset(&lbcf5_buf[0], 0, sizeof(lbcf5_buf));
	memset(&lbcf6_buf[0], 0, sizeof(lbcf6_buf));
	memset(&lbcf7_buf[0], 0, sizeof(lbcf7_buf));
	memset(&lbcf8_buf[0], 0, sizeof(lbcf8_buf));

	lbcf[0].pbuffer = &lbcf1_buf[0];
	lbcf[0].delay	= LBCF1_DELAY_LENGTH+ offset;
	lbcf[0].buf_len = LBCF1_DELAY_LENGTH+ offset;

	lbcf[1].pbuffer = &lbcf2_buf[0];
	lbcf[1].delay	= LBCF2_DELAY_LENGTH + offset;
	lbcf[1].buf_len = LBCF2_DELAY_LENGTH + offset;

	lbcf[2].pbuffer = &lbcf3_buf[0];
	lbcf[2].delay	= LBCF3_DELAY_LENGTH + offset;
	lbcf[2].buf_len = LBCF3_DELAY_LENGTH + offset;

	lbcf[3].pbuffer = &lbcf4_buf[0];
	lbcf[3].delay	= LBCF4_DELAY_LENGTH + offset;
	lbcf[3].buf_len = LBCF4_DELAY_LENGTH + offset;

	lbcf[4].pbuffer = &lbcf5_buf[0];
	lbcf[4].delay	= LBCF5_DELAY_LENGTH + offset;
	lbcf[4].buf_len = LBCF5_DELAY_LENGTH + offset;

	lbcf[5].pbuffer = &lbcf6_buf[0];
	lbcf[5].delay	= LBCF6_DELAY_LENGTH + offset;
	lbcf[5].buf_len = LBCF6_DELAY_LENGTH + offset;

	lbcf[6].pbuffer = &lbcf7_buf[0];
	lbcf[6].delay	= LBCF7_DELAY_LENGTH + offset;
	lbcf[6].buf_len = LBCF7_DELAY_LENGTH + offset;

	lbcf[7].pbuffer = &lbcf8_buf[0];
	lbcf[7].delay	= LBCF8_DELAY_LENGTH + offset;
	lbcf[7].buf_len = LBCF8_DELAY_LENGTH + offset;

}
void
AudioEffectFreeverb::update(void)
{
  
  audio_block_t *block;
  
 
  if (!(block = receiveWritable()))
    return;

  if (!block->data)
    return;

  arm_q15_to_q31(block->data, q31_buf, AUDIO_BLOCK_SAMPLES);
  arm_shift_q31(q31_buf, -3, q31_buf, AUDIO_BLOCK_SAMPLES);//scaling with 0.125 

  /*low pass combo feedback filters*/
  process_lbcf(&lbcf[0], q31_buf, sum_buf);
  process_lbcf(&lbcf[1], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);

  process_lbcf(&lbcf[2], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);

  process_lbcf(&lbcf[3], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);
  
  process_lbcf(&lbcf[4], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);

  process_lbcf(&lbcf[5], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);
  
  process_lbcf(&lbcf[6], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);

  process_lbcf(&lbcf[7], q31_buf, temp_buf);
  arm_add_q31(sum_buf, temp_buf, sum_buf, AUDIO_BLOCK_SAMPLES);
  
  /*All pass filters*/
  process_apf(&apf[0], sum_buf, q31_buf);
  process_apf(&apf[1], q31_buf, q31_buf);
  process_apf(&apf[2], q31_buf, q31_buf);
  process_apf(&apf[3], q31_buf, q31_buf);
  arm_q31_to_q15(q31_buf, block->data, AUDIO_BLOCK_SAMPLES);
   
  transmit(block, 0);
  
  release(block);
}

