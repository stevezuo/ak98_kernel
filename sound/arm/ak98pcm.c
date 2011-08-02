/*
 *  ak98pcm soundcard
 *  Copyright (c) by Anyka, Inc.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/moduleparam.h>
#include <linux/completion.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/i2c/aw9523.h>
#include <asm/bitops.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/info.h>
#include <sound/initval.h>

#include <mach/l2.h>

#include <mach/ak98_hal.h>


MODULE_AUTHOR("Anyka, Inc.");
MODULE_DESCRIPTION("ak98pcm soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,ak98pcm soundcard}}");

#define MIXER_ADDR_HPVOL	            0
#define MIXER_ADDR_LINEINVOL		    1
#define MIXER_ADDR_MICVOL		        2
#define MIXER_ADDR_LASTVOL              2

#define MIXER_ADDR_HPSRC                0
#define MIXER_ADDR_LINEOUTSRC           1
#define MIXER_ADDR_ADC23SRC             2
#define MIXER_ADDR_LASTSRC		        2

#define MIXER_ADDR_HPDET                0
#define MIXER_ADDR_SWITCH_LASTDET       0

#define DEFAULT_HPVOL                   5
#define DEFAULT_LINEINVOL               10
#define DEFAULT_MICVOL                  6

#define MIXER_ADDR_OUTMODE           0
#define MIXER_ADDR_OUTMODE_LAST      0

#define OUTMODE_AUTO                 0
#define OUTMODE_HP                   1
#define OUTMODE_LINEOUT              2
#define OUTMODE_MIN                  OUTMODE_AUTO
#define OUTMODE_MAX                  OUTMODE_LINEOUT

#define MIXER_ADDR_CHNLDURATION      0
#define MIXER_ADDR_CHNLDURATION_LAST 0

#define CHNLDURATION_CONSTANT        0
#define CHNLDURATION_EVEROPEN        1
#define CHNLDURATION_MIN             CHNLDURATION_CONSTANT
#define CHNLDURATION_MAX             CHNLDURATION_EVEROPEN

struct snd_ak98pcm {
	struct snd_card *card;
	struct snd_pcm *pcm;
	spinlock_t mixer_lock;
	struct snd_pcm_substream *playbacksubstrm;
	struct snd_pcm_substream *capturesubstrm;
	struct completion playbackHWDMA_completion;
	struct completion captureHWDMA_completion;
	void __iomem  *AnalogCtrlRegs;
	void __iomem  *I2SCtrlRegs;
	void __iomem  *ADC2ModeCfgRegs;
	u8   L2BufID_For_DAC;
	u8   L2BufID_For_ADC23;
	snd_pcm_uframes_t PlaybackCurrPos;
	snd_pcm_uframes_t CaptureCurrPos;
	unsigned long   playbackStrmDMARunning;//bit[0]:strm state(running or not). bit[1]:DMA state(running or finished)
	unsigned long   captureStrmDMARunning;//bit[0]:strm state(running or not). bit[1]:DMA state(running or finished)
	int mixer_volume[MIXER_ADDR_LASTVOL+1];
	int mixer_source[MIXER_ADDR_LASTSRC+1];
	int mixer_switch[MIXER_ADDR_SWITCH_LASTDET+1];
	int mixer_OutMode[MIXER_ADDR_OUTMODE_LAST+1];
	int mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION_LAST+1];
	struct snd_kcontrol *ctl_switch;
	int hp_det_irq;
	int hp_on_value;
	int irqType_for_hpOn;
	int irqType_for_hpOff;
	struct gpio_info hpdet_gpio;
	struct gpio_info spkrshdn_gpio;
	int hpmute_enable_value;
	int hpmute_disable_value;
	struct gpio_info hpmute_gpio;
	int bIsHPmuteUsed; // whether to use hardware de-pipa or not
	int bIsMetalFixed; // whether the chip DAC module has been fixed
	struct delayed_work d_work;
	struct work_struct stopoutput_work;
	struct timer_list timer;
};

extern REG_ADDR RegAddr;
struct captureSync{
	unsigned long long adcCapture_bytes;
	struct timeval tv;
};

struct captureSync capSync;

/*************
 * PCM interface
 *************/

#define USE_FORMATS 		(SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE			(SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000)
#define USE_RATE_MIN		5500
#define USE_RATE_MAX		48000
#define USE_CHANNELS_MIN 	2
#define USE_CHANNELS_MAX 	2


#define ak98pcm_playback_buf_bytes_max     (8*4096)
#define ak98pcm_playback_period_bytes_min  4096
#define ak98pcm_playback_periods_min       6
#define ak98pcm_playback_periods_max       8

#define ak98pcm_capture_buf_bytes_max      (10*4096)
#define ak98pcm_capture_period_bytes_min   2048
#define ak98pcm_capture_periods_min        1
#define ak98pcm_capture_periods_max        10

 /**
 * @brief   tell camera driver the audio capture samples for AV sync 
 * @author  
 * @date   
 * @input   void
 * @output  void *
 * @return  void
 */
void *getRecordSyncSamples(void)
{
	return &capSync;
}

/**
 * @brief  config input signal and output channel
 * @author  Cheng Mingjuan
 * @date   
 * @input  int addr: output channel
               int src: input signal
 * @return int
 */
//set source signal for both output channel(HP,speaker) and input channel(ADC23)
static int set_channel_source(struct snd_ak98pcm *ak98pcm,int addr,int src)
{
	int change = 0;
	change = (ak98pcm->mixer_source[addr]!= src);
	if(change)
	{
		if(MIXER_ADDR_HPSRC == addr) //set hp channel src
		{
			ak98_gpio_setpin(ak98pcm->hpmute_gpio.pin, ak98pcm->hpmute_enable_value);
			AK98_Poweron_HP((bool)src);
			ak98_gpio_setpin(ak98pcm->hpmute_gpio.pin, ak98pcm->hpmute_disable_value);
			AK98_Set_HP_In(src);
			AK98_Set_SrcPower(src,addr,ak98pcm->mixer_source);
		}
		else if(MIXER_ADDR_LINEOUTSRC == addr)  //set lineout channel src
		{
			if(src)
			{
				AK98_Set_SrcPower(src,addr,ak98pcm->mixer_source);
				AK98_Set_Bypass(src);
				mdelay(500);
			}
			AK98_Poweron_Speaker(ak98pcm->spkrshdn_gpio.pin,(bool)src);
			if(!src)
			{
				AK98_Set_Bypass(src);
				AK98_Set_SrcPower(src,addr,ak98pcm->mixer_source);
			}
		}
		else if(MIXER_ADDR_ADC23SRC ==addr)  //set ADC23 channel src
		{
			AK98_Set_SrcPower(src,addr,ak98pcm->mixer_source);
			AK98_Set_ADC23_In(src);
			/*set ADC23 poweron in PCM interface function: ak98pcm_capture_prepare()*/
		}
		else
		{
			printk("unsupport mixer addr!\n");
			return -EINVAL;
		}
		ak98pcm->mixer_source[addr] = src;
	}
	return change;	
}

/*
 * timer interrupt handler
 *power off HP
 */
static void ak98pcm_timer_func(unsigned long data)
{
	struct snd_ak98pcm *ak98pcm = (struct snd_ak98pcm *)data;
	int dstsrc = 0;
	printk("---------------------------ak98pcm_timer_func\n");
	dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] & (~SOURCE_DAC);
	set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
}

/**
 * @brief  when playback stop, schedule stopOutput_work to close output channel
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static void stopOutput_work(struct work_struct *work)
{
	struct snd_ak98pcm *ak98pcm =
		container_of(work, struct snd_ak98pcm, stopoutput_work);
	int dstsrc = 0;

	AK98_DAC_Close();
	//if we want to open some channel for ever, return
	if(ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION]==CHNLDURATION_EVEROPEN)
		return 0;

	//if we config output to hp, close hp channel
	if(OUTMODE_HP == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		//printk("------------------delay 30s to close hp channel\n");
		//ak98pcm->timer.expires = jiffies + 30*HZ;
		//add_timer(&ak98pcm->timer);
		printk("------------------close hp channel\n");
		dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] & (~SOURCE_DAC);
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
	}
	//if we config output to lineout, close lineout channel
	else if(OUTMODE_LINEOUT == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		printk("------------------close lineout channel\n");
		dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] & (~SOURCE_DAC);
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
	}
	//we want auto change, close hp/speaker channel
	else
	{
		if(ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION]==CHNLDURATION_CONSTANT)
		{
			if(ak98pcm->mixer_switch[MIXER_ADDR_HPDET])  //headset plug in, select headset for output
			{
				//printk("------------------delay 30s to close hp channel\n");
				//ak98pcm->timer.expires = jiffies + 30*HZ;
				//add_timer(&ak98pcm->timer);
				printk("------------------close hp channel\n");
				dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] & (~SOURCE_DAC);
				set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);	
			}
			else if(!ak98pcm->mixer_switch[MIXER_ADDR_HPDET])  //headset pull out, select speaker for output
			{
				printk("------------------close lineout channel\n");
				dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] & (~SOURCE_DAC);
				set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
			}
		}
	}
	// close all channels
	dstsrc = ak98pcm->mixer_source[MIXER_ADDR_ADC23SRC] & (~SOURCE_DAC);
	set_channel_source(ak98pcm,MIXER_ADDR_ADC23SRC,dstsrc);
}


/**
 * @brief  DMA transfer for playback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
void ak98pcm_playback_interrupt(unsigned long data)
{
	struct snd_ak98pcm *ak98pcm = (struct snd_ak98pcm *)data;
	struct snd_pcm_substream *substream = ak98pcm->playbacksubstrm;
	struct snd_pcm_runtime *runtime = substream->runtime;
	dma_addr_t paddr = runtime->dma_addr;
	u8 id = ak98pcm->L2BufID_For_DAC;
	unsigned long period_bytes = 0;
	unsigned long buffer_bytes = 0;

	period_bytes = frames_to_bytes(runtime,runtime->period_size);
	buffer_bytes = frames_to_bytes(runtime,runtime->buffer_size);
	ak98pcm->PlaybackCurrPos += period_bytes;
	if(ak98pcm->PlaybackCurrPos >= buffer_bytes)
	{
		ak98pcm->PlaybackCurrPos = 0;
	}
	snd_pcm_period_elapsed(substream);
	if(test_bit(0,&ak98pcm->playbackStrmDMARunning))//output stream is running
	{
		//printk(KERN_ERR "begin next dma");
		ak98_l2_combuf_dma(paddr+ak98pcm->PlaybackCurrPos, id, ak98pcm_playback_period_bytes_min, 
			(ak98_l2_dma_transfer_direction_t)MEM2BUF,1);
	}
	else //output strm has been stopped
	{
		printk("output stream stopped\n");
		clear_bit(1,&ak98pcm->playbackStrmDMARunning); //DMA has finished
		complete(&(ak98pcm->playbackHWDMA_completion));
		schedule_work(&ak98pcm->stopoutput_work);
	}
}

/**
 * @brief  DMA transfer for capture
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
void ak98pcm_capture_interrupt(unsigned long data)
{
	struct snd_ak98pcm *ak98pcm = (struct snd_ak98pcm *)data;
	struct snd_pcm_substream *substream = ak98pcm->capturesubstrm;
	struct snd_pcm_runtime *runtime = substream->runtime;
	dma_addr_t paddr = runtime->dma_addr;
	u8 id = ak98pcm->L2BufID_For_ADC23;
	int hpsrc=0,lineoutsrc=0,ADC23src=0;
	
	unsigned long period_bytes = 0;
	unsigned long buffer_bytes = 0;
	period_bytes = frames_to_bytes(runtime,runtime->period_size);
	buffer_bytes = frames_to_bytes(runtime,runtime->buffer_size);
	do_gettimeofday(&capSync.tv);
	capSync.adcCapture_bytes += period_bytes;
	ak98pcm->CaptureCurrPos += period_bytes;
	if(ak98pcm->CaptureCurrPos >= buffer_bytes)
	{
		ak98pcm->CaptureCurrPos = 0;
	}
	snd_pcm_period_elapsed(substream);
	if(test_bit(0,&ak98pcm->captureStrmDMARunning)) //input stream is running
	{
		//printk(KERN_ERR "begin next dma");
		ak98_l2_combuf_dma(paddr+ak98pcm->CaptureCurrPos, id, ak98pcm_capture_period_bytes_min, 
			(ak98_l2_dma_transfer_direction_t)BUF2MEM,1);
	}
	else  //input stream has been stopped
	{
		printk("input stream stopped\n");
		clear_bit(1,&ak98pcm->captureStrmDMARunning);
		complete(&(ak98pcm->captureHWDMA_completion));
		AK98_ADC23_Close();
		hpsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC];
		lineoutsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC];
		ADC23src = ak98pcm->mixer_source[MIXER_ADDR_ADC23SRC];
		if(SOURCE_MIC==ADC23src)
		{
			if((0==(hpsrc&SOURCE_MIC))&&(0==(lineoutsrc&SOURCE_MIC)))
			{
				AK98_Mic_PowerOn(0);//power off mic
			}
			AK98_Set_ADC23_In(0);  //clear adc23 src
			ak98pcm->mixer_source[MIXER_ADDR_ADC23SRC] = 0;
			if((0==hpsrc)&&((0==lineoutsrc)||(SOURCE_LINEIN==lineoutsrc)))
			{
				AK98_Poweron_VCM_REF(0);
			}
		}
	}
}

/**
 * @brief  trigger callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	{
		dma_addr_t paddr = runtime->dma_addr;
		u8 id = ak98pcm->L2BufID_For_DAC;
		del_timer(&ak98pcm->timer);
		set_bit(0,&ak98pcm->playbackStrmDMARunning);  //set bit to inform that playback stream is running
		set_bit(1,&ak98pcm->playbackStrmDMARunning);   //set bit to inform that DMA is working
		init_completion(&(ak98pcm->playbackHWDMA_completion));
		ak98_l2_clr_status(id);
		ak98_l2_combuf_dma(paddr, id, ak98pcm_playback_period_bytes_min, 
			(ak98_l2_dma_transfer_direction_t)MEM2BUF,1);//start dma
		return 0;
	}
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		clear_bit(0,&ak98pcm->playbackStrmDMARunning); //stop playback stream
		return 0;
	}
	return -EINVAL;
}

/**
 * @brief  trigger callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	{
		dma_addr_t paddr = runtime->dma_addr;
		u8 id = ak98pcm->L2BufID_For_ADC23;
		set_bit(0,&ak98pcm->captureStrmDMARunning);  //set bit to inform that capture stream is running
		set_bit(1,&ak98pcm->captureStrmDMARunning);  //set bit to inform that DMA is working
		init_completion(&(ak98pcm->captureHWDMA_completion));
		ak98_l2_clr_status(id);		
		ak98_l2_combuf_dma(paddr, id, ak98pcm_capture_period_bytes_min, 
			(ak98_l2_dma_transfer_direction_t)BUF2MEM,1); //start dma
		return 0;	
	}
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		clear_bit(0,&ak98pcm->captureStrmDMARunning); //stop capture stream
		return 0;		
	}
	return -EINVAL;
}

/**
 * @brief  prepare callback,open DAC, power on hp/speaker
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int dstsrc = 0;
	if(test_bit(1,&ak98pcm->playbackStrmDMARunning))
	{
		wait_for_completion(&(ak98pcm->playbackHWDMA_completion));
	}
	cancel_work_sync(&ak98pcm->stopoutput_work);
	ak98_l2_set_dma_callback(ak98pcm->L2BufID_For_DAC,ak98pcm_playback_interrupt,(unsigned long)ak98pcm);

	ak98pcm->PlaybackCurrPos = 0;
	AK98_DAC_Set_SampleRate(runtime->rate);
	AK98_DAC_Open();
	//if we want to open some channel for ever, return
	if(ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION]==CHNLDURATION_EVEROPEN)
	{
		return 0;
	}

	//if we config output to hp, open hp channel
	if(OUTMODE_HP == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		printk("--------------prepare hp channel\n");
		dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] | SOURCE_DAC;
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,0);
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
	}
	//if we config output to lineout, open lineout channel
	else if(OUTMODE_LINEOUT == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		printk("--------------prepare lineout channel\n");
		dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] | SOURCE_DAC;
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,0);
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
	}
	else   // open hp/speaker channel according to hp state
	{
		if(ak98pcm->mixer_switch[MIXER_ADDR_HPDET])  //hp persent, select hp for output
		{
			printk("--------------prepare hp channel\n");
			dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] | SOURCE_DAC;
			set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,0);
			set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
		}
		else if(!ak98pcm->mixer_switch[MIXER_ADDR_HPDET])//hp not persent, select speaker for output
		{
			printk("--------------prepare lineout channel\n");
			dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] | SOURCE_DAC;
			set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,0);
			set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
		}
	}

	return 0;
}

/**
 * @brief  prepare callback,open ADC23, power on mic
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int hpsrc=0,lineoutsrc=0,ADC23src=0;

	ak98_l2_set_dma_callback(ak98pcm->L2BufID_For_ADC23,ak98pcm_capture_interrupt,(unsigned long)ak98pcm);
	do_gettimeofday(&capSync.tv);
	capSync.adcCapture_bytes = 0;
	ak98pcm->CaptureCurrPos = 0;
	AK98_ADC23_Set_SampleRate(runtime->rate);
	AK98_ADC23_Set_Channels(runtime->channels);
	AK98_ADC23_Open();   //open ADC23
	hpsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC];
	lineoutsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC];
	ADC23src = ak98pcm->mixer_source[MIXER_ADDR_ADC23SRC];
	if(0==ADC23src)
	{
		AK98_Mic_PowerOn(1);  //power on mic
		AK98_Set_ADC23_In(SOURCE_MIC);
		ak98pcm->mixer_source[MIXER_ADDR_ADC23SRC] = SOURCE_MIC;
	}

	return 0;
}

/**
 * @brief  pointer callback,updata ringbuffer pointer
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static snd_pcm_uframes_t ak98pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	return(bytes_to_frames(runtime,ak98pcm->PlaybackCurrPos)); //updata ringbuffer pointer	
}

/**
 * @brief  pointer callback, updata ringbuffer pointer
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static snd_pcm_uframes_t ak98pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	return(bytes_to_frames(runtime,ak98pcm->CaptureCurrPos)); //updata ringbuffer pointer
}

static struct snd_pcm_hardware ak98pcm_playback_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		USE_FORMATS,
	.rates =		USE_RATE,
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX,
	.buffer_bytes_max =	ak98pcm_playback_buf_bytes_max,
	.period_bytes_min =	ak98pcm_playback_period_bytes_min,
	.period_bytes_max =	ak98pcm_playback_period_bytes_min,
	.periods_min =		ak98pcm_playback_periods_min,
	.periods_max =		ak98pcm_playback_periods_max,
	.fifo_size =		0,
};

static struct snd_pcm_hardware ak98pcm_capture_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_BLOCK_TRANSFER |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		USE_FORMATS,
	.rates =		USE_RATE,
	.rate_min =		USE_RATE_MIN,
	.rate_max =		USE_RATE_MAX,
	.channels_min =		USE_CHANNELS_MIN,
	.channels_max =		USE_CHANNELS_MAX,
	.buffer_bytes_max =	ak98pcm_capture_buf_bytes_max,
	.period_bytes_min =	ak98pcm_capture_period_bytes_min,
	.period_bytes_max =	ak98pcm_capture_period_bytes_min,
	.periods_min =		ak98pcm_capture_periods_min,
	.periods_max =		ak98pcm_capture_periods_max,
	.fifo_size =		0,
};

/**
 * @brief  hw_params callback, malloc ringbuffer
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_playback_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	if(BUF_NULL==ak98pcm->L2BufID_For_DAC)
	{
		ak98pcm->L2BufID_For_DAC = ak98_l2_alloc((ak98_l2_device_t)ADDR_DAC); //alloc l2 buffer for DAC	
		if(BUF_NULL==ak98pcm->L2BufID_For_DAC)
		{
			printk(KERN_ERR "alloc L2 buffer for DAC error!");
			return -ENOMEM;
		}
	}
	return(snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params)));
}

/**
 * @brief  hw_params callback, malloc ringbuffer
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_capture_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	if(BUF_NULL==ak98pcm->L2BufID_For_ADC23)
	{
		ak98pcm->L2BufID_For_ADC23 = ak98_l2_alloc((ak98_l2_device_t)ADDR_ADC); //alloc l2 buffer for ADC23	
		if(BUF_NULL==ak98pcm->L2BufID_For_ADC23)
		{
			printk(KERN_ERR "alloc L2 buffer for DAC error!");
			return -ENOMEM;
		}
	}
	return(snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params)));
}

/**
 * @brief  hw_free callback, free ringbuffer
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_playback_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	
	if(test_bit(1,&ak98pcm->playbackStrmDMARunning))
	{
		wait_for_completion(&(ak98pcm->playbackHWDMA_completion));
	}
	if(BUF_NULL!=ak98pcm->L2BufID_For_DAC)
	{
		AK98_DAC_Close();
		ak98_l2_free((ak98_l2_device_t)ADDR_DAC);
		ak98pcm->L2BufID_For_DAC = BUF_NULL;
	}
	return snd_pcm_lib_free_pages(substream);
}

/**
 * @brief  hw_free callback, free ringbuffer
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_capture_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	
	if(test_bit(1,&ak98pcm->captureStrmDMARunning))
	{
		wait_for_completion(&(ak98pcm->captureHWDMA_completion));
	}
	if(BUF_NULL!=ak98pcm->L2BufID_For_ADC23)
	{
		AK98_ADC23_Close();
		ak98_l2_free((ak98_l2_device_t)ADDR_ADC);
		ak98pcm->L2BufID_For_ADC23 = BUF_NULL;
	}
	return(snd_pcm_lib_free_pages(substream));	
}

/**
 * @brief  open callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	ak98pcm->playbacksubstrm = substream;
	runtime->hw = ak98pcm_playback_hardware;
	ak98pcm->PlaybackCurrPos = 0;

	return 0;
}

/**
 * @brief  open callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_capture_open(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	
	ak98pcm->capturesubstrm = substream;
	runtime->hw = ak98pcm_capture_hardware;	
	ak98pcm->CaptureCurrPos = 0;
	
	return 0;
}

/**
 * @brief  close callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);

	ak98pcm->playbacksubstrm=NULL;
	return 0;
}

/**
 * @brief  close callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_ak98pcm *ak98pcm = snd_pcm_substream_chip(substream);
	capSync.adcCapture_bytes = 0;
	ak98pcm->capturesubstrm=NULL;
	return 0;
}

/**
 * @brief  mmap callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int ak98pcm_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}


static struct snd_pcm_ops ak98pcm_playback_ops = {
	.open =		ak98pcm_playback_open,
	.close =	ak98pcm_playback_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	ak98pcm_playback_hw_params,
	.hw_free =	ak98pcm_playback_hw_free,
	.prepare =	ak98pcm_playback_prepare,
	.trigger =	ak98pcm_playback_trigger,
	.pointer =	ak98pcm_playback_pointer,
	.mmap =	    ak98pcm_pcm_mmap,
};

static struct snd_pcm_ops ak98pcm_capture_ops = {
	.open =		ak98pcm_capture_open,
	.close =	ak98pcm_capture_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	ak98pcm_capture_hw_params,
	.hw_free =	ak98pcm_capture_hw_free,
	.prepare =	ak98pcm_capture_prepare,
	.trigger =	ak98pcm_capture_trigger,
	.pointer =	ak98pcm_capture_pointer,
	.mmap =	    ak98pcm_pcm_mmap,
};

/**
 * @brief  create new card
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int __devinit snd_card_ak98pcm_pcm(struct snd_ak98pcm *ak98pcm, int device,
					int substreams)
{
	struct snd_pcm *pcm;
	int err;
	err = snd_pcm_new(ak98pcm->card, "ak98pcm PCM", device,
			       substreams, substreams, &pcm);
	if (err < 0)
		return err;
	ak98pcm->pcm = pcm;
	
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &ak98pcm_playback_ops); //register callbacks
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &ak98pcm_capture_ops);//register callbacks
	pcm->private_data = ak98pcm;
	pcm->info_flags = 0;
	strcpy(pcm->name, "ak98pcm PCM");

	snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_DEV,
			ak98pcm->card->dev,
			ak98pcm_playback_periods_max*ak98pcm_playback_period_bytes_min,
			ak98pcm_playback_buf_bytes_max); //malloc ringbuffer
	return 0;
}


/**************
 * mixer interface
 **************/


/**********************HPDet switch**************************/

/**
 * @brief  when hp state is changed,schedule hpDet_wq_work to config output \
               channel to speaker or not
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static void hpDet_wq_work(struct work_struct *work)
{
	struct snd_ak98pcm *ak98pcm = container_of(work, struct snd_ak98pcm,d_work.work);
	int hpsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC];
	int spksrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC];
	int dstsrc = hpsrc | spksrc;
	if(!test_bit(0,&ak98pcm->playbackStrmDMARunning))
	{
		dstsrc &= ~SOURCE_DAC;
	}
	if(ak98pcm->mixer_switch[MIXER_ADDR_HPDET])  //headset plug in, select headset for output
	{
		if(CHNLDURATION_EVEROPEN == ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION])
		{
			dstsrc |= SOURCE_DAC;
		}
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,0);
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
	}
	else  //headset pull out, select speaker for output
	{
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,0);
		if((dstsrc&SOURCE_DAC)&&(!test_bit(0,&ak98pcm->playbackStrmDMARunning)))
		{
			dstsrc &= (~SOURCE_DAC);
		}
		if(CHNLDURATION_EVEROPEN == ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION])
		{
			dstsrc |= SOURCE_DAC;
		}
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
	}
}

/**
 * @brief  hp det
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static irqreturn_t ak98pcm_HPDet_interrupt(int irq, void *dev_id)
{
	struct snd_card *card = dev_id;
	struct snd_ak98pcm *ak98pcm = card->private_data;

	if(ak98_gpio_getpin(ak98pcm->hpdet_gpio.pin) == ak98pcm->hp_on_value)
	{
		//hp is plugged in
		ak98pcm->mixer_switch[MIXER_ADDR_HPDET] = 1;
		set_irq_type(ak98pcm->hp_det_irq, ak98pcm->irqType_for_hpOff);
		printk("-------------- hp on\n");
	}
	else
	{
		//hp is pulled out
		ak98pcm->mixer_switch[MIXER_ADDR_HPDET] = 0;
		set_irq_type(ak98pcm->hp_det_irq, ak98pcm->irqType_for_hpOn);
		printk("-------------- hp off\n");
	}
	snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,&ak98pcm->ctl_switch->id);
	if(ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE] != OUTMODE_AUTO)
		return IRQ_HANDLED;
#ifdef CONFIG_SPKHP_SWITCH_AUTO
	schedule_delayed_work(&ak98pcm->d_work, msecs_to_jiffies(100));
#endif
	return IRQ_HANDLED;
}

/***********************config channel duration*******************************/
#define AK98PCM_OUTPUTCHNL_DURATION(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = snd_ak98pcm_ChnlDuration_info, \
  .get = snd_ak98pcm_ChnlDuration_get, \
  .put = snd_ak98pcm_ChnlDuration_put, \
  .private_value = addr \
}

/**
 * @brief  info callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_ChnlDuration_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	int addr = kcontrol->private_value;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	if(MIXER_ADDR_CHNLDURATION == addr)
	{
		uinfo->value.integer.min = CHNLDURATION_MIN;
		uinfo->value.integer.max = CHNLDURATION_MAX;
	}
	else
	{
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief  get callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_ChnlDuration_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	if(addr != MIXER_ADDR_CHNLDURATION)
	{
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = ak98pcm->mixer_ChnlDuration[addr];
	return 0;
}

/**
 * @brief  put callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_ChnlDuration_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	int duration = ucontrol->value.integer.value[0];
	int dstsrc = 0;
	printk("-------------snd_ak98pcm_ChnlDuration_put duration=%d\n",duration);
	if(duration == ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION])
	{
		return 0;
	}
	if(addr != MIXER_ADDR_CHNLDURATION)
	{
		return -EINVAL;
	}
	if((duration > CHNLDURATION_MAX)||(duration < CHNLDURATION_MIN))
	{
		return -EINVAL;
	}
	ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION] = duration;
	if(CHNLDURATION_CONSTANT == duration)
	{
		if(!test_bit(0,&ak98pcm->playbackStrmDMARunning))
		{
			dstsrc =ak98pcm->mixer_source[MIXER_ADDR_HPSRC] & (~SOURCE_DAC);
			set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
			dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] & (~SOURCE_DAC);
			set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
		}
		return 0;
	}
	
	if(OUTMODE_HP == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		printk("--------------prepare hp channel\n");
		dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] | SOURCE_DAC;
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,0);
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
	}
	else if(OUTMODE_LINEOUT == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		printk("--------------prepare lineout channel\n");
		dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] | SOURCE_DAC;
		set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,0);
		set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
	}
	else
	{
		if(ak98pcm->mixer_switch[MIXER_ADDR_HPDET])  //hp persent, select hp for output
		{
			printk("--------------prepare hp channel\n");
			dstsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC] | SOURCE_DAC;
			set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,0);
			set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
		}
		else if(!ak98pcm->mixer_switch[MIXER_ADDR_HPDET])//hp not persent, select speaker for output
		{
			printk("--------------prepare lineout channel\n");
			dstsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] | SOURCE_DAC;
			set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,0);
			set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
		}
	}
	return 0;
}

/*********************select fixed output channel**********************************/
#define AK98PCM_DAC_OUT_MODE(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = snd_ak98pcm_DACOutMode_info, \
  .get = snd_ak98pcm_DACOutMode_get, \
  .put = snd_ak98pcm_DACOutMode_put, \
  .private_value = addr \
}

/**
 * @brief  info callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_DACOutMode_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	int addr = kcontrol->private_value;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	if(MIXER_ADDR_OUTMODE == addr)
	{
		uinfo->value.integer.min = OUTMODE_MIN;
		uinfo->value.integer.max = OUTMODE_MAX;
	}
	else
	{
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief  get callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_DACOutMode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	if(addr != MIXER_ADDR_OUTMODE)
	{
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = ak98pcm->mixer_OutMode[addr];
	return 0;
}

/**
 * @brief  put callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_DACOutMode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	int mode = ucontrol->value.integer.value[0];
	int dstsrc = 0;
	int tempsrc = 0;
	printk("-------------snd_ak98pcm_DACOutMode_put mode=%d\n",mode);
	if(mode == ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE])
	{
		return 0;
	}
	if(addr != MIXER_ADDR_OUTMODE)
	{
		return -EINVAL;
	}
	if((mode > OUTMODE_MAX)||(mode < OUTMODE_MIN))
	{
		mode = OUTMODE_AUTO;
	}
	switch(mode)
	{
		case OUTMODE_HP:
			tempsrc = ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC];
			set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,0);
			if(tempsrc)
			{
				set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,tempsrc);
			}

			if(ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION]==CHNLDURATION_EVEROPEN)
			{
				dstsrc =ak98pcm->mixer_source[MIXER_ADDR_HPSRC]| SOURCE_DAC;
				set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,dstsrc);
			}
			break;
		case OUTMODE_LINEOUT:
			tempsrc = ak98pcm->mixer_source[MIXER_ADDR_HPSRC];
			set_channel_source(ak98pcm,MIXER_ADDR_HPSRC,0);
			if(tempsrc)
			{
				set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,tempsrc);
			}

			if(ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION]==CHNLDURATION_EVEROPEN)
			{
				dstsrc =ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC]| SOURCE_DAC;
				set_channel_source(ak98pcm,MIXER_ADDR_LINEOUTSRC,dstsrc);
			}
			break;
		case OUTMODE_AUTO:
		default:
			schedule_delayed_work(&ak98pcm->d_work, msecs_to_jiffies(100));
			break;
	}
	ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE] = mode;
	return 0;
}

/**********************hp detect (read-only)****************************/
#define AK98PCM_SWITCH(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_HWDEP, \
  .access = SNDRV_CTL_ELEM_ACCESS_READ, \
  .name = xname, .index = xindex, \
  .info = snd_ak98pcm_switch_info, \
  .get = snd_ak98pcm_switch_get, \
  .private_value = addr \
}

/**
 * @brief  info callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_switch_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	int addr = kcontrol->private_value;
	if(MIXER_ADDR_HPDET != addr)
	{
		return -EINVAL;
	}

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;		
	return 0;
}

/**
 * @brief  get callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_switch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	if(MIXER_ADDR_HPDET != addr)
	{
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = ak98pcm->mixer_switch[addr];
	return 0;
}

/***************************VOLUME*********************/
#define AK98PCM_VOLUME(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = snd_ak98pcm_volume_info, \
  .get = snd_ak98pcm_volume_get, \
  .put = snd_ak98pcm_volume_put, \
  .private_value = addr \
}

/**
 * @brief  info callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_volume_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	int addr = kcontrol->private_value;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	if(MIXER_ADDR_HPVOL == addr)
	{
		uinfo->value.integer.min = HEADPHONE_GAIN_MIN;
		uinfo->value.integer.max = HEADPHONE_GAIN_MAX;
	}
	else if(MIXER_ADDR_LINEINVOL == addr)
	{
		uinfo->value.integer.min = LINEIN_GAIN_MIN;
		uinfo->value.integer.max = LINEIN_GAIN_MAX;
	}
	else if(MIXER_ADDR_MICVOL == addr)
	{
		uinfo->value.integer.min = MIC_GAIN_MIN;
		uinfo->value.integer.max = MIC_GAIN_MAX;
	}
	else
	{
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief  get callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	if(addr > MIXER_ADDR_LASTVOL)
	{
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = ak98pcm->mixer_volume[addr];
	return 0;
}

/**
 * @brief  put callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	int vol = 0;
	int change = 0;

	vol = ucontrol->value.integer.value[0];
	if(MIXER_ADDR_HPVOL == addr)
	{
		if(vol < HEADPHONE_GAIN_MIN)
		{
			vol = HEADPHONE_GAIN_MIN;
		}
		if(vol > HEADPHONE_GAIN_MAX)
		{
			vol = HEADPHONE_GAIN_MAX;
		}

		change = (ak98pcm->mixer_volume[addr] != vol);
		if(change)
		{
			AK98_Set_HPGain(vol);
		}
		ak98pcm->mixer_volume[addr] = vol;
	}
	else if(MIXER_ADDR_LINEINVOL == addr)
	{
		if(vol < LINEIN_GAIN_MIN)
		{
			vol = LINEIN_GAIN_MIN;
		}
		if(vol > LINEIN_GAIN_MAX)
		{
			vol = LINEIN_GAIN_MAX;
		}

		change = (ak98pcm->mixer_volume[addr] != vol);
		if(change)
		{
			AK98_Set_LineinGain(vol);
		}
		ak98pcm->mixer_volume[addr] = vol;
	}
	else if(MIXER_ADDR_MICVOL == addr)
	{
		if(vol < MIC_GAIN_MIN)
		{
			vol = MIC_GAIN_MIN;
		}
		if(vol > MIC_GAIN_MAX)
		{
			vol = MIC_GAIN_MAX;
		}

		change = (ak98pcm->mixer_volume[addr] != vol);
		if(change)
		{
			AK98_Set_MicGain(vol);
		}
		ak98pcm->mixer_volume[addr] = vol;
	}
	else
	{
		return -EINVAL;
	}

	return change;
}

/***************************ROUTE**************************/
#define AK98PCM_ROUTE(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, \
  .index = xindex, \
  .info = snd_ak98pcm_route_info, \
  .get = snd_ak98pcm_route_get, \
  .put = snd_ak98pcm_route_put, \
  .private_value = addr \
}

/**
 * @brief  info callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_route_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	int addr = kcontrol->private_value;
	if(addr > MIXER_ADDR_LASTSRC)
	{
		return -EINVAL;
	}
	
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SIGNAL_SRC_MAX;
	return 0;
}

/**
 * @brief  get callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98pcm_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	if(addr > MIXER_ADDR_LASTSRC)
	{
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = ak98pcm->mixer_source[addr];
	return 0;
}

/**
 * @brief  put callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
/*application has to call this callback with params value=0 to power down input&output devices*/
static int snd_ak98pcm_route_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_ak98pcm *ak98pcm = snd_kcontrol_chip(kcontrol);
	int change, addr = kcontrol->private_value;
	int src = 0;

	src = ucontrol->value.integer.value[0];
	if(src < SIGNAL_SRC_MUTE)
	{
		src = SIGNAL_SRC_MUTE;
	}
	if(src > SIGNAL_SRC_MAX)
	{
		src = SIGNAL_SRC_MAX;
	}
	change = set_channel_source(ak98pcm,addr,src);
	return change;
}

static struct snd_kcontrol_new snd_ak98pcm_controls[] = {
AK98PCM_VOLUME("Headphone Playback Volume", 0, MIXER_ADDR_HPVOL),
AK98PCM_VOLUME("LineIn Capture Volume", 0, MIXER_ADDR_LINEINVOL),
AK98PCM_VOLUME("Mic Capture Volume", 0, MIXER_ADDR_MICVOL),
AK98PCM_ROUTE("Headphone Playback Route", 0, MIXER_ADDR_HPSRC),
AK98PCM_ROUTE("LineOut Playback Route", 0, MIXER_ADDR_LINEOUTSRC),
AK98PCM_ROUTE("ADC23 Capture Route", 0, MIXER_ADDR_ADC23SRC),
AK98PCM_SWITCH("HPDet switch", 0, MIXER_ADDR_HPDET),
AK98PCM_DAC_OUT_MODE("DAC out mode", 0, MIXER_ADDR_OUTMODE),
AK98PCM_OUTPUTCHNL_DURATION("duration of output channel", 0, MIXER_ADDR_CHNLDURATION),
};


/**
 * @brief  create new mixer interface
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int __devinit snd_card_ak98pcm_new_mixer(struct snd_ak98pcm *ak98pcm)
{
	struct snd_card *card = ak98pcm->card;
	unsigned int idx;
	struct snd_ctl_elem_id elem_id;
	int err;

	strcpy(card->mixername, "ak98pcm Mixer");

	for (idx = 0; idx < ARRAY_SIZE(snd_ak98pcm_controls); idx++) {
		err = snd_ctl_add(card, snd_ctl_new1(&snd_ak98pcm_controls[idx], ak98pcm));
		if (err < 0)
			return err;
	}

	/* attach master switch for HPDet switch control */	
	memset(&elem_id, 0, sizeof(elem_id));
	elem_id.iface = SNDRV_CTL_ELEM_IFACE_HWDEP;
	strcpy(elem_id.name, "HPDet switch");
	ak98pcm->ctl_switch = snd_ctl_find_id(card, &elem_id);

	return 0;
}

/**************
 * proc interface
 **************/
static void dac_mfix_proc_read(struct snd_info_entry *entry, struct snd_info_buffer *buffer)
{
	struct snd_ak98pcm *ak98pcm = entry->private_data;

	snd_iprintf(buffer,"%d\n",ak98pcm->bIsMetalFixed);
}

static int __devinit snd_ak98pcm_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct snd_ak98pcm *ak98pcm;
	struct resource *AnalogCtrlRegs;
	struct resource *I2SCtrlRegs;
	struct resource *ADC2ModeCfgRegs;
	struct ak98pcm_platform_data *ak98pcm_plat_data;
	int dev_id, err;

	//get analog control registers
	AnalogCtrlRegs = platform_get_resource_byname(devptr, IORESOURCE_MEM, "ak98pcm_AnalogCtrlRegs"); 
	if(!AnalogCtrlRegs)
	{
		printk(KERN_ERR "no memory resource for AnalogCtrlRegs\n");
		return -ENXIO;
	}

	//get I2S control registers
	I2SCtrlRegs = platform_get_resource_byname(devptr, IORESOURCE_MEM, "ak98pcm_I2SCtrlRegs"); 
	if(!I2SCtrlRegs)
	{
		printk(KERN_ERR "no memory resource for I2SCtrlRegs\n");
		return -ENXIO;
	}

	//get ADC23 mode registers
	ADC2ModeCfgRegs = platform_get_resource_byname(devptr, IORESOURCE_MEM, "ak98pcm_ADC2ModeCfgRegs");
	if(!ADC2ModeCfgRegs)
	{
		printk(KERN_ERR "no memory resource for ADC2ModeCfgRegs\n");
		return -ENXIO;
	}

	dev_id = devptr->id;
	if (dev_id < 0)
		dev_id = 0;
	err = snd_card_create(dev_id, NULL, THIS_MODULE, sizeof(struct snd_ak98pcm), &card);
	if (err < 0)
		return err;

	snd_card_set_dev(card, &devptr->dev);

	ak98pcm = card->private_data;
	ak98pcm->card = card;

	ak98pcm->AnalogCtrlRegs = ioremap(AnalogCtrlRegs->start, AnalogCtrlRegs->end - AnalogCtrlRegs->start + 1);
	if (!ak98pcm->AnalogCtrlRegs) {
		printk(KERN_ERR "could not remap AnalogCtrlRegs memory");
		goto __out_free_card;
	}

	ak98pcm->I2SCtrlRegs = ioremap(I2SCtrlRegs->start, I2SCtrlRegs->end - I2SCtrlRegs->start + 1);
	if (!ak98pcm->I2SCtrlRegs) {
		printk(KERN_ERR "could not remap I2SCtrlRegs memory");
		goto __out_unmap_AnalogCtrlRegs;
	}

	ak98pcm->ADC2ModeCfgRegs = ioremap(ADC2ModeCfgRegs->start, ADC2ModeCfgRegs->end - ADC2ModeCfgRegs->start + 1);
	if (!ak98pcm->ADC2ModeCfgRegs) {
		printk(KERN_ERR "could not remap ADC2ModeCfgRegs memory");
		goto __out_unmap_I2SCtrlRegs;
	}

	RegAddr.pAddress0800 = ak98pcm->AnalogCtrlRegs;
	RegAddr.pAddress2002E = ak98pcm->I2SCtrlRegs;
	RegAddr.pAddress2002D = ak98pcm->ADC2ModeCfgRegs;

	//init l2 buf for audio
	ak98pcm->L2BufID_For_DAC = BUF_NULL;
	ak98pcm->L2BufID_For_ADC23 = BUF_NULL;

	err = snd_card_ak98pcm_pcm(ak98pcm, 0, 1);
	if (err < 0)
		goto __out_unmap_ADC2ModeCfgRegs;
	err = snd_card_ak98pcm_new_mixer(ak98pcm);
	if(err < 0)
		goto __out_unmap_ADC2ModeCfgRegs;

	//init default volume
	ak98pcm->mixer_volume[MIXER_ADDR_HPVOL] = DEFAULT_HPVOL;
	AK98_Set_HPGain(DEFAULT_HPVOL);
	ak98pcm->mixer_volume[MIXER_ADDR_LINEINVOL] = DEFAULT_LINEINVOL;
	AK98_Set_LineinGain(DEFAULT_LINEINVOL);
	ak98pcm->mixer_volume[MIXER_ADDR_MICVOL] = DEFAULT_MICVOL;
	AK98_Set_MicGain(DEFAULT_MICVOL);
	ak98pcm->mixer_source[MIXER_ADDR_HPSRC] = 0;
	ak98pcm->mixer_source[MIXER_ADDR_LINEOUTSRC] = 0;
	ak98pcm->mixer_source[MIXER_ADDR_ADC23SRC] = 0;
	ak98pcm->mixer_OutMode[MIXER_ADDR_OUTMODE] = OUTMODE_AUTO;
	ak98pcm->mixer_ChnlDuration[MIXER_ADDR_CHNLDURATION] = CHNLDURATION_CONSTANT;
	
#if defined CONFIG_SPKHP_SWITCH_AUTO || defined CONFIG_SPKHP_SWITCH_MIXER
#ifdef CONFIG_SPKHP_SWITCH_AUTO
	INIT_DELAYED_WORK(&ak98pcm->d_work, hpDet_wq_work);
#endif
	//config hp det gpio
	ak98pcm_plat_data = (struct ak98pcm_platform_data *)devptr->dev.platform_data;
	ak98pcm->hpdet_gpio = ak98pcm_plat_data->hpdet_gpio;
	ak98_gpio_set(&(ak98pcm->hpdet_gpio));

	//set hp det pin irq
	ak98pcm->hp_det_irq = ak98pcm_plat_data->hpdet_irq;
	ak98pcm->hp_on_value = ak98pcm_plat_data->hp_on_value;
	if(AK98_GPIO_LOW == ak98pcm->hp_on_value)
	{
		ak98pcm->irqType_for_hpOn = IRQ_TYPE_LEVEL_LOW;
		ak98pcm->irqType_for_hpOff = IRQ_TYPE_LEVEL_HIGH;
	}
	else if(AK98_GPIO_HIGH == ak98pcm->hp_on_value)
	{
		ak98pcm->irqType_for_hpOn = IRQ_TYPE_LEVEL_HIGH;
		ak98pcm->irqType_for_hpOff = IRQ_TYPE_LEVEL_LOW;
	}
	if(ak98_gpio_getpin(ak98pcm->hpdet_gpio.pin) == ak98pcm->hp_on_value)
	{
		//hp is plugged in
		ak98pcm->mixer_switch[MIXER_ADDR_HPDET] = 1;
		set_irq_type(ak98pcm->hp_det_irq, ak98pcm->irqType_for_hpOff);
		printk("--------------probe: hp on\n");
	}
	else
	{
		//hp is pulled out
		ak98pcm->mixer_switch[MIXER_ADDR_HPDET] = 0;
		set_irq_type(ak98pcm->hp_det_irq, ak98pcm->irqType_for_hpOn);
		printk("--------------probe: hp off\n");
	}
	err = request_irq(ak98pcm->hp_det_irq, ak98pcm_HPDet_interrupt, IRQF_DISABLED, devptr->name, card);
	if(err)
	{
		printk(KERN_ERR "request irq error!");
		goto __out_unmap_ADC2ModeCfgRegs;
	}
#else
    // will not detect headset pin in audio driver
	memset(&(ak98pcm->hpdet_gpio),0,sizeof(struct gpio_info));
#endif

	//config speaker shutdown gpio
	ak98pcm->spkrshdn_gpio = ak98pcm_plat_data->spk_down_gpio;
	ak98_gpio_set(&(ak98pcm->spkrshdn_gpio));

	//config hp mute gpio
	ak98pcm->hpmute_gpio = ak98pcm_plat_data->hpmute_gpio;
	ak98_gpio_set(&(ak98pcm->hpmute_gpio));
	ak98pcm->hpmute_enable_value = ak98pcm_plat_data->hp_mute_enable_value;
	if(AK98_GPIO_HIGH == ak98pcm->hpmute_enable_value)
	{
		ak98pcm->hpmute_disable_value = AK98_GPIO_LOW;
	}
	else if(AK98_GPIO_LOW == ak98pcm->hpmute_enable_value)
	{
		ak98pcm->hpmute_disable_value = AK98_GPIO_HIGH;
	}

	ak98pcm->bIsHPmuteUsed = ak98pcm_plat_data->bIsHPmuteUsed;
	ak98pcm->bIsMetalFixed = ak98pcm_plat_data->bIsMetalfixed;

	//init workqueue to stop output channel
	INIT_WORK(&ak98pcm->stopoutput_work, stopOutput_work);

	clear_bit(0,&ak98pcm->playbackStrmDMARunning);
	clear_bit(1,&ak98pcm->playbackStrmDMARunning);
	clear_bit(0,&ak98pcm->captureStrmDMARunning);
	clear_bit(1,&ak98pcm->captureStrmDMARunning);

	//timer for delay to stop output channel when playback is stopped
	init_timer(&ak98pcm->timer);
	ak98pcm->timer.function = ak98pcm_timer_func;
	ak98pcm->timer.data = (unsigned long)ak98pcm;

	// register a proc file to tell user whether the chip DAC module has been fixed
	struct snd_info_entry *entry;
	snd_card_proc_new (ak98pcm->card, "dac-metalfix", &entry);
	snd_info_set_text_ops(entry, ak98pcm, dac_mfix_proc_read);
	
	strcpy(card->driver, "ak98pcm");
	strcpy(card->shortname, "Ak98 AD/DA");
	sprintf(card->longname, "Ak98 ADC DAC pcm input & output module %i", dev_id + 1);

	err = snd_card_register(card);
	if (err == 0) {
		platform_set_drvdata(devptr, card);
		return 0;
	}

	free_irq(ak98pcm->hp_det_irq, ak98pcm);
	__out_unmap_ADC2ModeCfgRegs:
	iounmap(ak98pcm->ADC2ModeCfgRegs);
	__out_unmap_I2SCtrlRegs:
	iounmap(ak98pcm->I2SCtrlRegs);	
	__out_unmap_AnalogCtrlRegs:
	iounmap(ak98pcm->AnalogCtrlRegs);	
	__out_free_card:
	snd_card_free(card);

	return err;
}

static int __devexit snd_ak98pcm_remove(struct platform_device *devptr)
{
	struct snd_card *card = platform_get_drvdata(devptr);
	struct snd_ak98pcm *ak98pcm = card->private_data;

#ifdef CONFIG_SPKHP_SWITCH_AUTO
	cancel_delayed_work_sync(&ak98pcm->d_work);
#endif
	del_timer(&ak98pcm->timer);
	cancel_work_sync(&ak98pcm->stopoutput_work);
	free_irq(ak98pcm->hp_det_irq, ak98pcm);
	iounmap(ak98pcm->AnalogCtrlRegs);
	iounmap(ak98pcm->I2SCtrlRegs);
	iounmap(ak98pcm->ADC2ModeCfgRegs);
	snd_card_set_dev(card, NULL);
	snd_card_free(card);
	platform_set_drvdata(devptr, NULL);
	return 0;
}

#ifdef CONFIG_PM
/**
 * @brief  suspend callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_ak98pcm *ak98pcm = card->private_data;

	if(test_bit(1,&ak98pcm->playbackStrmDMARunning))
	{
		wait_for_completion(&(ak98pcm->playbackHWDMA_completion));
	}

	if(test_bit(1,&ak98pcm->captureStrmDMARunning))
	{
		wait_for_completion(&(ak98pcm->captureHWDMA_completion));
	}

	//close analog module
	AK98_DAC_Close();
	ak98_gpio_setpin(ak98pcm->hpmute_gpio.pin, ak98pcm->hpmute_enable_value);
	AK98_Poweron_HP(0);
	ak98_gpio_setpin(ak98pcm->hpmute_gpio.pin, ak98pcm->hpmute_disable_value);
	AK98_Set_HP_In(0);
	AK98_Set_Bypass(0);
	AK98_Poweron_Speaker(ak98pcm->spkrshdn_gpio.pin,0);

	AK98_ADC23_Close();
	AK98_Set_ADC23_In(0);

	AK98_Poweron_VCM_REF(0);

	return 0;
}

/**
 * @brief  resume callback
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int snd_ak98_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_ak98pcm *ak98pcm = card->private_data;
	struct snd_pcm_substream *substream_playback = ak98pcm->playbacksubstrm;
	struct snd_pcm_substream *substream_capture = ak98pcm->capturesubstrm;
	struct snd_pcm_runtime *runtime_playback = NULL;
	struct snd_pcm_runtime *runtime_capture = NULL;
	dma_addr_t paddr_playback = 0;
	dma_addr_t paddr_capture = 0;
	u8 id_DAC = 0;
	u8 id_ADC23 = 0;
	int i=0;
	
	if(substream_playback!=NULL)
	{
		runtime_playback = substream_playback->runtime;
		paddr_playback = runtime_playback->dma_addr;
		id_DAC = ak98pcm->L2BufID_For_DAC;
	}
	if(substream_capture!=NULL)
	{
		runtime_capture = substream_capture->runtime;
		paddr_capture = runtime_capture->dma_addr;
		id_ADC23 = ak98pcm->L2BufID_For_ADC23;
	}

	for(i=MIXER_ADDR_HPSRC;i<=MIXER_ADDR_LASTSRC;i++)
	{
		if(ak98pcm->mixer_source[i])
		{
			if(ak98pcm->mixer_source[i]&SOURCE_DAC)//if playback stream is running when suspend
			{
				AK98_DAC_Open();
				if((substream_playback!=NULL) &&
					test_bit(0,&ak98pcm->playbackStrmDMARunning))
				{
					init_completion(&(ak98pcm->playbackHWDMA_completion));
					ak98_l2_clr_status(id_DAC);
					ak98_l2_combuf_dma(paddr_playback+ak98pcm->PlaybackCurrPos, id_DAC, ak98pcm_playback_period_bytes_min, 
							   (ak98_l2_dma_transfer_direction_t)MEM2BUF,1);
				}
			}
			if(ak98pcm->mixer_source[i]&SOURCE_LINEIN)  //linein input when suspend
			{
				AK98_Linein_PowerOn(1);
			}
			if(ak98pcm->mixer_source[i]&SOURCE_MIC)    //mic input(recording) when suspend
			{
				AK98_Mic_PowerOn(1);
			}
			AK98_Poweron_VCM_REF(1);
			if(MIXER_ADDR_HPSRC == i)  //hp channel is opend when suspend
			{
				AK98_Set_HP_In(ak98pcm->mixer_source[i]);
				ak98_gpio_setpin(ak98pcm->hpmute_gpio.pin, ak98pcm->hpmute_enable_value);
				AK98_Poweron_HP(1);
				ak98_gpio_setpin(ak98pcm->hpmute_gpio.pin, ak98pcm->hpmute_disable_value);
			}
			else if(MIXER_ADDR_LINEOUTSRC == i)  //lineout channel is opend when suspend
			{
				AK98_Set_Bypass(ak98pcm->mixer_source[i]);
				AK98_Poweron_Speaker(ak98pcm->spkrshdn_gpio.pin,1);
			}
			else if(MIXER_ADDR_ADC23SRC == i)   //ADC23 is powed on when suspend
			{
				AK98_Set_ADC23_In(ak98pcm->mixer_source[i]);
				AK98_ADC23_Open();
				if((substream_capture!=NULL) &&
					test_bit(0,&ak98pcm->captureStrmDMARunning))
				{
					init_completion(&(ak98pcm->captureHWDMA_completion));
					ak98_l2_clr_status(id_ADC23);
					ak98_l2_combuf_dma(paddr_capture+ak98pcm->CaptureCurrPos, id_ADC23, ak98pcm_capture_period_bytes_min, 
							   (ak98_l2_dma_transfer_direction_t)BUF2MEM,1);
				}
			}
		}
	}
	return 0;
}
#else
#define snd_ak98_suspend NULL
#define snd_ak98_resume NULL
#endif


#define SND_AK98PCM_DRIVER	"snd_ak98pcm"

static struct platform_driver snd_ak98pcm_driver = {
	.probe		= snd_ak98pcm_probe,
	.remove		= __devexit_p(snd_ak98pcm_remove),
	.suspend	= snd_ak98_suspend,
	.resume		= snd_ak98_resume,
	.driver		= {
		.name	= SND_AK98PCM_DRIVER
	},
};

/**
 * @brief  register driver
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static int __init alsa_card_ak98pcm_init(void)
{
	int err;

	err = platform_driver_register(&snd_ak98pcm_driver);
	if (err < 0)
		return err;
	
	return 0;
}

/**
 * @brief  unregister driver
 * @author  Cheng Mingjuan
 * @date   
 * @return void
 */
static void __exit alsa_card_ak98pcm_exit(void)
{
	platform_driver_unregister(&snd_ak98pcm_driver);
}

module_init(alsa_card_ak98pcm_init)
module_exit(alsa_card_ak98pcm_exit)
