/************************************************************************************************
**
**    AUDIO EXTERNAL AMP
**
**    FILE
**        pantech_snd_ext_amp_yda165.c
**
**    DESCRIPTION
**        This file contains audio external amp api
**          
**          void snd_extamp_api_Init()
**          void snd_extamp_api_SetPath()
**          void snd_extamp_api_SetVolume()
**          void snd_extamp_api_Sleep()
**
**    Copyright (c) 2010 by PANTECH Incorporated.  All Rights Reserved.
*************************************************************************************************/


/************************************************************************************************
** Includes
*************************************************************************************************/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <asm/ioctls.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include "yda165.h"
#include "yda165_ctrl.h"
#include "yda165_machdep.h"
#ifdef YDA_CALTOOL
//for yda caltool
#include "yda165_cal.h"
#endif

/************************************************************************************************
** Definition
*************************************************************************************************/
struct yda165_data {
	struct i2c_client *client;	/* for I2C devices only */
	const char *name;
// 	struct mutex update_lock;
// 	enum chips type;
};
/************************************************************************************************
** Variables
*************************************************************************************************/

extern struct miscdevice miscdev;

#ifdef FEATURE_PANTECH_SND_DOMESTIC
extern int is_dual_path;
extern int is_voice_call_mode;
extern int is_vt_mode;
#endif

#ifdef CONFIG_SKY_SND_DOCKING_CRADLE
extern int is_docking_mode;
extern int wcd9310_headsetJackStatusGet(void);
#endif

int g_pantech_amp_log[4]={0,1,1,1};
struct i2c_client *yda165_i2c_client = NULL;

int g_current_dev;
int g_current_mode;

D4HP3_SETTING_INFO g_extamp_info[3];

// ------------------------------------------------------------

D4HP3_SETTING_INFO g_extamp_info_org[3] = {

	{   // mm
	    .bLine1Gain         = INVOL_GAIN_M_1D5_DB, //20130130 frogLove - gain reduce 0dB -> -1.5dB by HW
	    .bLine2Gain         = INVOL_GAIN_P_0_DB,
	    .bLine1Balance      = LINE_BALANCE_SINGLE,
	    .bLine2Balance      = LINE_BALANCE_DIFF,
			
	    .bHpCpMode          = HP_CP_3,
	    .bHpAvddLev         = HP_AVDD_1P65_2P4,
	    .bHpEco             = HP_ECO_OFF,
	    .bHpAtt             = HP_ATT_GAIN,
	    .bHpGainUp          = HP_OUTPUT_AMP_GAIN,
	    .bHpSvol            = HP_SOFTVOL_ON,
	    .bHpZcs             = HP_ZC_OFF,
	    .bHpCh              = HP_CH_STEREO,
	    .bHpMixer_Line1     = HP_MIX_OFF,   //HP_MIX_ON,
	    .bHpMixer_Line2     = HP_MIX_OFF,
	            
	    .bSpAtt             = SP_ATT_GAIN,
	    .bSpGainUp          = SP_OUTPUT_AMP_GAIN,
	    .bSpSvol            = SP_SOFTVOL_ON,
	    .bSpZcs             = SP_ZC_OFF,
	    .bSpMixer_Line1     = SP_MIX_OFF,
	    .bSpMixer_Line2     = SP_MIX_OFF, //SP_MIX_ON,
	    .bSpNg_DetectionLv  = SP_NG_OFF,
	    .bSpNg_AttackTime   = SP_NG_ATIME_800,
	    .bSpNcpl_NonClipRatio   = 1,        // LQ 1%
	    .bSpNcpl_PowerLimit = 11,   // 800mW
	    .bSpNcpl_AttackTime = 1,
	    .bSpNcpl_ReleaseTime    = 1,
	},
	{   // voice
	    .bLine1Gain         = INVOL_GAIN_M_1D5_DB, //20130130 frogLove - gain reduce 0dB -> -1.5dB by HW
	    .bLine2Gain         = INVOL_GAIN_P_12_DB,  //SPKR INVOL 12dB for HW Tune 20121213
	    .bLine1Balance      = LINE_BALANCE_SINGLE,
	    .bLine2Balance      = LINE_BALANCE_DIFF,

	    .bHpCpMode          = HP_CP_3,
	    .bHpAvddLev         = HP_AVDD_1P65_2P4,
	    .bHpEco             = HP_ECO_OFF,
	    .bHpAtt             = HP_ATT_GAIN,
	    .bHpGainUp          = HP_OUTPUT_AMP_GAIN,
	    .bHpSvol            = HP_SOFTVOL_ON,
	    .bHpZcs             = HP_ZC_OFF,
	    .bHpCh              = HP_CH_STEREO,
	    .bHpMixer_Line1     = HP_MIX_OFF,   //HP_MIX_ON,
	    .bHpMixer_Line2     = HP_MIX_OFF,

	    .bSpAtt             = SP_ATT_GAIN,
	    .bSpGainUp          = SPAMP_GAIN_P_20_DB,  // V:20 // 110406
	    .bSpSvol            = SP_SOFTVOL_ON,
	    .bSpZcs             = SP_ZC_OFF,
	    .bSpMixer_Line1     = SP_MIX_OFF,
	    .bSpMixer_Line2     = SP_MIX_OFF, //SP_MIX_ON,
	    .bSpNg_DetectionLv  = SP_NG_OFF,
	    .bSpNg_AttackTime   = SP_NG_ATIME_800,
	    .bSpNcpl_NonClipRatio   = 3,        // V: 5% // 110406
	    .bSpNcpl_PowerLimit = 11,   // 800mW
	    .bSpNcpl_AttackTime = 2,        // V: 0.5 // 110406
	    .bSpNcpl_ReleaseTime    = 1,        // V: 200
	},
	{   // vt & volte
	    .bLine1Gain         = INVOL_GAIN_M_1D5_DB, //20130130 frogLove - gain reduce 0dB -> -1.5dB by HW
		.bLine2Gain 		= INVOL_GAIN_P_12_DB,  //SPKR INVOL 12dB for HW Tune 20121213
		.bLine1Balance		= LINE_BALANCE_SINGLE,
		.bLine2Balance		= LINE_BALANCE_DIFF,
		
		.bHpCpMode			= HP_CP_3,
		.bHpAvddLev 		= HP_AVDD_1P65_2P4,
		.bHpEco 			= HP_ECO_OFF,
		.bHpAtt 			= HP_ATT_GAIN,
		.bHpGainUp			= HP_OUTPUT_AMP_GAIN,
		.bHpSvol			= HP_SOFTVOL_ON,
		.bHpZcs 			= HP_ZC_OFF,
		.bHpCh				= HP_CH_STEREO,
		.bHpMixer_Line1 	= HP_MIX_OFF,	//HP_MIX_ON,
		.bHpMixer_Line2 	= HP_MIX_OFF,
		
		.bSpAtt 			= SP_ATT_GAIN,
		.bSpGainUp			= SPAMP_GAIN_P_20_DB,  // V:20 // 110406
		.bSpSvol			= SP_SOFTVOL_ON,
		.bSpZcs 			= SP_ZC_OFF,
		.bSpMixer_Line1 	= SP_MIX_OFF,
		.bSpMixer_Line2 	= SP_MIX_OFF, //SP_MIX_ON,
		.bSpNg_DetectionLv	= SP_NG_OFF,
		.bSpNg_AttackTime	= SP_NG_ATIME_800,
		.bSpNcpl_NonClipRatio	= 3,		// V: 5% // 110406
		.bSpNcpl_PowerLimit = 11,	// 800mW
		.bSpNcpl_AttackTime = 2,		// V: 0.5 // 110406
		.bSpNcpl_ReleaseTime	= 1,		// V: 200
	}
};
// ------------------------------------------------------------

/************************************************************************************************
** Function Declaration
*************************************************************************************************/
//static long aud_sub_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
//static int aud_sub_open(struct inode *inode, struct file *file);
//static int aud_sub_release(struct inode *inode, struct file *file);

static int yda165_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit yda165_remove(struct i2c_client *client);
static int yda165_suspend(struct device *dev);
static int yda165_resume(struct device *dev);
static void yda165_shutdown(struct i2c_client *client);

void snd_extamp_api_Init(void);
void snd_extamp_api_SetDevice(int on, uint32_t cad_device);

void snd_extamp_api_set_default_dev(int mode);
void snd_extamp_api_SetVolume(uint32_t hp_vol, uint32_t sp_vol);
void snd_extamp_print_all_regs(D4HP3_SETTING_INFO *amp_info);

/************************************************************************************************
** Function Definition
*************************************************************************************************/

#define ON	1
#define OFF	0

/*==========================================================================
** yda165_probe
**=========================================================================*/
static int yda165_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;
	struct yda165_data *data;

	data = kzalloc(sizeof(struct yda165_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->client = client;
	data->name = client->name;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c func not supported\n");
		return -EIO;
	}

	yda165_i2c_client = client;

	/*
		TO DO platform data support
	*/

	g_current_mode = MODE_MM;
	snd_extamp_api_set_default(0);
	snd_extamp_api_set_default(1);
	snd_extamp_api_set_default(2);

	printk(KERN_INFO "%s: success addr=[%x]\n",__func__,client->addr);

	return 0;

exit:
	printk(KERN_ERR "%s: failed err=%d\n",__func__,err);
	return err;
    
}

/*==========================================================================
** yda165_remove
**=========================================================================*/
static int __exit yda165_remove(struct i2c_client *client)
{
	int rc = 0;
	pantech_amp_log(0, "yda165_remove");
	yda165_i2c_client = NULL;
	/*rc = i2c_detach_client(client);*/
	return rc;
}

/*==========================================================================
** yda165_suspend
**=========================================================================*/
static int yda165_suspend(struct device *dev)
{
	pantech_amp_log(0, "yda165_suspend");
	return 0;
}

/*==========================================================================
** yda165_resume
**=========================================================================*/
static int yda165_resume(struct device *dev)
{
	pantech_amp_log(0, "yda165_resume");
	return 0;
}

/*==========================================================================
** yda165_shutdown
**=========================================================================*/
static void yda165_shutdown(struct i2c_client *client)
{
	pantech_amp_log(0, "yda165_shutdown");
	// TODO:
	//g_tExtampInfo.power_man_ctl_reg_val &= ~SHDN_EN;
	//g_tExtampInfo.power_man_ctl_reg_val = g_tExtampInfo.power_man_ctl_reg_val | SHDN_DIS;
	
	//snd_extamp_i2c_write(POWER_MANAGEMENT_CTL_REG_ADDR, g_tExtampInfo.power_man_ctl_reg_val);
	pantech_amp_log(0, "AMP Shutdown for power-off");
}

/*=========================================================================*/

static const struct i2c_device_id yda165_id[] = {
	{ "yda165-amp", 0},
};

static struct dev_pm_ops i2c_device_yda165_pm_ops = {
             .suspend = yda165_suspend, 
             .resume = yda165_resume,
};

static struct i2c_driver yda165_driver = {
	.id_table = yda165_id,
	.probe  = yda165_probe,
	.remove = __exit_p(yda165_remove),
	.shutdown = yda165_shutdown,
	.driver = {
		.name = "yda165-amp",
		.pm = &i2c_device_yda165_pm_ops,
	},
};

/*==========================================================================
** snd_extamp_api_Init
**=========================================================================*/
void snd_extamp_api_Init(void)
{
	int result = 0;

	result = misc_register(&miscdev);
	if(result)
	{
		pantech_amp_log(1, "+++++ snd_extamp_api_Init: misc_register failed");
	}

	pantech_amp_log(0, "+++++ snd_extamp_api_Init");
	//printk("============================ i2c_add_driver\n");
	result = i2c_add_driver(&yda165_driver);
	//printk("i2c_add_driver result(%i)\n", result);
	if(result){
		pantech_amp_log(1, "init yda165 Fail");
	}
}

/*==========================================================================
** snd_extamp_api_SetDevice
**
** 	voice	0	multimedia
**			1	voice
**			2	tty
**=========================================================================*/
void snd_extamp_api_SetDevice(int on, uint32_t cad_device)
{	
	int mode = 0;
	int new_mode = 0;
	UINT8 hp_flag = OFF, sp_flag = OFF;

	pantech_amp_log(0, "+++++ snd_extamp_api_SetDevice(%d, %d, %d)", on, mode, cad_device);

#ifdef F_PANTECH_SND_EXTAMP_TEST_TOOL
	if (on) g_current_dev = cad_device;
	else g_current_dev = -1;
#endif

	// ------------------------------------------------------------
	if ( is_dual_path == 1 ) {
		mode = MODE_MM;
		pantech_amp_log(0, "MODE_MM");
	}
	else if ( is_voice_call_mode == 1 ) {
		mode = MODE_VOICE;
		pantech_amp_log(0, "MODE_VOICE");
	}
	else if ( is_vt_mode == 1 ) {
		mode = MODE_VT;
		pantech_amp_log(0, "MODE_VT");		
	}
	else {
		mode = MODE_MM;
		pantech_amp_log(0, "MODE_MM");
	}

	if (g_current_mode != mode) {
		new_mode = 1;
		g_current_mode = mode;
	}

#ifdef CONFIG_SKY_SND_SPK_DEBUG_LOG  // 20121217 jhsong : cannot hear in speaker, so debug msg log added
	pr_err("@#@#snd_extamp_api_SetDevice...............is_docking_mode (%d),   cad_device : %d  -> on : %d, mode : %d, new_mode : %d", is_docking_mode, cad_device, on, mode,new_mode);
#endif

	// ------------------------------------------------------------
	
	switch(cad_device) {
		case SND_DEVICE_HANDSET_RX: {
			pantech_amp_log(0, "snd_extamp_api_SetDevice: SND_DEVICE_HANDSET_RX (%d)", on);
			hp_flag = ON;
			sp_flag = ON;

			g_extamp_info[g_current_mode].bHpCh = HP_CH_STEREO;

			g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_OFF;
			g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_OFF;

			g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
			g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_OFF;		
		} break;
		case SND_DEVICE_HEADSET_RX: {
			pantech_amp_log(0, "snd_extamp_api_SetDevice: SND_DEVICE_HEADSET_RX (%d)", on);
			hp_flag = ON;

			g_extamp_info[g_current_mode].bHpCh = HP_CH_STEREO;
			if (on) {
				g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_ON;
				g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_OFF;
			} else {
				g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_OFF;
				g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_OFF;
			}
		} break;
		
		case SND_DEVICE_SPEAKER_RX: {
			pantech_amp_log(0, "snd_extamp_api_SetDevice: SND_DEVICE_SPEAKER_RX (%d)", on);
			sp_flag = ON;
			
			if (on) {
				g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
				g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_ON;
			} else {
				g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
				g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_OFF;
			}
		} break;

		case SND_DEVICE_TTY_HEADSET_MONO_RX:
		case SND_DEVICE_SPEAKER_HEADSET_RX: {
			pantech_amp_log(0, "snd_extamp_api_SetDevice: SND_DEVICE_SPEAKER_HEADSET_RX (%d)", on);
			hp_flag = ON;
			sp_flag = ON;

			g_extamp_info[g_current_mode].bHpCh = HP_CH_STEREO;
			if (on) {
				g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_ON;
				g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_OFF;

				g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
				g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_ON;
			} else {
				g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_OFF;
				g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_OFF;

				g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
				g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_OFF;
			}
		} break;

		case SND_DEVICE_DOCK_SPEAKER_RX:
		{
			pantech_amp_log(0, "snd_extamp_api_SetDevice: SND_DEVICE_DOCK_SPEAKER_RX (%d)", on);

			hp_flag = ON;
			sp_flag = ON;

			g_extamp_info[g_current_mode].bHpCh = HP_CH_STEREO;

			if (on) {
				g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_OFF;
				g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_ON;

				g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
				g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_OFF;
			} else {
				g_extamp_info[g_current_mode].bHpMixer_Line1 = HP_MIX_OFF;
				g_extamp_info[g_current_mode].bHpMixer_Line2 = HP_MIX_OFF;

				g_extamp_info[g_current_mode].bSpMixer_Line1 = SP_MIX_OFF;
				g_extamp_info[g_current_mode].bSpMixer_Line2 = SP_MIX_OFF;
			}
		} break;

		default: {
			pantech_amp_log(0, "snd_extamp_api_SetDevice: INVALID_DEVICE-%d (%d)", cad_device, on);
		} break;
	}

	// ------------------------------------------------------------

	if (new_mode) {
		D4Hp3_PowerOn(&g_extamp_info[g_current_mode]);  // set all reg values
	}
	else {
		D4Hp3_ControlMixer(hp_flag, sp_flag, &g_extamp_info[g_current_mode]); // set only mixer values
	}
}

inline void set_amp_gain(int amp_state)
{
	switch(amp_state) {
	case SPK_ON:
		snd_extamp_api_SetDevice(1, SND_DEVICE_SPEAKER_RX);
		break;
	case SPK_OFF:
		snd_extamp_api_SetDevice(0, SND_DEVICE_SPEAKER_RX);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(set_amp_gain);
/*==========================================================================
** snd_extamp_api_set_default
**
** 	mode	0	multimedia
**			1	voice
**			2	tty
**=========================================================================*/

void snd_extamp_api_set_default(int mode)
{
	//D4HP3_SETTING_INFO amp_reg_info;
	extamp_hpch_e _bHpCh = HP_CH_STEREO;
	extamp_hpmix_e _bHpMixer_Line1 = HP_MIX_OFF;
	extamp_hpmix_e _bHpMixer_Line2 = HP_MIX_OFF;
	extamp_spmix_e _bSpMixer_Line1 = SP_MIX_OFF;
	extamp_spmix_e _bSpMixer_Line2 = SP_MIX_OFF;

	//pantech_amp_log(0, "snd_extamp_api_set_default() mode %i", mode);

	if (g_current_mode == mode) {
		// jykim110414@PS3
		// If we do not backup the mixer value, all mixer will turn off after calling D4Hp3_PowerOn.
		_bHpCh = g_extamp_info[g_current_mode].bHpCh;

		_bHpMixer_Line1 = g_extamp_info[g_current_mode].bHpMixer_Line1;
		_bHpMixer_Line2 = g_extamp_info[g_current_mode].bHpMixer_Line2;

		_bSpMixer_Line1 = g_extamp_info[g_current_mode].bSpMixer_Line1;
		_bSpMixer_Line2 = g_extamp_info[g_current_mode].bSpMixer_Line2;
	}

	// ------------------------------------------------------------

	// set register values
	switch(mode) {
		case MODE_MM: {  // jykim110411@PS3: multimedia setting
			memcpy(&g_extamp_info[MODE_MM], &g_extamp_info_org[MODE_MM], sizeof(D4HP3_SETTING_INFO));
		} break;
		case MODE_VOICE: {  // voice
			memcpy(&g_extamp_info[MODE_VOICE], &g_extamp_info_org[MODE_VOICE], sizeof(D4HP3_SETTING_INFO));
		} break;
		case MODE_VT: {  // tty
			memcpy(&g_extamp_info[MODE_VT], &g_extamp_info_org[MODE_VT], sizeof(D4HP3_SETTING_INFO));
		} break;
		default: {
			pantech_amp_log(1, "ERROR: Unknown mode %i", mode);
		}
	}

	// ------------------------------------------------------------

	if (g_current_mode == mode) {
		g_extamp_info[g_current_mode].bHpCh= _bHpCh;

		g_extamp_info[g_current_mode].bHpMixer_Line1 = _bHpMixer_Line1;
		g_extamp_info[g_current_mode].bHpMixer_Line2 = _bHpMixer_Line2;

		g_extamp_info[g_current_mode].bSpMixer_Line1 = _bSpMixer_Line1;
		g_extamp_info[g_current_mode].bSpMixer_Line2 = _bSpMixer_Line2;
		
		D4Hp3_PowerOn(&g_extamp_info[g_current_mode]);
	}
}


/*==========================================================================
** snd_extamp_api_SetDefault
**
** 	mode	0	multimedia
**			1	voice
**			2	tty
**
** If this fuction is called, the mixer is tured on. This function does not consider the current mixer status.
** It's better calling only snd_extamp_api_set_default.
**=========================================================================*/

void snd_extamp_api_set_default_dev(int mode)
{
	snd_extamp_api_set_default(mode);
	if ((g_current_dev != -1) && (g_current_mode == mode)) snd_extamp_api_SetDevice(1, g_current_dev);
}

/*==========================================================================
** snd_extamp_print_all_regs
**=========================================================================*/
void snd_extamp_print_all_regs(D4HP3_SETTING_INFO *amp_info)
{
	pantech_amp_log(1, "\n-----------------------------------");
	pantech_amp_log(1, "bLine1Gain = %d", amp_info->bLine1Gain);
	pantech_amp_log(1, "bLine2Gain = %d", amp_info->bLine2Gain);
	pantech_amp_log(1, "bLine1Balance = %d", amp_info->bLine1Balance);
	pantech_amp_log(1, "bLine2Balance = %d\n", amp_info->bLine2Balance);
	
	pantech_amp_log(1, "bHpCpMode = %d", amp_info->bHpCpMode);
	pantech_amp_log(1, "bHpAvddLev = %d", amp_info->bHpAvddLev);
	pantech_amp_log(1, "bHpEco = %d", amp_info->bHpEco);
	pantech_amp_log(1, "bHpAtt = %d", amp_info->bHpAtt);
	pantech_amp_log(1, "bHpGainUp = %d", amp_info->bHpGainUp);
	pantech_amp_log(1, "bHpSvol = %d", amp_info->bHpSvol);
	pantech_amp_log(1, "bHpZcs = %d", amp_info->bHpZcs);
	pantech_amp_log(1, "bHpCh = %d", amp_info->bHpCh);
	pantech_amp_log(1, "bHpMixer_Line1 = %d", amp_info->bHpMixer_Line1);
	pantech_amp_log(1, "bHpMixer_Line2 = %d\n", amp_info->bHpMixer_Line2 );
	
	pantech_amp_log(1, "bSpAtt = %d", amp_info->bSpAtt);
	pantech_amp_log(1, "bSpGainUp = %d", amp_info->bSpGainUp);
	pantech_amp_log(1, "bSpSvol = %d", amp_info->bSpSvol);
	pantech_amp_log(1, "bSpZcs = %d", amp_info->bSpZcs);
	pantech_amp_log(1, "bSpMixer_Line1 = %d", amp_info->bSpMixer_Line1);
	pantech_amp_log(1, "bSpMixer_Line2 = %d", amp_info->bSpMixer_Line2);
	pantech_amp_log(1, "bSpNg_DetectionLv = %d", amp_info->bSpNg_DetectionLv);
	pantech_amp_log(1, "bSpNg_AttackTime = %d", amp_info->bSpNg_AttackTime);
	pantech_amp_log(1, "bSpNcpl_NonClipRatio = %d", amp_info->bSpNcpl_NonClipRatio);
	pantech_amp_log(1, "bSpNcpl_PowerLimit = %d", amp_info->bSpNcpl_PowerLimit);
	pantech_amp_log(1, "bSpNcpl_AttackTime = %d", amp_info->bSpNcpl_AttackTime);
	pantech_amp_log(1, "bSpNcpl_ReleaseTime = %d", amp_info->bSpNcpl_ReleaseTime);
	pantech_amp_log(1, "-----------------------------------\n");
}

#ifdef FEATURE_PANTECH_SND_DOMESTIC
int snd_get_dual_path(void) {
	return is_dual_path;
}
#endif

#ifdef CONFIG_SKY_SND_DOCKING_CRADLE
int snd_get_docking_mode(void) {
	return is_docking_mode;
}
#endif

#ifdef YDA_CALTOOL
/*==========================================================================
** Function: 		yda_snd_subsystem_get_device
** Parameter: 	void
** Return:		Device type(number)
** Description:	Return current device type
**=========================================================================*/
int yda_snd_subsystem_get_device(void)
{

	return g_current_mode;
}

#endif

static int __init yda165_amp_init(void)
{
	int rc;

	rc =  misc_register(&miscdev);
	if (rc) {
		pr_err("yda165_amp_init: register miscdevice failed\n");
		goto exit;
	}
	rc = i2c_add_driver(&yda165_driver);
	if (rc) {
		pr_err("yda165_amp_init: register i2c driver failed\n");
		goto exit;
	}
	printk(KERN_INFO "%s: misc & i2c driver ret=%d completed\n",__func__,rc);

exit:
	return rc;
}

static void __exit yda165_amp_exit(void)
{
	return i2c_del_driver(&yda165_driver);
}

module_init(yda165_amp_init);
module_exit(yda165_amp_exit);

MODULE_DESCRIPTION("YDA165 Amp Control");
MODULE_AUTHOR("Jeong JungSik <chero@pantech.com>");
MODULE_LICENSE("GPL");

