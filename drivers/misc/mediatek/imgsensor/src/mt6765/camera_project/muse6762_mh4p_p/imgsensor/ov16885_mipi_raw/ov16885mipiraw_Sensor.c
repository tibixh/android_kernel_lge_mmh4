/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
#include <mt-plat/mtk_boot.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov16885mipiraw_Sensor.h"

#define PFX "ov16885mipiraw_Sensor"

//Gionee <GN_BSP_CAMERA> <xuxiaojiang> <20170826> add for CR179907 ov16885 capture size and setting change from 16m to 4m begin
//static  kal_uint16 ROTATION=01; //00 for normal, 01 for ratation 180degree.
//Gionee <GN_BSP_CAMERA> <xuxiaojiang> <20170826> add for CR179907 ov16885 capture size and setting change from 16m to 4m end
//[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2019-02-21] enable the burst i2c write mode
#define I2C_BUFFER_LEN 254//for Burst mode
#define BLOCK_I2C_DATA_WRITE iBurstWriteReg
//[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2019-02-21] enable the burst i2c write mode
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_DEBUG(format, args...)    pr_err(PFX "[LGE][%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId, u16 speed);
extern char rear_sensor_name[20];/*LGE_CHANGE, 2019-07-04, add the camera identifying logic , kyunghun.oh@lge.com*/
static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV16885_SENSOR_ID,	/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0xf5f23ab,	/* checksum value for Camera Auto Test */

	.pre = {                    //check
		.pclk = 160000000,	
		.linelength = 1400,	
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
	},
	.cap1 = {		/* 95:line 5312, 52/35:line 5336 */
	        .pclk = 160000000,	
		.linelength = 1400, 
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4672,
		.grabwindow_height = 3504,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
	},
	.cap = {
		.pclk = 160000000,	
		.linelength = 1400, 
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4672,
		.grabwindow_height = 3504,	
		.mipi_data_lp2hs_settle_dc = 105,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
	},
	.normal_video = {
		.pclk = 160000000,	
		.linelength = 1400,	
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4672,/* make the fullsize recording works*/
		.grabwindow_height = 3504,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
	},
	.hs_video = {
		.pclk = 160000000,	
		.linelength = 1040,	
		.framelength = 1282,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,	
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 1200,
		.mipi_pixel_rate = 576000000,
	},
	.slim_video = {
		.pclk = 160000000,	
		.linelength = 1400,	
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2336,
		.grabwindow_height = 1752,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
	},
	.custom1 = {
		.pclk = 128000000,
		.linelength = 1400, 
		.framelength = 3808,//3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4672,
		.grabwindow_height = 3504,	
		.mipi_data_lp2hs_settle_dc = 105,
		.max_framerate = 240,	
		.mipi_pixel_rate = 460800000,
	},
	.margin = 8,		/* sensor framelength & shutter margin */
	.min_shutter = 8,	/* min shutter */
	.max_frame_length = 0x7fff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,	/* shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,	/* sensor gain delay frame for AE cycle, 2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.frame_time_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 6,	/* support sensor mode num ,don't support Slow motion */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 video delay frame num */

	.isp_driving_current = ISP_DRIVING_8MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_CSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,/*SENSOR_OUTPUT_FORMAT_RAW_4CELL_B,*/ /*sensor output first pixel color*/
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x6C, 0xff},	/* record sensor support all write id addr,
							 * only supprt 4must end with 0xff*/
    .i2c_speed = 1000,
};


static struct imgsensor_struct imgsensor = {
//Gionee:malp 20150514 modify for mirrorflip start
#ifdef ORIGINAL_VERSION
	.mirror = IMAGE_NORMAL, 			//mirrorflip information
#else 
	.mirror = IMAGE_HV_MIRROR,			   //mirrorflip information
#endif
//Gionee:malp 20150514 modify for mirrorflip end

	.sensor_mode = IMGSENSOR_MODE_INIT,	/* IMGSENSOR_MODE enum value,
						 * record current sensor mode,such as:
						 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
						 */
	.shutter = 0x4C00,	/* current shutter */
	.gain = 0x200,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 30,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,	/* auto flicker enable:
					 * KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
					 */
	.test_pattern = KAL_FALSE,	/* test pattern mode or not.
					 * KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
					 */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20,	/* record current sensor's i2c write id */
};
//[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2019-02-21] enable the burst i2c write mode
/* Sensor output window information*/
void OV16885_write_cmos_sensor_burst(ov16885_short_t* para, kal_uint32 len, kal_uint32 slave_addr)
{
	ov16885_short_t* pPara = (ov16885_short_t*) para;
	kal_uint8 puSendCmd[I2C_BUFFER_LEN]={0,};
	kal_uint32 tosend=0 , IDX=0;
	kal_uint16 addr, addr_next, data;

	if(pPara == NULL)
{
		LOG_INF("[OV16885] ERROR!! pPara is Null!!\n");
		return;
	}

	while(IDX < len)
	{
		addr = pPara->address;
		if(tosend == 0)
		{
			puSendCmd[tosend++] = (kal_uint8)(addr >> 8) & 0xff;
			puSendCmd[tosend++] = (kal_uint8)(addr & 0xff);
			//data = (pPara->data >> 8) & 0xff;
			//puSendCmd[tosend++] = (kal_uint8)data;
			data = pPara->data & 0xff;
			puSendCmd[tosend++] = (kal_uint8)data;
			addr_next = addr + 1;
			IDX ++;
			pPara++;
		}
		else if (addr == addr_next)
		{
			//data = (pPara->data >> 8) & 0xff;
			//puSendCmd[tosend++] = (kal_uint8)data;
			data = pPara->data & 0xff;
			puSendCmd[tosend++] = (kal_uint8)data;
			addr_next = addr + 1;
			IDX ++;
			pPara++;
		}

		else // to send out the data if the address not incremental.
		{
			BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, slave_addr, imgsensor_info.i2c_speed);
			tosend = 0;
		}

		// to send out the data if the sen buffer is full or last data.
		if ((tosend >= (I2C_BUFFER_LEN-8)) || (IDX == len))
		{
			BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, slave_addr, imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return;
}
//[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2019-02-21] enable the burst i2c write mode
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{4704, 3536,    0,    0, 4704, 3536, 2352, 1768,  8,  8, 2336, 1752, 0, 0, 2336, 1752},	/* Preview check*/
	{4704, 3536,    0,    0, 4704, 3536, 4704, 3536, 16, 16, 4672, 3504, 0, 0, 4672, 3504},	/* capture */
	{4704, 3536,    0,    0, 4704, 3536, 4704, 3536, 16, 16, 4672, 3504, 0, 0, 4672, 3504},	/* video */
	{4704, 3536, 1064, 1040, 2576, 1456, 1288,  728,  4,  4, 1280,  720, 0, 0, 1280, 720},	/* hs vedio */
	{4704, 3536,    0,    0, 4704, 3536, 2352, 1768,  8,  8, 2336, 1752, 0, 0, 2336, 1752},	/* slim vedio */
	{4704, 3536,    0,    0, 4704, 3536, 4704, 3536, 16, 16, 4672, 3504, 0, 0, 4672, 3504},	/* custom1 */
};

/*PDAF START*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX =  0,
	.i4OffsetY =  0,
	.i4PitchX  = 32, 
	.i4PitchY  = 32, 
	.i4PairNum =  8, 
	.i4SubBlkW = 16, 
	.i4SubBlkH =  8,
	.i4BlockNumX = 146,
	.i4BlockNumY = 109,
	.i4PosL = {
		{14, 6}, {30, 6}, {6, 10}, {22, 10}, {14, 22}, {30, 22}, {6, 26}, {22, 26},
	},
	.i4PosR = {
		{14, 2}, {30, 2}, {6, 14}, {22, 14}, {14, 18}, {30, 18}, {6, 30}, {22, 30},
	},
	.iMirrorFlip = 0,
};
typedef struct SET_PD_BLOCK_INFO_T SET_PD_BLOCK_INFO_T;
static SET_PD_BLOCK_INFO_T *PDAFinfo;
/*PDAF END*/

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

/*#ifdef CONFIG_HQ_HARDWARE_INFO
static kal_uint16 read_eeprom_module(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 1, EEPROM_ADDR);

	return get_byte;
}
#endif*/

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	/* check */
	/* LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel); */

	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable? %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */

#if 0
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	    ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
			write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length */
		imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter */
	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	write_cmos_sensor(0x3502, (shutter << 4) & 0xF0);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

	/* LOG_INF("frame_length = %d ", frame_length); */

}				/*      write_shutter  */
#endif

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	    ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	shutter = (shutter >> 1) << 1;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter */
	write_cmos_sensor(0x3502, (shutter << 4) & 0xF0);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	LOG_INF("Exit! shutter =%d, framelength =%d, for flicker realtime_fps=%d\n", shutter,
		imgsensor.frame_length, realtime_fps);

}



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	/* platform 1xgain = 64, sensor driver 1*gain = 0x80 */
	iReg = gain * 128 / BASEGAIN;

	if (iReg < 0x80)	/* sensor 1xGain */
		iReg = 0X80;

	if (iReg > 0x7c0)	/* sensor 15.5xGain */
		iReg = 0X7C0;

	return iReg;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x3508, (reg_gain >> 8));
	write_cmos_sensor(0x3509, (reg_gain & 0xFF));
	write_cmos_sensor(0x350c, (reg_gain >> 8));
	write_cmos_sensor(0x350d, (reg_gain & 0xFF));
	return gain;
}				/*      set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_control enable =%d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0x01);
	else
		write_cmos_sensor(0x0100, 0x00);

	mdelay(10);

	return ERROR_NONE;
}

static void set_shutter_frame_length(kal_uint16 shutter,
			kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;

	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	if (frame_length > 1)
		imgsensor.frame_length = frame_length;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter > (imgsensor_info.max_frame_length -
		imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
		imgsensor_info.margin) : shutter;

	//frame_length and shutter should be an even number.
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
//auroflicker:need to avoid 15fps and 30 fps
	if (imgsensor.autoflicker_en == KAL_TRUE) {
		realtime_fps = imgsensor.pclk /
			imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			realtime_fps = 296;
	    set_max_framerate(realtime_fps, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			realtime_fps = 146;
	    set_max_framerate(realtime_fps, 0);
		} else {
		imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
	    write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	    write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
	imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;

		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	write_cmos_sensor(0x3500, (shutter >> 12) & 0x0F);
	write_cmos_sensor(0x3501, (shutter >> 4) & 0xFF);
	write_cmos_sensor(0x3502, (shutter<<4)  & 0xF0);

	LOG_INF("shutter =%d, framelength =%d, realtime_fps =%d\n",
		shutter, imgsensor.frame_length, realtime_fps);
}				/* set_shutter_frame_length */

#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_DEBUG("image_mirror = %d\n", image_mirror);

	/********************************************************
	 *
	 *   0x3820[2] ISP Vertical flip
	 *   0x3820[1] Sensor Vertical flip
	 *
	 *   0x3821[2] ISP Horizontal mirror
	 *   0x3821[1] Sensor Horizontal mirror
	 *
	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/
}
#endif
/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function */
}				/*      night_mode      */

static void sensor_init(void)
{
//[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2019-02-21] enable the burst i2c write mode
	kal_uint32 len = 0;
	LOG_DEBUG("[OV16885] init_setting start\n");
	len = sizeof(Sensor_Init_Reg_OV16885) / sizeof(Sensor_Init_Reg_OV16885[0]);
	OV16885_write_cmos_sensor_burst(Sensor_Init_Reg_OV16885, len, imgsensor.i2c_write_id);
//[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2019-02-21] enable the burst i2c write mode
}

static void preview_setting(void)
{
	LOG_DEBUG("[OV16885] preview_setting start\n");
	
	// @@2336x1752_30fps_bin
	// Sysclk 160Mhz, MIPI4_768Mbps/Lane, 30Fps.
	// Line_length =1400, Frame_length =3808
	//write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x301 , 0xa5);
	write_cmos_sensor(0x304 , 0x28);
	write_cmos_sensor(0x316 , 0xa0);
	write_cmos_sensor(0x3501, 0xec);
	write_cmos_sensor(0x3511, 0xec);
	write_cmos_sensor(0x3600, 0x40);
	write_cmos_sensor(0x3602, 0x82);
	write_cmos_sensor(0x3621, 0x66);
	write_cmos_sensor(0x366c, 0x3 );
	write_cmos_sensor(0x3701, 0x14);
	write_cmos_sensor(0x3709, 0x38);
	write_cmos_sensor(0x3726, 0x21);
	write_cmos_sensor(0x3800, 0x0 );
	write_cmos_sensor(0x3801, 0x0 );
	write_cmos_sensor(0x3802, 0x0 );
	write_cmos_sensor(0x3803, 0x0 );
	write_cmos_sensor(0x3804, 0x12);
	write_cmos_sensor(0x3805, 0x5f);
	write_cmos_sensor(0x3806, 0xd );
	write_cmos_sensor(0x3807, 0xcf);
	write_cmos_sensor(0x3808, 0x9 );
	write_cmos_sensor(0x3809, 0x20);
	write_cmos_sensor(0x380a, 0x6 );
	write_cmos_sensor(0x380b, 0xd8);
	write_cmos_sensor(0x380c, 0x5 );
	write_cmos_sensor(0x380d, 0x78);
	write_cmos_sensor(0x380e, 0xe );
	write_cmos_sensor(0x380f, 0xe0);
	write_cmos_sensor(0x3811, 0x9 );
	write_cmos_sensor(0x3813, 0x4 );
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x1 );
	write_cmos_sensor(0x3834, 0xf4);
	write_cmos_sensor(0x3f03, 0x1a);
	write_cmos_sensor(0x3f05, 0x67);
	write_cmos_sensor(0x4013, 0x14);
	write_cmos_sensor(0x4014, 0x8 );
	write_cmos_sensor(0x4016, 0x11);
	write_cmos_sensor(0x4018, 0x7 );
	write_cmos_sensor(0x4500, 0x45);
	write_cmos_sensor(0x4501, 0x1 );
	write_cmos_sensor(0x4503, 0x31);
	write_cmos_sensor(0x4837, 0x14);
	write_cmos_sensor(0x5000, 0xa1);
	write_cmos_sensor(0x5001, 0x46);
	write_cmos_sensor(0x583e, 0x5 );
	//write_cmos_sensor(0x0100, 0x01);
}				/*      preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_DEBUG("[OV16885] capture_setting start\n");
		if (0) {
	//write_cmos_sensor(0x0100, 0x00);	
	write_cmos_sensor(0x301 , 0xe5);
	write_cmos_sensor(0x304 , 0x4b);
	write_cmos_sensor(0x316 , 0x50);
	write_cmos_sensor(0x3501, 0xec);
	write_cmos_sensor(0x3511, 0xec);
	write_cmos_sensor(0x3600, 0x0 );
	write_cmos_sensor(0x3602, 0x86);
	write_cmos_sensor(0x3621, 0x88);
	write_cmos_sensor(0x366c, 0x53);
	write_cmos_sensor(0x3701, 0xe );
	write_cmos_sensor(0x3709, 0x30);
	write_cmos_sensor(0x3726, 0x20);
	write_cmos_sensor(0x3800, 0x0 );
	write_cmos_sensor(0x3801, 0x0 );
	write_cmos_sensor(0x3802, 0x0 );
	write_cmos_sensor(0x3803, 0x0 );
	write_cmos_sensor(0x3804, 0x12);
	write_cmos_sensor(0x3805, 0x5f);
	write_cmos_sensor(0x3806, 0xd );
	write_cmos_sensor(0x3807, 0xcf);
	write_cmos_sensor(0x3808, 0x12);
	write_cmos_sensor(0x3809, 0x40);
	write_cmos_sensor(0x380a, 0xd );
	write_cmos_sensor(0x380b, 0xb0);
	write_cmos_sensor(0x380c, 0x5 );
	write_cmos_sensor(0x380d, 0x78);
	write_cmos_sensor(0x380e, 0xe );
	write_cmos_sensor(0x380f, 0xe0);
	write_cmos_sensor(0x3811, 0x11);
	write_cmos_sensor(0x3813, 0x8 );
	write_cmos_sensor(0x3815, 0x11);
	write_cmos_sensor(0x3820, 0x0 );
	write_cmos_sensor(0x3834, 0xf0);
	write_cmos_sensor(0x3f03, 0x10);
	write_cmos_sensor(0x3f05, 0x66);
	write_cmos_sensor(0x4013, 0x28);
	write_cmos_sensor(0x4014, 0x10);
	write_cmos_sensor(0x4016, 0x25);
	write_cmos_sensor(0x4018, 0xf );
	write_cmos_sensor(0x4500, 0x0 );
	write_cmos_sensor(0x4501, 0x5 );
	write_cmos_sensor(0x4503, 0x31);
	write_cmos_sensor(0x4837, 0x16);
	write_cmos_sensor(0x5000, 0x83);
	write_cmos_sensor(0x5001, 0x52);
	write_cmos_sensor(0x583e, 0x3 );
	//write_cmos_sensor(0x0100, 0x01);	
	return;
	}else if (currefps == 300) {
	//write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x301 , 0xa5);
	//write_cmos_sensor(0x301 , 0xe5);//stone mod
	write_cmos_sensor(0x304 , 0x4b);
	write_cmos_sensor(0x316 , 0xa0);
	//write_cmos_sensor(0x316 , 0x50);//stone mod
	write_cmos_sensor(0x3501, 0xec);
	write_cmos_sensor(0x3511, 0xec);
	write_cmos_sensor(0x3600, 0x0 );
	write_cmos_sensor(0x3602, 0x86);
	write_cmos_sensor(0x3621, 0x88);
	write_cmos_sensor(0x366c, 0x53);
	write_cmos_sensor(0x3701, 0xe );
	write_cmos_sensor(0x3709, 0x30);
	write_cmos_sensor(0x3726, 0x20);
	write_cmos_sensor(0x3800, 0x0 );
	write_cmos_sensor(0x3801, 0x0 );
	write_cmos_sensor(0x3802, 0x0 );
	write_cmos_sensor(0x3803, 0x0 );
	write_cmos_sensor(0x3804, 0x12);
	write_cmos_sensor(0x3805, 0x5f);
	write_cmos_sensor(0x3806, 0xd );
	write_cmos_sensor(0x3807, 0xcf);
	write_cmos_sensor(0x3808, 0x12);
	write_cmos_sensor(0x3809, 0x40);
	write_cmos_sensor(0x380a, 0xd );
	write_cmos_sensor(0x380b, 0xb0);
	write_cmos_sensor(0x380c, 0x5 );
	write_cmos_sensor(0x380d, 0x78);
	write_cmos_sensor(0x380e, 0xe );
	write_cmos_sensor(0x380f, 0xe0);
	//write_cmos_sensor(0x380e, 0x0e);//stone mod
	//write_cmos_sensor(0x380f, 0x10);//stone mod
	write_cmos_sensor(0x3811, 0x11);
	write_cmos_sensor(0x3813, 0x8 );
	write_cmos_sensor(0x3815, 0x11);
	write_cmos_sensor(0x3820, 0x0 );
	write_cmos_sensor(0x3834, 0xf0);
	write_cmos_sensor(0x3f03, 0x10);
	write_cmos_sensor(0x3f05, 0x66);
	write_cmos_sensor(0x4013, 0x28);
	write_cmos_sensor(0x4014, 0x10);
	write_cmos_sensor(0x4016, 0x25);
	write_cmos_sensor(0x4018, 0xf );
	write_cmos_sensor(0x4500, 0x0 );
	write_cmos_sensor(0x4501, 0x5 );
	write_cmos_sensor(0x4503, 0x31);
	write_cmos_sensor(0x4837, 0xb );
	write_cmos_sensor(0x5000, 0x83);
	write_cmos_sensor(0x5001, 0x52);
	write_cmos_sensor(0x583e, 0x3 );
	//write_cmos_sensor(0x0100, 0x01);
	return;
	}
	/*write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x301 , 0xe5); 
	write_cmos_sensor(0x304 , 0x4b); 
	write_cmos_sensor(0x316 , 0x50); 
	write_cmos_sensor(0x3600, 0x0 ); 
	write_cmos_sensor(0x3602, 0x86); 
	write_cmos_sensor(0x3621, 0x88); 
	write_cmos_sensor(0x366c, 0x53); 
	write_cmos_sensor(0x3701, 0xe ); 
	write_cmos_sensor(0x3709, 0x30); 
	write_cmos_sensor(0x3726, 0x20); 
	write_cmos_sensor(0x3808, 0x12); 
	write_cmos_sensor(0x3809, 0x40); 
	write_cmos_sensor(0x380a, 0xd ); 
	write_cmos_sensor(0x380b, 0xb0); 
	write_cmos_sensor(0x3811, 0x11); 
	write_cmos_sensor(0x3813, 0x8 ); 
	write_cmos_sensor(0x3815, 0x11); 
	write_cmos_sensor(0x3820, 0x0 ); 
	write_cmos_sensor(0x3834, 0xf0); 
	write_cmos_sensor(0x3f03, 0x10); 
	write_cmos_sensor(0x3f05, 0x66); 
	write_cmos_sensor(0x4013, 0x28); 
	write_cmos_sensor(0x4014, 0x10); 
	write_cmos_sensor(0x4016, 0x25); 
	write_cmos_sensor(0x4018, 0xf ); 
	write_cmos_sensor(0x4500, 0x0 ); 
	write_cmos_sensor(0x4501, 0x5 ); 
	write_cmos_sensor(0x4837, 0x16); 
	write_cmos_sensor(0x5000, 0x83); 
	write_cmos_sensor(0x5001, 0x52); 
	write_cmos_sensor(0x583e, 0x3 );
	write_cmos_sensor(0x0100, 0x01);*/
	return;
}
/*
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_DEBUG("E! currefps:%d\n", currefps);
	LOG_DEBUG("E! video just has 30fps preview size setting ,NOT HAS 24FPS SETTING!\n");
	capture_setting(30);

}
*/
static void hs_video_setting(void)
{
  	//write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x301 , 0xa5);
	write_cmos_sensor(0x304 , 0x1e);
	write_cmos_sensor(0x316 , 0xa0);
	write_cmos_sensor(0x3501, 0x34);
	write_cmos_sensor(0x3511, 0x34);
	write_cmos_sensor(0x3600, 0x0 );
	write_cmos_sensor(0x3602, 0x82);
	write_cmos_sensor(0x3621, 0x88);
	write_cmos_sensor(0x366c, 0x3 );
	write_cmos_sensor(0x3701, 0xe );
	write_cmos_sensor(0x3709, 0x30);
	write_cmos_sensor(0x3726, 0x21);
	write_cmos_sensor(0x3800, 0x4 );
	write_cmos_sensor(0x3801, 0x28);
	write_cmos_sensor(0x3802, 0x4 );
	write_cmos_sensor(0x3803, 0x10);
	write_cmos_sensor(0x3804, 0xe );
	write_cmos_sensor(0x3805, 0x37);
	write_cmos_sensor(0x3806, 0x9 );
	write_cmos_sensor(0x3807, 0xbf);
	write_cmos_sensor(0x3808, 0x5 );
	write_cmos_sensor(0x3809, 0x0 );
	write_cmos_sensor(0x380a, 0x2 );
	write_cmos_sensor(0x380b, 0xd0);
	write_cmos_sensor(0x380c, 0x4 );
	write_cmos_sensor(0x380d, 0x10);
	write_cmos_sensor(0x380e, 0x05);
	write_cmos_sensor(0x380f, 0x02);
	write_cmos_sensor(0x3811, 0x7 );
	write_cmos_sensor(0x3813, 0x4 );
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x1 );
	write_cmos_sensor(0x3834, 0xf0);
	write_cmos_sensor(0x3f03, 0x10);
	write_cmos_sensor(0x3f05, 0x29);
	write_cmos_sensor(0x4013, 0x14);
	write_cmos_sensor(0x4014, 0x8 );
	write_cmos_sensor(0x4016, 0x11);
	write_cmos_sensor(0x4018, 0x7 );
	write_cmos_sensor(0x4500, 0x40);
	write_cmos_sensor(0x4501, 0x1 );
	write_cmos_sensor(0x4503, 0x15);
	write_cmos_sensor(0x4837, 0x1b);
	write_cmos_sensor(0x5000, 0xa1);
	write_cmos_sensor(0x5001, 0x46);
	write_cmos_sensor(0x583e, 0x5 );
	//write_cmos_sensor(0x0100, 0x01);

}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}

static void custom1_setting(kal_uint16 currefps)
{
	LOG_DEBUG("[OV16885] custom1_setting start\n");
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x301 , 0xa5);
	write_cmos_sensor(0x304 , 0x3c);	//0x4b for 30fps
	write_cmos_sensor(0x316 , 0x80);	//0xa0 for 30fps
	write_cmos_sensor(0x3501, 0xee);
	write_cmos_sensor(0x3502, 0x10);
	write_cmos_sensor(0x3511, 0xec);
	write_cmos_sensor(0x3600, 0x0 );
	write_cmos_sensor(0x3602, 0x86);
	write_cmos_sensor(0x3621, 0x88);
	write_cmos_sensor(0x366c, 0x53);
	write_cmos_sensor(0x3701, 0xe );
	write_cmos_sensor(0x3709, 0x30);
	write_cmos_sensor(0x3726, 0x20);
	write_cmos_sensor(0x3800, 0x0 );
	write_cmos_sensor(0x3801, 0x0 );
	write_cmos_sensor(0x3802, 0x0 );
	write_cmos_sensor(0x3803, 0x0 );
	write_cmos_sensor(0x3804, 0x12);
	write_cmos_sensor(0x3805, 0x5f);
	write_cmos_sensor(0x3806, 0xd );
	write_cmos_sensor(0x3807, 0xcf);
	write_cmos_sensor(0x3808, 0x12);
	write_cmos_sensor(0x3809, 0x40);
	write_cmos_sensor(0x380a, 0xd );
	write_cmos_sensor(0x380b, 0xb0);
	write_cmos_sensor(0x380c, 0x5 );
	write_cmos_sensor(0x380d, 0x78);
	write_cmos_sensor(0x380e, 0xe);
	write_cmos_sensor(0x380f, 0xe0);
	write_cmos_sensor(0x3811, 0x11);
	write_cmos_sensor(0x3813, 0x8 );
	write_cmos_sensor(0x3815, 0x11);
	write_cmos_sensor(0x3820, 0x0 );
	write_cmos_sensor(0x3834, 0xf0);
	write_cmos_sensor(0x3f03, 0x10);
	write_cmos_sensor(0x3f05, 0x66);
	write_cmos_sensor(0x4013, 0x28);
	write_cmos_sensor(0x4014, 0x10);
	write_cmos_sensor(0x4016, 0x25);
	write_cmos_sensor(0x4018, 0xf );
	write_cmos_sensor(0x4500, 0x0 );
	write_cmos_sensor(0x4501, 0x5 );
	write_cmos_sensor(0x4503, 0x31);
	write_cmos_sensor(0x4837, 0xd );	//0x0b for 30fps
	write_cmos_sensor(0x5000, 0x83);
	write_cmos_sensor(0x5001, 0x52);
	write_cmos_sensor(0x583e, 0x3 );
}

/*#ifdef CONFIG_HQ_HARDWARE_INFO
#define DEBUG 0
static void get_eeprom_data(EEPROM_DATA *data)
{
	kal_uint8 i =0x0;
	u8 *otp_data = (u8 *)data;

	for (;i <= 0xE; i++, otp_data++)
		*otp_data = read_eeprom_module(i);

#if DEBUG
	otp_data = (u8 *)data;
	for (i=0;i<=0xE;i++)
		pr_err(" otpdata[0x%x]=0x%x    ", i, *(otp_data + i));
#endif
	return ;
}
#endif*/

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static kal_uint32 return_sensor_id(void)
{
	return (((read_cmos_sensor(0x300a) << 16) | (read_cmos_sensor(0x300b) << 8) |
		 read_cmos_sensor(0x300c)) & 0xFFFFFF);
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			/* lx_revised */
			LOG_DEBUG("[ov16885]Read sensor id OK, write id:0x%x ,sensor Id:0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
                strcpy(rear_sensor_name, "ov16885_qtech");/*LGE_CHANGE, 2019-07-04, add the camera identifying logic , kyunghun.oh@lge.com*/
				LOG_DEBUG("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				/*#ifdef CONFIG_HQ_HARDWARE_INFO
					get_eeprom_data(&pOtp_data);
					hw_info_main_otp.otp_valid = pOtp_data.vaild_flag;
					hw_info_main_otp.vendor_id = pOtp_data.vendor_id;
					hw_info_main_otp.module_code = pOtp_data.module_code;
					hw_info_main_otp.module_ver = pOtp_data.module_ver;
					hw_info_main_otp.sw_ver = pOtp_data.sw_ver;
					hw_info_main_otp.year = pOtp_data.year;
					hw_info_main_otp.month = pOtp_data.month;
					hw_info_main_otp.day = pOtp_data.day;
					hw_info_main_otp.vcm_vendorid = pOtp_data.vcm_id;
					hw_info_main_otp.vcm_moduleid = pOtp_data.vcm_id;
				#endif*/

				return ERROR_NONE;
			}
			LOG_DEBUG("Read sensor id OK:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,
				*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	LOG_DEBUG("park Read sensor sensor_id fail, id: 0x%x\n", *sensor_id);
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	/* const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2}; */
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	LOG_DEBUG("[OV16885] open sensor\n");

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_DEBUG("[16885]i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_DEBUG("Read sensor id fail, id: 0x%x,sensor_id =0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	preview_setting();
	write_cmos_sensor(0x0100, 0x01);

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*      open  */



/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_DEBUG("[OV16885] close sensor\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*      close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("[OV16885] preview start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor.current_fps;
/* imgsensor.autoflicker_en = KAL_FALSE; */
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	
 	//mdelay(10); 
	set_mirror_flip(imgsensor.mirror); ////GIONEE:malp modify
	return ERROR_NONE;
}				/*      preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("[OV16885] Capture start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 16m */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		/* imgsensor.autoflicker_en = KAL_FALSE; */

	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_DEBUG
			    ("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
			     imgsensor_info.cap1.max_framerate / 10,
			     imgsensor_info.cap.max_framerate);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		/* imgsensor.autoflicker_en = KAL_FALSE; */
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);
	
	set_mirror_flip(imgsensor.mirror); ////GIONEE:malp modify

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("[OV16885] NORMAL_VIDEO Start\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	/* imgsensor.autoflicker_en = KAL_FALSE; */
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps); //[LGE_UPDATE] [kyunghun.oh@lge.com] [2019-01-28] make the fullsize recording works
	
	set_mirror_flip(imgsensor.mirror); ////GIONEE:malp modify

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();

	//mdelay(10); 
	set_mirror_flip(imgsensor.mirror); ////GIONEE:malp modify

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	//mdelay(10); 
	set_mirror_flip(imgsensor.mirror); ////GIONEE:malp modify

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("[OV16885] custom1 setting\n");
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	//imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting(imgsensor.current_fps);
	
	set_mirror_flip(imgsensor.mirror); ////GIONEE:malp modify
	return ERROR_NONE;
}/* custom1() */


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;
	
	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
	
	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d %d\n", scenario_id, sensor_info->SensorOutputDataFormat);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;	/* The frame of setting
										 * shutter default 0 for TG int
										 */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting
												 * sensor gain
												 */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */
	sensor_info->PDAF_Support = 1;	//0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	default:
		LOG_DEBUG("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)	/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	/* kal_int16 dummyLine; */
	kal_uint32 frameHeight;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frameHeight =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		LOG_INF("frameHeight = %d\n", frameHeight);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)
		    ? (frameHeight - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frameHeight =
		    imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >  imgsensor_info.normal_video.framelength)
			? (frameHeight - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frameHeight =
		    imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);

		imgsensor.dummy_line = (frameHeight > imgsensor_info.cap.framelength)
		    ? (frameHeight - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frameHeight =
		    imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight >  imgsensor_info.hs_video.framelength)
		    ? (frameHeight - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frameHeight =
		    imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.slim_video.framelength)
			? (frameHeight - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		printk("KYUNGHUN_SCENARIO_ID_CUSTOM1\n");
		frameHeight = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.custom1.framelength) ? (frameHeight - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	default:		/* coding with  preview scenario by default */
		frameHeight =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frameHeight > imgsensor_info.pre.framelength)
			? (frameHeight - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		LOG_DEBUG("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x5081, 0x01);
		write_cmos_sensor(0x5000, 0x01);
		write_cmos_sensor(0x5001 , 0x44);
		//write_cmos_sensor(0x5001,(read_cmos_sensor(0x5000)&0xfd));
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x5081, 0x00);
		write_cmos_sensor(0x5000 , 0xa1);
		write_cmos_sensor(0x5001 , 0x46);
		//write_cmos_sensor(0x5000,(read_cmos_sensor(0x5000)&0xa1)|0x040 );
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

#define EEPROM_READ_ID  0xA0
static void read_eeprom(int offset, char *data, kal_uint32 size)
{
	int i = 0, addr = offset;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	for (i = 0; i < size; i++) {
		pu_send_cmd[0] = (char)(addr >> 8);
		pu_send_cmd[1] = (char)(addr & 0xFF);
		iReadRegI2C(pu_send_cmd, 2, &data[i], 1, EEPROM_READ_ID);

		addr++;
	}
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	kal_uint32 rate;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/* LOG_INF("feature_id = %d\n", feature_id); */
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (UINT8) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32) *feature_data);
		PDAFinfo = (SET_PD_BLOCK_INFO_T *) (uintptr_t) (*(feature_data + 1));
	
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				   sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n", *feature_data);
		/* PDAF capacity enable or not, OV16885 only full size support PDAF */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_DEBUG("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		LOG_INF("SENSOR_FEATURE_GET_MIPI_PIXEL_RATE\n");
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				 rate =	imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
#if 0
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
#endif
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				rate = imgsensor_info.pre.mipi_pixel_rate;
			default:
				rate = 0;
				break;
			}
		LOG_INF("SENSOR_FEATURE_GET_MIPI_PIXEL_RATE wangweifeng:rate%d\n",rate);
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;	
	}
	break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32) *feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16) *feature_data,
			(UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data, (UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_DEBUG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_DEBUG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)((feature_data+1));

		if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {/*only copy Cross Talk calibration data*/
			read_eeprom(0x763, data, 600+2);
			LOG_INF("read Cross Talk calibration data size= %d %d\n", data[0], data[1]);
		} else if (type == FOUR_CELL_CAL_TYPE_DPC) {
			read_eeprom(0x9BE, data, 832+2);
			LOG_INF("read DPC calibration data size= %d %d\n", data[0], data[1]);
		}
		break;
	}
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME://lzl
		set_shutter_frame_length((UINT16)*feature_data,(UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_PIXEL_RATE: {
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;
		break;

		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom1.pclk /
			(imgsensor_info.custom1.linelength - 80))*
			imgsensor_info.custom1.grabwindow_width;
		break;
		
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
		(imgsensor_info.normal_video.pclk /
		(imgsensor_info.normal_video.linelength - 80))*
		imgsensor_info.normal_video.grabwindow_width;
		}
	}
	break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV16885_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*      OV5693_MIPI_RAW_SensorInit      */
