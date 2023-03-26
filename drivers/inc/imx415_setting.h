/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_IMX415_SETTING_H_
#define UTILITY_SENSOR_INC_IMX415_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

#define IMX415_SHR0 (0x3050)
#define IMX415_SHR1 (0x3054)
#define IMX415_SHR2 (0x3058)
#define IMX415_SHR3 (0x305C)

#define IMX415_RHS1 (0x3060) // Readout timing setting of SEF1 Designated in line units.
#define IMX415_RHS2 (0x3064) // Readout timing setting of SEF2 Designated in line units. 

#define GAIN_PCG0   (0x3090)
#define GAIN_PCG1   (0x3092)
#define GAIN_PCG2   (0x3094)
#define GAIN_PCG3   (0x3094)


#define IMX415_VAMX (0x3024)
#define IMX415_HAMX (0x3028)

#define PIX_HST (0x3040) 
#define PIX_VST (0x3044) 

#define PIX_HWIDTH (0x3042) //  PIX_HWIDTH
#define PIX_VWIDTH (0x3046) //  PIX_VWIDTH

#define IMX415_PARAM_HOLD  	(0x3001)
#define IMX415_LINE      	(0x3050)
#define IMX415_GAIN         (0x3090)
#define IMX415_DGAIN		(0x3090)

#define IMX415_CSI_LANE_MODE (0x4001)  // 1:CSI-2 2lane, 3:CSI-2 4lane

// #define IMX415_VTS	       	(0x0340)
// #define IMX415_HTS	        (0x0342)
// #define IMX415_X_SIZE   	(0x034C)
// #define IMX415_Y_SIZE  		(0x034E)
// #define DEFAULT_FPS_REGVALUE	(3064)
// #define DEFAULT_FPS				(30)


#define IMX415_REG_CTRL_MODE 0x3000
#define IMX415_MODE_SW_STANDBY 0x1
#define IMX415_MODE_STREAMING 0x0

#define IMX415_REG_MASTER_MODE 0x3002
#define IMX415_MASTER_MODE_START 0x0
#define IMX415_MASTER_MODE_STOP 0x1


static uint16_t imx415_stream_on_setting[] = {
        0x3000, 0x0,
        0x3002, 0x0
};

static uint16_t imx415_stream_off_setting[] = {
        0x3000, 0x1,
        0x3002, 0x1
};

static uint32_t imx415_gain_lut[] = {
	0x0,
	0x1,
	0x2,
	0x2,
	0x3,
	0x4,
	0x4,
	0x5,
	0x6,
	0x6,
	0x7,
	0x7,
	0x8,
	0x9,
	0x9,
	0xA,
	0xB,
	0xB,
	0xC,
	0xC,
	0xD,
	0xE,
	0xE,
	0xF,
	0x10,
	0x10,
	0x11,
	0x11,
	0x12,
	0x13,
	0x13,
	0x14,
	0x15,
	0x15,
	0x16,
	0x16,
	0x17,
	0x18,
	0x18,
	0x19,
	0x1A,
	0x1A,
	0x1B,
	0x1B,
	0x1C,
	0x1D,
	0x1D,
	0x1E,
	0x1F,
	0x1F,
	0x20,
	0x20,
	0x21,
	0x22,
	0x22,
	0x23,
	0x24,
	0x24,
	0x25,
	0x26,
	0x26,
	0x27,
	0x27,
	0x28,
	0x29,
	0x29,
	0x2A,
	0x2B,
	0x2B,
	0x2C,
	0x2C,
	0x2D,
	0x2E,
	0x2E,
	0x2F,
	0x30,
	0x30,
	0x31,
	0x31,
	0x32,
	0x33,
	0x33,
	0x34,
	0x35,
	0x35,
	0x36,
	0x36,
	0x37,
	0x38,
	0x38,
	0x39,
	0x3A,
	0x3A,
	0x3B,
	0x3B,
	0x3C,
	0x3D,
	0x3D,
	0x3E,
	0x3F,
	0x3F,
	0x40,
	0x40,
	0x41,
	0x42,
	0x42,
	0x43,
	0x44,
	0x44,
	0x45,
	0x45,
	0x46,
	0x47,
	0x47,
	0x48,
	0x49,
	0x49,
	0x4A,
	0x4B,
	0x4B,
	0x4C,
	0x4C,
	0x4D,
	0x4E,
	0x4E,
	0x4F,
	0x50,
	0x50,
	0x51,
	0x51,
	0x52,
	0x53,
	0x53,
	0x54,
	0x55,
	0x55,
	0x56,
	0x56,
	0x57,
	0x58,
	0x58,
	0x59,
	0x5A,
	0x5A,
	0x5B,
	0x5B,
	0x5C,
	0x5D,
	0x5D,
	0x5E,
	0x5F,
	0x5F,
	0x60,
	0x60,
	0x61,
	0x62,
	0x62,
	0x63,
	0x64,
	0x64,
	0x65,
	0x65,
	0x66,
	0x67,
	0x67,
	0x68,
	0x69,
	0x69,
	0x6A,
	0x6A,
	0x6B,
	0x6C,
	0x6C,
	0x6D,
	0x6E,
	0x6E,
	0x6F,
	0x70,
	0x70,
	0x71,
	0x71,
	0x72,
	0x73,
	0x73,
	0x74,
	0x75,
	0x75,
	0x76,
	0x76,
	0x77,
	0x78,
	0x78,
	0x79,
	0x7A,
	0x7A,
	0x7B,
	0x7B,
	0x7C,
	0x7D,
	0x7D,
	0x7E,
	0x7F,
	0x7F,
	0x80,
	0x80,
	0x81,
	0x82,
	0x82,
	0x83,
	0x84,
	0x84,
	0x85,
	0x85,
	0x86,
	0x87,
	0x87,
	0x88,
	0x89,
	0x89,
	0x8A,
	0x8A,
	0x8B,
	0x8C,
	0x8C,
	0x8D,
	0x8E,
	0x8E,
	0x8F,
	0x8F,
	0x90,
	0x91,
	0x91,
	0x92,
	0x93,
	0x93,
	0x94,
	0x95,
	0x95,
	0x96,
	0x96,
	0x97,
	0x98,
	0x98,
	0x99,
	0x9A,
	0x9A,
	0x9B,
	0x9B,
	0x9C,
	0x9D,
	0x9D,
	0x9E,
	0x9F,
	0x9F,
	0xA0,
	0xA0,
};

static uint16_t imx415_init_3840x2160_linear_setting[] = {
	// MCLK=24Mhz MIPI 4lane 3840*2160 30FPS 10bit
	0x3000, 0x01, // standby
	0x3002, 0x01, // Master mode stop
	0x3008, 0x54,
	0x300A, 0x3B,
	0x301C, 0x04, // set to window mode
	0x3024, 0xCA,
	0x3025, 0x08, //Vmax to 2700
	0x3026, 0x00, //Vmax to 2700
	0x3028, 0x2A,
	0x3029, 0x04, //Hmax to 4400
	0x3031, 0x00,
	0x3032, 0x00,
	0X3033, 0x09, // SYS_MODE

	// 0X3042, 0x00, // width 3840
	// 0X3046, 0xE0, // width 3840
	// 0X3047, 0x10, // width 3840
	0x3040, 0x00, // PIX_HST
	0x3041, 0x00, // PIX_HST
	0x3042, 0x00, // PIX_HWIDTH 0xF00=3840
	0x3043, 0x0F, // PIX_HWIDTH 
	0x3044, 0x28, // PIX_VST ## 0x28=40
	0x3045, 0x00, // PIX_VST
	0x3046, 0xE0, // PIX_VWIDTH 0x870=2160
	0x3047, 0x10, // PIX_VWIDTH

	0X3050, 0x08,
	0x30C1, 0x00,
	0x3116, 0x23,
	0x3118, 0xB4,
	0x311A, 0xFC,
	0x311E, 0x23,
	0x32D4, 0x21,
	0x32EC, 0xA1,
	0x3452, 0x7F,
	0x3453, 0x03,
	0x358A, 0x04,
	0x35A1, 0x02,
	0x36BC, 0x0C,
	0x36CC, 0x53,
	0x36CD, 0x00,
	0x36CE, 0x3C,
	0x36D0, 0x8C,
	0x36D1, 0x00,
	0x36D2, 0x71,
	0x36D4, 0x3C,
	0x36D6, 0x53,
	0x36D7, 0x00,
	0x36D8, 0x71,
	0x36DA, 0x8C,
	0x36DB, 0x00,
	0x3701, 0x00,
	0x3724, 0x02,
	0x3726, 0x02,
	0x3732, 0x02,
	0x3734, 0x03,
	0x3736, 0x03,
	0x3742, 0x03,
	0x3862, 0xE0,
	0x38CC, 0x30,
	0x38CD, 0x2F,
	0x395C, 0x0C,
	0x3A42, 0xD1,
	0x3A4C, 0x77,
	0x3AE0, 0x02,
	0x3AEC, 0x0C,
	0x3B00, 0x2E,
	0x3B06, 0x29,
	0x3B98, 0x25,
	0x3B99, 0x21,
	0x3B9B, 0x13,
	0x3B9C, 0x13,
	0x3B9D, 0x13,
	0x3B9E, 0x13,
	0x3BA1, 0x00,
	0x3BA2, 0x06,
	0x3BA3, 0x0B,
	0x3BA4, 0x10,
	0x3BA5, 0x14,
	0x3BA6, 0x18,
	0x3BA7, 0x1A,
	0x3BA8, 0x1A,
	0x3BA9, 0x1A,
	0x3BAC, 0xED,
	0x3BAD, 0x01,
	0x3BAE, 0xF6,
	0x3BAF, 0x02,
	0x3BB0, 0xA2,
	0x3BB1, 0x03,
	0x3BB2, 0xE0,
	0x3BB3, 0x03,
	0x3BB4, 0xE0,
	0x3BB5, 0x03,
	0x3BB6, 0xE0,
	0x3BB7, 0x03,
	0x3BB8, 0xE0,
	0x3BBA, 0xE0,
	0x3BBC, 0xDA,
	0x3BBE, 0x88,
	0x3BC0, 0x44,
	0x3BC2, 0x7B,
	0x3BC4, 0xA2,
	0x3BC8, 0xBD,
	0x3BCA, 0xBD,
	0x4004, 0x00,
	0x4005, 0x06,
	0x400C, 0x00,
	0x4018, 0x6F,
	0x401A, 0x2F,
	0x401C, 0x2F,
	0x401E, 0xBF,
	0x401F, 0x00,
	0x4020, 0x2F,
	0x4022, 0x57,
	0x4024, 0x2F,
	0x4026, 0x4F,
	0x4028, 0x27,
	0x4074, 0x01,
	// SEN_CMD_SETVD, 0x0,
	// SEN_CMD_PRESET, 0x0,
	// SEN_CMD_DIRECTION, 0x0,
	0x3000, 0x1,  // standby cancel
	// SEN_CMD_DELAY, 25,
	0x3002, 0x1,  //Master mode start
};

static uint16_t imx415_init_3840x2160_60_fps_linear_setting[] = {
	// MCLK=24Mhz MIPI 4lane 3840*2160 30FPS 10bit
	//0x3000, 0x01, // standby
	//0x3002, 0x01, // Master mode stop
	0x3008, 0x54,
	0x300A, 0x3B,
	//vhd60fps
	0x301C, 0x04, // set to window mode
	0x3024, 0xCB,
	//0x3025, 0x08, //Vmax to 2700
	//0x3026, 0x00, //Vmax to 2700
	0x3028, 0x15,
	//0x3029, 0x02, //Hmax to 4400
	0x3031, 0x00,
	0x3032, 0x00,
	0X3033, 0x08, // SYS_MODE

	// 0X3042, 0x00, // width 3840
	// 0X3046, 0xE0, // width 3840
	// 0X3047, 0x10, // width 3840

	//vhd60fps
	0x3040, 0x0C, // PIX_HST
	//0x3041, 0x00, // PIX_HST
	0x3042, 0x00, // PIX_HWIDTH 0xF00=3840
	//0x3043, 0x0F, // PIX_HWIDTH
	0x3044, 0x20, // PIX_VST ## 0x28=40
	0x3045, 0x00, // PIX_VST
	0x3046, 0xE0, // PIX_VWIDTH 0x870=2160
	0x3047, 0x10, // PIX_VWIDTH

	0x3050, 0x85,
	0x3051, 0x03,
	0x3090, 0x14,
	//vhd60fps
	//0x3081, 0x02,
	0x30C1, 0x00,
	0x3116, 0x23,
	0x3118, 0xB4,
	0x311A, 0xFC,
	0x311E, 0x23,
	0x32D4, 0x21,
	0x32EC, 0xA1,
	0x3452, 0x7F,
	0x344C, 0x2B,
	0x344D, 0x01,
	0x344E, 0xED,
	0x344F, 0x01,
	0x3450, 0xF6,
	0x3451, 0x02,
	0x3452, 0x7F,
	0x3453, 0x03,
	0x358A, 0x04,
	0x35A1, 0x02,
	0x35EC, 0x27,
	0x35EE, 0x8D,
	0x35F0, 0x8D,
	0x35F2, 0x29,
	0x36BC, 0x0C,
	0x36CC, 0x53,
	0x36CD, 0x00,
	0x36CE, 0x3C,
	0x36D0, 0x8C,
	0x36D1, 0x00,
	0x36D2, 0x71,
	0x36D4, 0x3C,
	0x36D6, 0x53,
	0x36D7, 0x00,
	0x36D8, 0x71,
	0x36DA, 0x8C,
	0x36DB, 0x00,
	0x3701, 0x00,
	0x3720, 0x00,
	0x3724, 0x02,
	0x3726, 0x02,
	0x3732, 0x02,
	0x3734, 0x03,
	0x3736, 0x03,
	0x3742, 0x03,
	0x3862, 0xE0,
	0x38CC, 0x30,
	0x38CD, 0x2F,
	0x395C, 0x0C,
	0x39A4, 0x07,
	0x39A8, 0x32,
	0x39AA, 0x32,
	0x39AC, 0x32,
	0x39AE, 0x32,
	0x39B0, 0x32,
	0x39B2, 0x2F,
	0x39B4, 0x2D,
	0x39B6, 0x28,
	0x39B8, 0x30,
	0x39BA, 0x30,
	0x39BC, 0x30,
	0x39BE, 0x30,
	0x39C0, 0x30,
	0x39C2, 0x2E,
	0x39C4, 0x2B,
	0x39C6, 0x25,
	0x3A42, 0xD1,
	0x3A4C, 0x77,
	0x3AE0, 0x02,
	0x3AEC, 0x0C,
	0x3B00, 0x2E,
	0x3B06, 0x29,
	0x3B98, 0x25,
	0x3B99, 0x21,
	0x3B9B, 0x13,
	0x3B9C, 0x13,
	0x3B9D, 0x13,
	0x3B9E, 0x13,
	0x3BA1, 0x00,
	0x3BA2, 0x06,
	0x3BA3, 0x0B,
	0x3BA4, 0x10,
	0x3BA5, 0x14,
	0x3BA6, 0x18,
	0x3BA7, 0x1A,
	0x3BA8, 0x1A,
	0x3BA9, 0x1A,
	0x3BAC, 0xED,
	0x3BAD, 0x01,
	0x3BAE, 0xF6,
	0x3BAF, 0x02,
	0x3BB0, 0xA2,
	0x3BB1, 0x03,
	0x3BB2, 0xE0,
	0x3BB3, 0x03,
	0x3BB4, 0xE0,
	0x3BB5, 0x03,
	0x3BB6, 0xE0,
	0x3BB7, 0x03,
	0x3BB8, 0xE0,
	0x3BBA, 0xE0,
	0x3BBC, 0xDA,
	0x3BBE, 0x88,
	0x3BC0, 0x44,
	0x3BC2, 0x7B,
	0x3BC4, 0xA2,
	0x3BC8, 0xBD,
	0x3BCA, 0xBD,
	0x4004, 0x00,
	0x4005, 0x06,
	//vhd60fps
	//0x400C, 0x00,
	0x4018, 0x9F,
	0x401A, 0x57,
	0x401C, 0x57,
	0x401E, 0x87,
	//vhd60fps
	//0x401F, 0x01,
	0x4020, 0x5F,
	0x4022, 0xa7,
	0x4024, 0x5F,
	0x4026, 0x97,
	0x4028, 0x4f,
	//0x4074, 0x01,
	// SEN_CMD_SETVD, 0x0,
	// SEN_CMD_PRESET, 0x0,
	// SEN_CMD_DIRECTION, 0x0,
	0x3000, 0x1,  // standby cancel
	// SEN_CMD_DELAY, 25,
	0x3002, 0x1,  //Master mode start
};

static uint16_t imx415_init_3840x2160_dol2_setting[] = {
	// MCLK=27Mhz MIPI 4lane 3840*2160 30.004FPS 10bit  LEF: 24.001ms,  SEF: 5.999ms
	0x3000, 0x01, // standby 
	0x3002, 0x01, // Master mode stop
	0x3008, 0x5D,
	0x300A, 0x42,
	0x301C, 0x04, // set to window mode
	0x3024,  0xCA,// 0xbe, // 0xCA, // 0x3E,
	0x3025,  0x08,// 0x08, // 0x08, // 0x0D, //Vmax to 2700
	0x3026, 0x00, //Vmax to 2700
	0x3028, 0x26,// 0x28, // 0x26, // 0x6D,
	0x3029, 0x02,// 0x02, // 0x02, // 0x01, //Hmax to 4400
	0x302C, 0x01,
	0x302D, 0x01,

	0x3031, 0x00,
	0x3032, 0x00,
	0X3033, 0x00, // SYS_MODE

	// 0X3042, 0x00, // width 3840
	// 0X3046, 0xE0, // width 3840
	// 0X3047, 0x10, // width 3840
	0x3040, 0x00, // PIX_HST
	0x3041, 0x00, // PIX_HST
	0x3042, 0x00, // PIX_HWIDTH 0xF00=3840
	0x3043, 0x0F, // PIX_HWIDTH 
	0x3044, 0x28, // PIX_VST ## 0x28=40
	0x3045, 0x00, // PIX_VST
	0x3046, 0xE0, // PIX_VWIDTH 0x870=2160
	0x3047, 0x10, // PIX_VWIDTH

	0X3050, 0x6A, //0x6A,
	0x3051, 0x07, //0x07,
	// 0x3052, 0x07, //0x07,
	0x3054, 0x09,
	// 0x3054, 0xCD, // exp
	// 0x3055, 0x04, // exp
	
	0x3060, 0xB9, //A9-B9-AF
	0x3061, 0x01, //01-01-01
	0x30C1, 0x00,
	0x30CF, 0x01,
	0x3116, 0x23,
	0x3118, 0x08,
	0x3119, 0x01,
	0x311A, 0xE7,
	0x311E, 0x23,
	0x32D4, 0x21,
	0x32EC, 0xA1,
	0x3452, 0x7F,
	0x3453, 0x03,
	0x358A, 0x04,//--
	0x35A1, 0x02,
	0x36BC, 0x0C,
	0x36CC, 0x53,
	0x36CD, 0x00,
	0x36CE, 0x3C,
	0x36D0, 0x8C,
	0x36D1, 0x00,
	0x36D2, 0x71,
	0x36D4, 0x3C,
	0x36D6, 0x53,
	0x36D7, 0x00,
	0x36D8, 0x71,
	0x36DA, 0x8C,
	0x36DB, 0x00,
	0x3701, 0x00,//--
	0x3724, 0x02,
	0x3726, 0x02,
	0x3732, 0x02,
	0x3734, 0x03,
	0x3736, 0x03,
	0x3742, 0x03,
	0x3862, 0xE0,
	0x38CC, 0x30,
	0x38CD, 0x2F,
	0x395C, 0x0C,
	0x3A42, 0xD1,
	0x3A4C, 0x77,
	0x3AE0, 0x02,
	0x3AEC, 0x0C,
	0x3B00, 0x2E,
	0x3B06, 0x29,
	0x3B98, 0x25,
	0x3B99, 0x21,
	0x3B9B, 0x13,
	0x3B9C, 0x13,
	0x3B9D, 0x13,
	0x3B9E, 0x13,
	0x3BA1, 0x00,//--
	0x3BA2, 0x06,
	0x3BA3, 0x0B,
	0x3BA4, 0x10,
	0x3BA5, 0x14,
	0x3BA6, 0x18,
	0x3BA7, 0x1A,
	0x3BA8, 0x1A,
	0x3BA9, 0x1A,
	0x3BAC, 0xED,
	0x3BAD, 0x01,
	0x3BAE, 0xF6,
	0x3BAF, 0x02,
	0x3BB0, 0xA2,
	0x3BB1, 0x03,
	0x3BB2, 0xE0,
	0x3BB3, 0x03,
	0x3BB4, 0xE0,
	0x3BB5, 0x03,
	0x3BB6, 0xE0,
	0x3BB7, 0x03,
	0x3BB8, 0xE0,
	0x3BBA, 0xE0,
	0x3BBC, 0xDA,
	0x3BBE, 0x88,
	0x3BC0, 0x44,
	0x3BC2, 0x7B,
	0x3BC4, 0xA2,
	0x3BC8, 0xBD,
	0x3BCA, 0xBD,//--
	0x4004, 0xC0,
	0x4005, 0x06,
	0x400C, 0x01,
	0x4018, 0xE7,
	0x401A, 0x8F,
	0x401C, 0x8F,
	0x401E, 0x7F,
	0x401F, 0x02,
	0x4020, 0x97,
	0x4022, 0x0F,
	0x4023, 0x01,
	0x4024, 0x97,
	0x4026, 0xF7,
	0x4028, 0x7F,
	0x4074, 0x00,
	// SEN_CMD_SETVD, 0x0,
	// SEN_CMD_PRESET, 0x0,
	// SEN_CMD_DIRECTION, 0x0,
	0x3000, 0x00,  // standby cancel
	// SEN_CMD_DELAY, 25,
	0x3002, 0x00,  //Master mode start
};


static uint16_t imx415_sony_init_3840x2160_dol2_setting[] = {
// IMX415-AAQR Window cropping 3840x2160 CSI-2_4lane 27MHz AD:10bit Output:10bit 1485Mbps Master Mode DOL HDR 2frame VC 30.002fps Integration Time LEF:10.001ms SEF:1.016ms Gain:6dB 
 	0x3000, 0x01, // standby 
 	0x3002, 0x01, // Master mode stop
0x3008, 0x5D,
0x300A, 0x42,// CPWAIT_TIME[9:0]
0x301C, 0x04,// WINMODE[3:0]
0x3024, 0xFC,// VMAX[19:0]
0x3028, 0x1A,// HMAX[15:0]
0x302C, 0x01,// WDMODE[1:0]
0x302D, 0x01,// WDSEL[1:0]
0x3031, 0x00,// ADBIT[1:0]
0x3032, 0x00,// MDBIT
0x3033, 0x08,// SYS_MODE[3:0]
0x3040, 0x0C,// PIX_HST[12:0]
0x3042, 0x00,// PIX_HWIDTH[12:0]
0x3044, 0x20,// PIX_VST[12:0]
0x3045, 0x00,// 
0x3046, 0xE0,// PIX_VWIDTH[12:0]
0x3047, 0x10,// 
0x3050, 0x94,// SHR0[19:0]
0x3051, 0x0C,// 
0x3054, 0x09,// SHR1[19:0]
0x3060, 0x95,// RHS1[19:0]
0x3090, 0x14,// GAIN_PCG_0[8:0]
0x30C1, 0x00,// XVS_DRV[1:0]
0x30CF, 0x01,// XVSMSKCNT_INT[1:0]
0x3116, 0x23,// INCKSEL2[7:0]
0x3118, 0xA5,// INCKSEL3[10:0]
0x311A, 0xE7,// INCKSEL4[10:0]
0x311E, 0x23,// INCKSEL5[7:0]
0x32D4, 0x21,// -
0x32EC, 0xA1,// -
0x3452, 0x7F,// -
0x3453, 0x03,// -
0x358A, 0x04,// -
0x35A1, 0x02,// -
0x36BC, 0x0C,// -
0x36CC, 0x53,// -
0x36CD, 0x00,// -
0x36CE, 0x3C,// -
0x36D0, 0x8C,// -
0x36D1, 0x00,// -
0x36D2, 0x71,// -
0x36D4, 0x3C,// -
0x36D6, 0x53,// -
0x36D7, 0x00,// -
0x36D8, 0x71,// -
0x36DA, 0x8C,// -
0x36DB, 0x00,// -
0x3701, 0x00,// ADBIT1[7:0]
0x3724, 0x02,// -
0x3726, 0x02,// -
0x3732, 0x02,// -
0x3734, 0x03,// -
0x3736, 0x03,// -
0x3742, 0x03,// -
0x3862, 0xE0,// -
0x38CC, 0x30,// -
0x38CD, 0x2F,// -
0x395C, 0x0C,// -
0x3A42, 0xD1,// -
0x3A4C, 0x77,// -
0x3AE0, 0x02,// -
0x3AEC, 0x0C,// -
0x3B00, 0x2E,// -
0x3B06, 0x29,// -
0x3B98, 0x25,// -
0x3B99, 0x21,// -
0x3B9B, 0x13,// -
0x3B9C, 0x13,// -
0x3B9D, 0x13,// -
0x3B9E, 0x13,// -
0x3BA1, 0x00,// -
0x3BA2, 0x06,// -
0x3BA3, 0x0B,// -
0x3BA4, 0x10,// -
0x3BA5, 0x14,// -
0x3BA6, 0x18,// -
0x3BA7, 0x1A,// -
0x3BA8, 0x1A,// -
0x3BA9, 0x1A,// -
0x3BAC, 0xED,// -
0x3BAD, 0x01,// -
0x3BAE, 0xF6,// -
0x3BAF, 0x02,// -
0x3BB0, 0xA2,// -
0x3BB1, 0x03,// -
0x3BB2, 0xE0,// -
0x3BB3, 0x03,// -
0x3BB4, 0xE0,// -
0x3BB5, 0x03,// -
0x3BB6, 0xE0,// -
0x3BB7, 0x03,// -
0x3BB8, 0xE0,// -
0x3BBA, 0xE0,// -
0x3BBC, 0xDA,// -
0x3BBE, 0x88,// -
0x3BC0, 0x44,// -
0x3BC2, 0x7B,// -
0x3BC4, 0xA2,// -
0x3BC8, 0xBD,// -
0x3BCA, 0xBD,// -
0x4004, 0xC0,// TXCLKESC_FREQ[15:0]
0x4005, 0x06,// 
0x4018, 0xA7,// TCLKPOST[15:0]
0x401A, 0x57,// TCLKPREPARE[15:0]
0x401C, 0x5F,// TCLKTRAIL[15:0]
0x401E, 0x97,// TCLKZERO[15:0]
0x4020, 0x5F,// THSPREPARE[15:0]
0x4022, 0xAF,// THSZERO[15:0]
0x4024, 0x5F,// THSTRAIL[15:0]
0x4026, 0x9F,// THSEXIT[15:0]
0x4028, 0x4F,// TLPX[15:0]
0x3000, 0x00,
// wait_ms(30)
0x3002, 0x00,

};

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_SENSOR_INC_IMX415_SETTING_H_

