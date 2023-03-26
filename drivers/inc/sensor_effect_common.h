/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_
#define UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_IOC_MAGIC   'x'
#define CAMERA_REG_PARAM    _IOW(CAMERA_IOC_MAGIC, 11, camera_state_register_t)

#define CAMERA_SENSOR_NAME  20

typedef struct register_info {
	uint8_t reg_width;
	uint8_t value_width;
	uint16_t *register_table;
	uint32_t size;
}register_info_t;

typedef struct camera_state_register_info {
	uint8_t  bus_num;
	uint8_t  mipi_index;
	uint8_t  deserial_addr;
	uint8_t  serial_addr;
	uint8_t  sensor_addr;
	register_info_t deserial_register_info;
	register_info_t serial_register_info;
	register_info_t sensor_register_info;
}camera_state_register_t;

enum sensor_mode_e {
    NORMAL_M = 1,
    DOL2_M = 2,
    DOL3_M = 3,
    DOL4_M = 4,
    PWL_M = 5,
    INVALID_MOD,
};

typedef struct sensor_data {
	uint32_t  turning_type;  //  1:imx290 2: ar0233
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
}sensor_data_t;

/* line use y = ratio * x + offset;
 * input param:
 * ratio(0,1) : 0: -1, 1: 1
 * offset:
 * max:
 */

typedef struct ctrlp_s {
	int32_t ratio;
	uint32_t offset;
	uint32_t max;
} ctrlp_t;

/*
 * distinguish between dgain and again
 * note1: a sensor could only have again or dgain
 * note2: some sensor again/dgain in the same register, could only use again
 *        eg. imx327
 * note3: some sensor the again is stepped, could noly use again.
 * note4: again/dgain could have multiple registers,
 * note5: again [0,255], actual = 2^(again/32)
 * note6: dgain [0,255], actual = 2^(dgain/32)
 * note7: dol2/dol3/dol4 used the same again/dgain
 */

typedef struct dol3_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[3];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
} dol3_t;

typedef struct dol2_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[2];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}dol2_t;
typedef struct normal_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}normal_t;
typedef struct pwl_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t line;
	uint32_t line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}pwl_t;

typedef struct stream_ctrl_s {
	uint32_t stream_on[10];
	uint32_t stream_off[10];
	uint32_t data_length;
}stream_ctrl_t;

typedef struct sensor_awb_ctrl_s {
	uint32_t rgain_addr[4];
	uint32_t rgain_length[4];
	uint32_t bgain_addr[4];
	uint32_t bgain_length[4];
	uint32_t grgain_addr[4];
	uint32_t grgain_length[4];
	uint32_t gbgain_addr[4];
	uint32_t gbgain_length[4];
	uint32_t rb_prec;
} sensor_awb_ctrl_t;

typedef struct sensor_turning_data {
	uint32_t  port;
	char      sensor_name[CAMERA_SENSOR_NAME];
	uint32_t  sensor_addr;
    uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_id;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	pwl_t    pwl;  // ar0233
	sensor_awb_ctrl_t sensor_awb;
	stream_ctrl_t stream_ctrl;
	sensor_data_t sensor_data;
}sensor_turning_data_t;

typedef struct sensor_turning_data_ex {
	uint32_t  ratio_en;
	uint32_t  ratio_value;
	uint32_t  l_line;
	uint32_t  l_line_length;
	uint32_t  lexposure_time_min;
}sensor_turning_data_ex_t;

struct MODE_SW_s {
	uint8_t WDC_OUTSEL_num;
	uint8_t WDC_THR_FRM_SEL_num;
};

enum mode_e {
	SP1_HCG_P = 0x000000,
	SP1_HCG_T = 0x010000,
	SP1_LCG_P = 0x000001,
	SP1_LCG_T = 0x010001,
	SP2_P = 0x000002,
	SP2_T = 0x010002,
	HDR = 0x000100
};

struct STA_exposure_s {
	uint32_t MODE_VMAX_num;
	uint16_t MODE_HMAX_num;
	uint32_t OFFSET_CLK_SP1H_num;
	uint32_t OFFSET_CLK_SP1L_num;
	uint8_t  FMAX_num;
};

struct SP1_exposure_s {
	uint32_t SHS1_num;
};

struct SP2_exposure_s {
	uint32_t SHS1_num;
	uint32_t SHS2_num;
};

enum exposure_sw_e {
	SP1_HCG_e_mode = 0,
	SP1_LCG_e_mode,
	SP2_1_e_mode,
	SP2_2_e_mode
};

enum gain_toal_sw_e {
	Part_mode = 0,
	Total_mode = 1
};


struct GAIN_mode_s {
	enum gain_toal_sw_e GAIN_SP1H_sw;
	enum gain_toal_sw_e GAIN_SP1L_sw;
	enum gain_toal_sw_e GAIN_SP2_sw;
};

struct GAIN_SEP_mode_s {
	uint8_t serverd:5;
	uint8_t sp2_sep_mode:1;
	uint8_t sp1l_sep_mode:1;
	uint8_t sp1h_sep_mode:1;
};

union GAIN_SEP_mode_u {
	uint8_t sep_mode_g;
	struct GAIN_SEP_mode_s sep_mode_bit;
};

struct SP1_HCG_P_gain_s {
	uint16_t AGAIN_SP1H_num;
	uint16_t PGA_GAIN_SP1H_num;
};

struct SP1_HCG_T_gain_s {
	uint16_t GAIN_SP1H_num;
};


struct SP1_LCG_P_gain_s {
	uint16_t AGAIN_SP1L_num;
	uint16_t PGA_GAIN_SP1L_num;
};

struct SP1_LCG_T_gain_s {
	uint16_t GAIN_SP1L_num;
};


struct SP2_P_gain_s {
	uint16_t AGAIN_SP1L_num;
	uint16_t PGA_GAIN_SP2_num;
};

struct GAIN_LIMIT_s {
	uint16_t AG_SP1H_LIMIT_num;
	uint16_t AG_SP1L_LIMIT_num;
};

struct GAIN_PWL_s {
        uint16_t AGAIN_SP1H_num;
        uint16_t AGAIN_SP1L_num;
        uint16_t PGA_GAIN_SP1H_num;
};

struct GAIN_NORMAL_P_u {
	struct SP2_P_gain_s sp2_p_g_num;
	struct SP1_LCG_P_gain_s sp1_p_lcg_num;
	struct SP1_HCG_P_gain_s sp1_p_hcg_num;
};

struct GAIN_NORMAL_T_u {
	struct SP1_HCG_T_gain_s sp1_t_hcg_num;
	struct SP1_LCG_T_gain_s sp1_t_lcg_num;
};

struct GAIN_DATA_s {
	struct GAIN_PWL_s pwl_d;
	struct GAIN_NORMAL_P_u gain_p_d;
	struct GAIN_NORMAL_T_u gain_t_d;
};

#ifdef __cplusplus
}
#endif

#endif // UTILITY_SENSOR_INC_SENSOR_EFFECT_COMMON_H_

