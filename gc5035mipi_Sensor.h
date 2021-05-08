/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc5035mipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef __GC5035MIPI_SENSOR_H__
#define __GC5035MIPI_SENSOR_H__

#ifndef NULL
#ifdef __cplusplus
#define NULL	0
#else
#define NULL	((void *)0)
#endif
#endif

typedef int bool;
typedef int BOOL;
#define KAL_TRUE	1
#define KAL_FALSE	0
#define FALSE		0

#define kal_uint8				unsigned char 
#define kal_uint16				unsigned short 
#define kal_int16				short 
#define kal_uint32				unsigned int 
#define kal_bool				bool

#define u8		unsigned char
#define u16		unsigned short
#define u32		unsigned int

#define UINT8		unsigned char
#define UINT16		unsigned short
#define UINT32		unsigned int
#define MUINT32		unsigned int

#define uintptr_t	unsigned int*

#define ERROR_NONE					0
#define ERROR_SENSOR_CONNECT_FAIL	0x01
#define ERROR_INVALID_SCENARIO_ID	0x01

#define GC5035MIPI_SENSOR_ID		0x5035

#define ISP_DRIVING_6MA					0x06
#define SENSOR_INTERFACE_TYPE_MIPI		0x01
#define MIPI_OPHY_NCSI2					0x01
#define MIPI_SETTLEDELAY_AUTO			0x01
#define SENSOR_OUTPUT_FORMAT_RAW_B		0x01
#define SENSOR_OUTPUT_FORMAT_RAW_R		0x00
#define SENSOR_MIPI_2_LANE				0x02
#define IMAGE_NORMAL					0x00
#define SENSOR_CLOCK_POLARITY_LOW		0x00
#define LENS_DRIVER_ID_DO_NOT_CARE		0x01

typedef struct exp_win {
	kal_uint8 xxx;
}MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT;

typedef struct sensor_config {
	kal_uint8 xxxx;
}MSDK_SENSOR_CONFIG_STRUCT;

typedef struct sensor_info {
	kal_uint8 SensorClockPolarity;
	kal_uint8 SensorClockFallingPolarity;
	kal_uint8 SensorHsyncPolarity;
	kal_uint8 SensorVsyncPolarity;
	kal_uint8 SensorInterruptDelayLines;
	BOOL SensorResetActiveHigh;
	kal_uint8 SensorResetDelayCount;

	kal_uint8 SensroInterfaceType;
	kal_uint8 MIPIsensorType;
	kal_uint8 SettleDelayMode;
	kal_uint8 SensorOutputDataFormat;

	kal_uint8 CaptureDelayFrame;
	kal_uint8 PreviewDelayFrame;
	kal_uint8 VideoDelayFrame;
	kal_uint8 HighSpeedVideoDelayFrame;
	kal_uint8 SlimVideoDelayFrame;

	kal_uint8 SensorMasterClockSwitch;
	kal_uint8 SensorDrivingCurrent;
	kal_uint8 AEShutDelayFrame;
	kal_uint8 AESensorGainDelayFrame;
	kal_uint8 AEISPGainDelayFrame;
	kal_uint8 IHDR_Support;
	kal_uint8 IHDR_LE_FirstLine;
	kal_uint8 SensorModeNum;

	kal_uint8 SensorMIPILaneNumber;
	kal_uint8 SensorClockFreq;
	kal_uint8 SensorClockDividCount;
	kal_uint8 SensorClockRisingCount;
	kal_uint8 SensorClockFallingCount;
	kal_uint8 SensorPixelClockCount;
	kal_uint8 SensorDataLatchCount;

	kal_uint8 MIPIDataLowPwr2HighSpeedTermDelayCount;
	kal_uint8 MIPICLKLowPwr2HighSpeedTermDelayCount;
	kal_uint8 SensorWidthSampling;
	kal_uint8 SensorHightSampling;
	kal_uint8 SensorPacketECCOrder;\

	kal_uint8 SensorGrabStartX;
	kal_uint8 SensorGrabStartY;
	kal_uint8 MIPIDataLowPwr2HighSpeedSettleDelayCount;

}MSDK_SENSOR_INFO_STRUCT;

typedef struct sensor_res {
	kal_uint16 SensorFullWidth;
	kal_uint16 SensorFullHeight;
	kal_uint16 SensorPreviewWidth;
	kal_uint16 SensorPreviewHeight;

	kal_uint16 SensorVideoWidth;
	kal_uint16 SensorVideoHeight;
	kal_uint16 SensorHighSpeedVideoWidth;
	kal_uint16 SensorHighSpeedVideoHeight;
	kal_uint16 SensorSlimVideoWidth;
	kal_uint16 SensorSlimVideoHeight;
}MSDK_SENSOR_RESOLUTION_INFO_STRUCT;

typedef struct sensor_reg_data {
	kal_uint8 RegAddr;
	kal_uint8 RegData;
	
}MSDK_SENSOR_REG_INFO_STRUCT;

typedef struct win_info {
	kal_uint16 sensor_width;
	kal_uint16 sensor_height;
	kal_uint16 sensor_width_offset;
	kal_uint16 sensor_height_offset;
	kal_uint16 sensor_crop_width;
	kal_uint16 sensor_crop_height;
	kal_uint16 sensor_crop_width_binning;
	kal_uint16 sensor_crop_height_binning;
	kal_uint16 sensor_crop_width_binning_offset;
	kal_uint16 sensor_crop_height_binning_offset;
	kal_uint16 wind_width;
	kal_uint16 wind_height;
	kal_uint16 wind_width_crop;
	kal_uint16 wind_height_crop;
	kal_uint16 wind_width_finaly;
	kal_uint16 wind_height_finaly;
}SENSOR_WINSIZE_INFO_STRUCT;


typedef enum { MSDK_SCENARIO_ID_CAMERA_PREVIEW = 0x01, MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG, MSDK_SCENARIO_ID_VIDEO_PREVIEW,\
	MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO, MSDK_SCENARIO_ID_SLIM_VIDEO}MSDK_SCENARIO_ID_ENUM;

typedef enum {
	SENSOR_FEATURE_GET_PERIOD = 0x01, SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ, SENSOR_FEATURE_GET_MIPI_PIXEL_RATE, SENSOR_FEATURE_SET_ESHUTTER,\
	SENSOR_FEATURE_SET_NIGHTMODE, SENSOR_FEATURE_SET_GAIN, SENSOR_FEATURE_SET_FLASHLIGHT, SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ,\
	SENSOR_FEATURE_SET_REGISTER, SENSOR_FEATURE_GET_REGISTER, SENSOR_FEATURE_GET_LENS_DRIVER_ID, SENSOR_FEATURE_SET_VIDEO_MODE,\
	SENSOR_FEATURE_CHECK_SENSOR_ID, SENSOR_FEATURE_SET_AUTO_FLICKER_MODE, SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO,\
	SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO, SENSOR_FEATURE_SET_TEST_PATTERN, SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE,
	SENSOR_FEATURE_SET_FRAMERATE, SENSOR_FEATURE_SET_HDR, SENSOR_FEATURE_GET_CROP_INFO, SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN
}MSDK_SENSOR_FEATURE_ENUM;

typedef struct sensor_function_struct {
	uintptr_t open;
	uintptr_t get_info;
	uintptr_t get_resolution;
	uintptr_t feature_control;
	uintptr_t control;
	uintptr_t close;
}SENSOR_FUNCTION_STRUCT;


#define PSENSOR_FUNCTION_STRUCT		uintptr_t


/* SENSOR MIRROR FLIP INFO */
#define GC5035_MIRROR_FLIP_ENABLE         0
#if GC5035_MIRROR_FLIP_ENABLE
#define GC5035_MIRROR                     0x83
#define GC5035_RSTDUMMY1                  0x03
#define GC5035_RSTDUMMY2                  0xfc
#else
#define GC5035_MIRROR                     0x80
#define GC5035_RSTDUMMY1                  0x02
#define GC5035_RSTDUMMY2                  0x7c
#endif

/* SENSOR PRIVATE INFO FOR GAIN SETTING */
#define GC5035_SENSOR_GAIN_BASE             256
#define GC5035_SENSOR_GAIN_MAX              (16 * GC5035_SENSOR_GAIN_BASE)
#define GC5035_SENSOR_GAIN_MAX_VALID_INDEX  17
#define GC5035_SENSOR_GAIN_MAP_SIZE         17
#define GC5035_SENSOR_DGAIN_BASE            0x100

/* SENSOR PRIVATE INFO FOR OTP SETTINGS */
#define GC5035_OTP_FOR_CUSTOMER            0
#define GC5035_OTP_DEBUG                   0

/* DEBUG */
#if GC5035_OTP_DEBUG
#define GC5035_OTP_START_ADDR              0x0000
#endif

#define GC5035_OTP_DATA_LENGTH             1024

/* OTP FLAG TYPE */
#define GC5035_OTP_FLAG_EMPTY              0x00
#define GC5035_OTP_FLAG_VALID              0x01
#define GC5035_OTP_FLAG_INVALID            0x02
#define GC5035_OTP_FLAG_INVALID2           0x03
#define GC5035_OTP_GET_OFFSET(size)           (size << 3)
#define GC5035_OTP_GET_2BIT_FLAG(flag, bit)   ((flag >> bit) & 0x03)
#define GC5035_OTP_CHECK_1BIT_FLAG(flag, bit) (((flag >> bit) & 0x01) == GC5035_OTP_FLAG_VALID)

#define GC5035_OTP_ID_SIZE                 9
#define GC5035_OTP_ID_DATA_OFFSET          0x0020

/* OTP DPC PARAMETERS */
#define GC5035_OTP_DPC_FLAG_OFFSET         0x0068
#define GC5035_OTP_DPC_TOTAL_NUMBER_OFFSET 0x0070
#define GC5035_OTP_DPC_ERROR_NUMBER_OFFSET 0x0078

/* OTP REGISTER UPDATE PARAMETERS */
#define GC5035_OTP_REG_FLAG_OFFSET         0x0880
#define GC5035_OTP_REG_DATA_OFFSET         0x0888
#define GC5035_OTP_REG_MAX_GROUP           5
#define GC5035_OTP_REG_BYTE_PER_GROUP      5
#define GC5035_OTP_REG_REG_PER_GROUP       2
#define GC5035_OTP_REG_BYTE_PER_REG        2
#define GC5035_OTP_REG_DATA_SIZE           (GC5035_OTP_REG_MAX_GROUP * GC5035_OTP_REG_BYTE_PER_GROUP)
#define GC5035_OTP_REG_REG_SIZE            (GC5035_OTP_REG_MAX_GROUP * GC5035_OTP_REG_REG_PER_GROUP)

#if GC5035_OTP_FOR_CUSTOMER
#define GC5035_OTP_CHECK_SUM_BYTE          1
#define GC5035_OTP_GROUP_CNT               2

/* OTP MODULE INFO PARAMETERS */
#define GC5035_OTP_MODULE_FLAG_OFFSET      0x1f10
#define GC5035_OTP_MODULE_DATA_OFFSET      0x1f18
#define GC5035_OTP_MODULE_DATA_SIZE        6

/* OTP WB PARAMETERS */
#define GC5035_OTP_WB_FLAG_OFFSET          0x1f78
#define GC5035_OTP_WB_DATA_OFFSET          0x1f80
#define GC5035_OTP_WB_DATA_SIZE            4
#define GC5035_OTP_GOLDEN_DATA_OFFSET      0x1fc0
#define GC5035_OTP_GOLDEN_DATA_SIZE        4
#define GC5035_OTP_WB_CAL_BASE             0x0800
#define GC5035_OTP_WB_GAIN_BASE            0x0400

/* WB TYPICAL VALUE 0.5 */
#define GC5035_OTP_WB_RG_TYPICAL           0x0400
#define GC5035_OTP_WB_BG_TYPICAL           0x0400
#endif

enum {
	otp_close = 0,
	otp_open,
};

/* DPC STRUCTURE */
struct gc5035_dpc_t {
	kal_uint8 flag;
	kal_uint16 total_num;
};

struct gc5035_reg_t {
	kal_uint8 page;
	kal_uint8 addr;
	kal_uint8 value;
};

/* REGISTER UPDATE STRUCTURE */
struct gc5035_reg_update_t {
	kal_uint8 flag;
	kal_uint8 cnt;
	struct gc5035_reg_t reg[GC5035_OTP_REG_REG_SIZE];
};

#if GC5035_OTP_FOR_CUSTOMER
/* MODULE INFO STRUCTURE */
struct gc5035_module_info_t {
	kal_uint8 module_id;
	kal_uint8 lens_id;
	kal_uint8 year;
	kal_uint8 month;
	kal_uint8 day;
};

/* WB STRUCTURE */
struct gc5035_wb_t {
	kal_uint8  flag;
	kal_uint16 rg;
	kal_uint16 bg;
};
#endif

/* OTP STRUCTURE */
struct gc5035_otp_t {
	kal_uint8 otp_id[GC5035_OTP_ID_SIZE];
	struct gc5035_dpc_t dpc;
	struct gc5035_reg_update_t regs;
#if GC5035_OTP_FOR_CUSTOMER
	struct gc5035_wb_t wb;
	struct gc5035_wb_t golden;
#endif
};

enum {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;                /* record different mode's pclk */
	kal_uint32 linelength;          /* record different mode's linelength */
	kal_uint32 framelength;         /* record different mode's framelength */
	kal_uint8  startx;              /* record different mode's startx of grabwindow */
	kal_uint8  starty;              /* record different mode's startx of grabwindow */
	kal_uint16 grabwindow_width;    /* record different mode's width of grabwindow */
	kal_uint16 grabwindow_height;   /* record different mode's height of grabwindow */
	kal_uint32 mipi_pixel_rate;
	/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
	kal_uint8  mipi_data_lp2hs_settle_dc;
	/* following for GetDefaultFramerateByScenario() */
	kal_uint16 max_framerate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES */
struct imgsensor_struct {
	kal_uint8  mirror;                /* mirrorflip information */
	kal_uint8  sensor_mode;           /* record IMGSENSOR_MODE enum value */
	kal_uint32 shutter;               /* current shutter */
	kal_uint16 gain;                  /* current gain */
	kal_uint32 pclk;                  /* current pclk */
	kal_uint32 frame_length;          /* current framelength */
	kal_uint32 line_length;           /* current linelength */
	kal_uint32 min_frame_length;      /* current min  framelength to max framerate */
	kal_uint16 dummy_pixel;           /* current dummypixel */
	kal_uint16 dummy_line;            /* current dummline */
	kal_uint16 current_fps;           /* current max fps */
	kal_bool   autoflicker_en;        /* record autoflicker enable or disable */
	kal_bool   test_pattern;          /* record test pattern mode or not */
	//MSDK_SCENARIO_ID_ENUM current_scenario_id; /* current scenario id */
	MSDK_SCENARIO_ID_ENUM current_scenario_id; /* current scenario id */
	kal_bool   ihdr_en;                        /* ihdr enable or disable */
	kal_uint8  i2c_write_id;          /* record current sensor's i2c write id */
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct imgsensor_info_struct {
	kal_uint32 sensor_id;                     /* record sensor id defined in Kd_imgsensor.h */
	kal_uint32 checksum_value;                /* checksum value for Camera Auto Test */
	struct imgsensor_mode_struct pre;         /* preview scenario relative information */
	struct imgsensor_mode_struct cap;         /* capture scenario relative information */
	struct imgsensor_mode_struct cap1;        /* capture for PIP 24fps relative information */
	/* capture1 mode must use same framelength, linelength with Capture mode for shutter calculate */
	struct imgsensor_mode_struct cap2;
	struct imgsensor_mode_struct normal_video;/* normal video  scenario relative information */
	struct imgsensor_mode_struct hs_video;    /* high speed video scenario relative information */
	struct imgsensor_mode_struct slim_video;  /* slim video for VT scenario relative information */
	kal_uint8 ae_shut_delay_frame;            /* shutter delay frame for AE cycle */
	kal_uint8 ae_sensor_gain_delay_frame;     /* sensor gain delay frame for AE cycle */
	kal_uint8 ae_ispGain_delay_frame;         /* isp gain delay frame for AE cycle */
	kal_uint8 ihdr_support;                   /* 1, support; 0,not support */
	kal_uint8 ihdr_le_firstline;              /* 1,le first ; 0, se first */
	kal_uint8 sensor_mode_num;                /* support sensor mode num */
	kal_uint8 cap_delay_frame;                /* enter capture delay frame num */
	kal_uint8 pre_delay_frame;                /* enter preview delay frame num */
	kal_uint8 video_delay_frame;              /* enter video delay frame num */
	kal_uint8 hs_video_delay_frame;           /* enter high speed video  delay frame num */
	kal_uint8 slim_video_delay_frame;         /* enter slim video delay frame num */
	kal_uint8 margin;                         /* sensor framelength & shutter margin */
	kal_uint32 min_shutter;                   /* min shutter */
	kal_uint32 max_frame_length;              /* max framelength by sensor register's limitation */
	kal_uint8 isp_driving_current;            /* mclk driving current */
	kal_uint8 sensor_interface_type;          /* sensor_interface_type */
	kal_uint8 mipi_sensor_type;
	/* 0, MIPI_OPHY_NCSI2; 1, MIPI_OPHY_CSI2, default is NCSI2, don't modify this para */
	kal_uint8 mipi_settle_delay_mode;
	/* 0, high speed signal auto detect; 1, use settle delay, unit is ns*/
	/* default is auto detect, don't modify this para */
	kal_uint8 sensor_output_dataformat;       /* sensor output first pixel color */
	kal_uint8 mclk;                           /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	kal_uint8 mipi_lane_num;                  /* mipi lane num */
	kal_uint8 i2c_addr_table[5];
	/* record sensor support all write id addr, only supprt 4must end with 0xff */
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);

void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en);
void set_dummy(void);
void set_shutter(kal_uint16 shutter);
kal_uint16 set_gain(kal_uint16 gain);

kal_uint32 open(void);


#endif
