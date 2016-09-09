
#pragma once

#include "drivers/gpio.h"


#define TOF_DEVICE_COUNT 5
#define TOF_DEVICE_START_ADDR 0x30


//tof = Laser Range Finder

typedef enum {
	tof_DEFAULT = 0,
	tof_NONE = 1,
	tof_VL53L0X = 2,
	tof_FAKE
} laserRangeFinderSensor_e;

typedef void(*tofOpFuncPtr)(void);    // task operation

typedef enum {
	//CW:抖皐
	//tof_ALIGN_DEFAULT,   // driver-provided alignment
	tof_ALIGN_FRONT = 0,
	tof_ALIGN_RIGHT,
	tof_ALIGN_BACK,
	tof_ALIGN_LEFT,
	tof_ALIGN_BOTTOM
	//tof_ALIGN_Z_CW0_DEG = 1,
	//tof_ALIGN_Z_CW90_DEG = 2,
	//tof_ALIGN_Z_CW180_DEG = 3,
	//tof_ALIGN_Z_CW270_DEG = 4,
	//tof_ALIGN_Z_CW0_DEG_X_CW90_DEG = 5
} tof_align_e;



typedef struct tofDevice_s {
	GPIO_TypeDef* i2cXsdnGpioType; //﹍てㄏノXsdn pin type
	gpio_config_t i2cXsdnGpioCfg; //﹍てㄏノXsdn pin Config
	uint8_t i2cAddr;
	//tofOpFuncPtr init; //砞﹚禯瞒秖代家Α
} tofDevice_t;
typedef struct tofData_s {
	uint16_t range;

	// -------- VL53L0X --------
	uint8_t rangeStatus;
	uint8_t deviceError;
	uint16_t ambientRate;
	uint32_t signalRate;
	uint16_t effectiveSpadRtnCount;
	// -------------------------
} tofData_t;
typedef struct tofConfig_s {
	uint32_t validRangeWaitLimit;


} tofConfig_t;

typedef struct tof_s {
	//int tofDeviceCount;
	bool enable; //琌币ノ
	tofDevice_t device;
	tofData_t data;
	
	uint16_t lastValidRange;
	uint32_t lastValidRangeDaltaTime;

	
	// ----- i2c -----
	//tofOpFuncPtr setRangeingMode; //砞﹚禯瞒秖代家Α
	//tofOpFuncPtr rangingMeasurement; //device秨﹍秖代(single shot mode)
	//tofOpFuncPtr getRangingMeasurementData; //device眔秖代挡狦
} tof_t;

extern int16_t tof_angle[ANGLE_INDEX_COUNT];
extern tof_t tof[TOF_DEVICE_COUNT];
extern bool tof_debug_avoidanceMode;

void tofInit(void);
void tofUpdate(void);
void updatetofStateForAvoidanceMode(void);

bool isAltitudetofEnable(void);
int32_t baroCalculateAltitude(void);

//void resetDistanceTrims(distanceTrims_t *distanceTrims);
//void updateLaserRangeFinderReadings(distanceTrims_t *distanceTrims);
//void setLaserRangeFinderTrims(flightDynamicsTrims_t *laserRangeFinderTrimsToUse);



