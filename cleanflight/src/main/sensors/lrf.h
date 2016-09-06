
#pragma once

#include "drivers/gpio.h"


#define LRF_DEVICE_COUNT 1
#define LRF_DEVICE_START_ADDR 0x30


//lrf = Laser Range Finder

typedef enum {
	LRF_DEFAULT = 0,
	LRF_NONE = 1,
	LRF_VL53L0X = 2,
	LRF_FAKE
} laserRangeFinderSensor_e;

typedef void(*lrfOpFuncPtr)(void);    // task operation

typedef enum {
	//CW:抖皐
	LRF_ALIGN_DEFAULT = 0,   // driver-provided alignment
	LRF_ALIGN_FRONT,
	LRF_ALIGN_RIGHT,
	LRF_ALIGN_BACK,
	LRF_ALIGN_LEFT,
	LRF_ALIGN_BOTTOM
	//LRF_ALIGN_Z_CW0_DEG = 1,
	//LRF_ALIGN_Z_CW90_DEG = 2,
	//LRF_ALIGN_Z_CW180_DEG = 3,
	//LRF_ALIGN_Z_CW270_DEG = 4,
	//LRF_ALIGN_Z_CW0_DEG_X_CW90_DEG = 5
} lrf_align_e;



typedef struct lrfDevice_s {
	GPIO_TypeDef* i2cXsdnGpioType; //﹍てㄏノXsdn pin type
	gpio_config_t i2cXsdnGpioCfg; //﹍てㄏノXsdn pin Config
	uint8_t i2cAddr;
	//lrfOpFuncPtr init; //砞﹚禯瞒秖代家Α
} lrfDevice_t;
typedef struct lrfData_s {
	uint16_t range;
} lrfData_t;

typedef struct lrf_s {
	//int lrfDeviceCount;
	bool enable; //琌币ノ
	lrfDevice_t device;
	lrfData_t data;
	// ----- i2c -----
	//lrfOpFuncPtr setRangeingMode; //砞﹚禯瞒秖代家Α
	//lrfOpFuncPtr rangingMeasurement; //device秨﹍秖代(single shot mode)
	//lrfOpFuncPtr getRangingMeasurementData; //device眔秖代挡狦
} lrf_t;

extern int16_t LRF_angle[ANGLE_INDEX_COUNT];
extern lrf_t lrf[LRF_DEVICE_COUNT];
extern bool lrf_debug_avoidanceMode;

void lrfInit(void);
void lrfUpdate(void);
void updateLrfStateForAvoidanceMode(void);

//void resetDistanceTrims(distanceTrims_t *distanceTrims);
//void updateLaserRangeFinderReadings(distanceTrims_t *distanceTrims);
//void setLaserRangeFinderTrims(flightDynamicsTrims_t *laserRangeFinderTrimsToUse);



