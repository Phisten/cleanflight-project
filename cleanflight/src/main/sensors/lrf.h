
#pragma once


//lrf = Laser Range Finder

typedef enum {
	LRF_DEFAULT = 0,
	LRF_NONE = 1,
	LRF_VL53L0X = 2,
	LRF_FAKE
} laserRangeFinderSensor_e;

typedef void(*lrfOpFuncPtr)(void);    // task operation

typedef enum {
	//CW:¶¶®É°w
	LRF_ALIGN_DEFAULT = 0,   // driver-provided alignment
	LRF_ALIGN_Z_CW0_DEG = 1,
	LRF_ALIGN_Z_CW90_DEG = 2,
	LRF_ALIGN_Z_CW180_DEG = 3,
	LRF_ALIGN_Z_CW270_DEG = 4,
	LRF_ALIGN_Z_CW0_DEG_X_CW90_DEG = 5
} lrf_align_e;


extern lrf_t lrf;

void lrfInit(void);

//void resetDistanceTrims(distanceTrims_t *distanceTrims);
//void updateLaserRangeFinderReadings(distanceTrims_t *distanceTrims);
//void setLaserRangeFinderTrims(flightDynamicsTrims_t *laserRangeFinderTrimsToUse);



