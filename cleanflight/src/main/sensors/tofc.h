
#pragma once

#include "drivers/gpio.h"


#define TOFC_DEVICE_COUNT 5
#define TOFC_DEVICE_START_ADDR (0x52 >> 1) + 1
#define TOFC_INVALID_RANGE 20


//tofc = time of flight camera

typedef enum {
	TOFC_DEFAULT = 0,
	TOFC_NONE = 1,
	TOFC_VL53l0X = 2,
	TOFC_FAKE
} tofcSensor_e;

typedef void(*tofcOpFuncPtr)(void);    // task operation

typedef enum {
	//CW:���ɰw
	//TOFC_ALIGN_DEFAULT,   // driver-provided alignment
	TOFC_ALIGN_FRONT = 0,
	TOFC_ALIGN_RIGHT,
	TOFC_ALIGN_BACK,
	TOFC_ALIGN_LEFT,
	TOFC_ALIGN_BOTTOM
	//TOFC_ALIGN_Z_CW0_DEG = 1,
	//TOFC_ALIGN_Z_CW90_DEG = 2,
	//TOFC_ALIGN_Z_CW180_DEG = 3,
	//TOFC_ALIGN_Z_CW270_DEG = 4,
	//TOFC_ALIGN_Z_CW0_DEG_X_CW90_DEG = 5
} tofc_align_e;



typedef struct tofcDevice_s {
	GPIO_TypeDef* i2cXsdnGpioType; //��l�ƨϥΪ�Xsdn pin type
	gpio_config_t i2cXsdnGpioCfg; //��l�ƨϥΪ�Xsdn pin Config
	uint8_t i2cAddr;
	//tofcOpFuncPtr init; //�]�w�Z���q���Ҧ�
} tofcDevice_t;
typedef struct tofcData_s {
	uint16_t range;

	// -------- VL53L0X --------
	uint8_t rangeStatus;
	uint8_t deviceError;
	uint16_t ambientRate;
	uint32_t signalRate;
	uint16_t effectiveSpadRtnCount;
	// -------------------------
} tofcData_t;
typedef struct tofcConfig_s {
	uint32_t validRangeKeepMs;

	//TODO ���R�W�U�C�ܼƨϨ�Ȱ줣���V�c
	//���ƭ�"�H�U"���L�ĶZ��
	uint16_t inValidRangeBoundaryLow;
	//"�W�L"���ƭȬ��L�ĶZ��
	uint16_t inValidRangeBoundaryHigh;
} tofcConfig_t;

typedef struct tofc_s {
	//int tofDeviceCount;
	bool enable; //�O�_�ҥ�
	tofcDevice_t device;
	tofcData_t data;
	tofcConfig_t config;
	
	uint16_t lastValidRange;
	uint32_t lastValidRangeTime;

	
	// ----- i2c -----
	//tofcOpFuncPtr setRangeingMode; //�]�w�Z���q���Ҧ�
	//tofcOpFuncPtr rangingMeasurement; //�Odevice�}�l�q��(single shot mode)
	//tofcOpFuncPtr getRangingMeasurementData; //�Vdevice���o�q�����G
} tofc_t;

extern int16_t TOFC_angle[ANGLE_INDEX_COUNT];
extern tofc_t tofc[TOFC_DEVICE_COUNT];
extern bool tofc_debug_avoidanceMode;

void tofcInit(void);
void tofcUpdate(void);
void updateTofcStateForAvoidanceMode(void);

bool tofcIsValidRange(tofc_t tofc1);

//alt hold
bool tofcIsAltitudeEnable(void);
bool tofcGetAltitudeSensor(tofc_t* altTofc);
int32_t tofcGetAltitudeMm(float cosTiltAngle);

//void resetDistanceTrims(distanceTrims_t *distanceTrims);
//void updateLaserRangeFinderReadings(distanceTrims_t *distanceTrims);
//void setLaserRangeFinderTrims(flightDynamicsTrims_t *laserRangeFinderTrimsToUse);



