

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>
//#include "build_config.h"

#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"


#include "drivers/sensor.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/tofc_vl53l0x.h"
#include "drivers/system.h"
#include "drivers/accgyro.h"

#include "io/rc_controls.h"
#include "io/motor_and_servo.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "sensors/tofc.h"

#ifdef TOFC

#define DEBUG_TOF
//#define VL53L0X

//close #20160906%phis106 未來改為5個感測器
tofc_t tofc[TOFC_DEVICE_COUNT];
uint16_t tofXsdnGpio[TOFC_DEVICE_COUNT]			= { GPIO_Pin_0	,GPIO_Pin_1	,GPIO_Pin_4	,GPIO_Pin_5	,GPIO_Pin_1 }; // IO1 LEDS = GPIO_Pin_8 GPIOB
GPIO_TypeDef* tofXsdnGpioType[TOFC_DEVICE_COUNT] = { GPIOA		,GPIOA		,GPIOB		,GPIOB		,GPIOB };

uint32_t tofCurrentTimeMs = 0;

uint16_t tofAngleAlign[TOFC_DEVICE_COUNT][ANGLE_INDEX_COUNT] = {
	// [tof index: 0~4] [Roll: 0 / Pitch: 1]
	[TOFC_ALIGN_FRONT] = {0,-1}
	,[TOFC_ALIGN_RIGHT] = {-1,0}
	,[TOFC_ALIGN_BACK] = {0,1}
	,[TOFC_ALIGN_LEFT] = {1,0}
	,[TOFC_ALIGN_BOTTOM] = {0,0}
};

bool tofc_debug_avoidanceMode = false;
//level(x,y) avoidance
int16_t TOFC_angle[ANGLE_INDEX_COUNT] = { 0, 0 }; // 測距後避障邏輯輸出的期望傾角
//int16_t TOFC_Throttle = 0; // 測距後避障邏輯輸出的補正油門

//alt(z) avoidance
int32_t tofcErrorVelocityI = 0;


uint16_t makeuint16____(uint16_t lsb, uint16_t msb)
{
	return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void tofcInit(void)
{
	
	//VL53L0X 參數設定
	for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	{
		//GPIO_Pin_1 = PA1 藍線 ,GPIO_Pin_0 = PA0 白線
		gpio_config_t gpioCfg;
		gpioCfg.mode = GPIO_Mode_OUT;
		gpioCfg.pin = tofXsdnGpio[i];
		gpioCfg.speed = 1;

		tofcDevice_t curtofDevice;
		curtofDevice.i2cXsdnGpioCfg = gpioCfg;
		curtofDevice.i2cXsdnGpioType = tofXsdnGpioType[i];
		curtofDevice.i2cAddr = TOFC_DEVICE_START_ADDR + i;

		tofcData_t curTocfData = {0};
		//tofcData_t curRowData[] = { {0},{0},{0} };
		//for (int j = 0; j < TOFC_ROWDATA_COUNT; j++)
		//{
		//	tofcData_t tmpTocfData = { 0 };
		//	curRowData[j] = tmpTocfData;
		//}

		tofcConfig_t curtofcCfg = {
			.validRangeKeepMs = 500,
			.inValidRangeBoundaryLow = VL53L0X_MIN_OF_RANGE,
			.inValidRangeBoundaryHigh = VL53L0X_OUT_OF_RANGE
		};

		tofc_t curtof = {
			.enable = false,
			.device = curtofDevice,
			.data = curTocfData,
			.config = curtofcCfg,
			.rowData = { { 0 },{ 0 },{ 0 },{ 0 },{ 0 } },
			.nextRowDataIndex = 0,

			.lastValidRange = 0,
			.lastValidRangeTime = 0
		};
		//curtof.lastValidRangeTime = millis();
		tofc[i] = curtof;
	}

	//tofc XSDN init
	//for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	//{
	//	tofcDevice_t curtofDevice = tofc[i].device;
	//	gpioInit(curtofDevice.i2cXsdnGpioType, &(curtofDevice.i2cXsdnGpioCfg));
	//}
	//delay(300);
	//tofc裝置初始化
	for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	{
		tofcDevice_t curtofDevice = tofc[i].device;
		tofc[i].enable = tofc_vl53l0x_i2c_init(curtofDevice.i2cXsdnGpioType, curtofDevice.i2cXsdnGpioCfg, curtofDevice.i2cAddr);
	}
	sensorsSet(SENSOR_TOFC);
}

//ToF Camera Sensor Read Data(I2C DMA)
void tofcI2cReadData(tofc_t *pTofc1)
{
	uint8_t in_addr = pTofc1->device.i2cAddr;
	uint8_t VL53L0X_REG_buf[12];

	for (int i = 0; i < 12; i++)
		VL53L0X_REG_buf[i] = 0;
	i2cRead(in_addr, VL53L0X_REG_RESULT_RANGE_STATUS, 12, VL53L0X_REG_buf);

	uint16_t dist = makeuint16____(VL53L0X_REG_buf[11], VL53L0X_REG_buf[10]);
	pTofc1->data.range = dist;
	pTofc1->data.rangeStatus = VL53L0X_REG_buf[0];
	pTofc1->data.deviceError = (VL53L0X_REG_buf[0] & 0x78) >> 3;
	pTofc1->data.signalRate = (uint32_t)makeuint16____(VL53L0X_REG_buf[7], VL53L0X_REG_buf[6]);
	pTofc1->data.ambientRate = makeuint16____(VL53L0X_REG_buf[9], VL53L0X_REG_buf[8]);
	//effective Single-photon avalanche diode Return Count
	pTofc1->data.effectiveSpadRtnCount = makeuint16____(VL53L0X_REG_buf[3], VL53L0X_REG_buf[2]);

	//update rowData
	pTofc1->rowData[pTofc1->nextRowDataIndex] = pTofc1->data;
	pTofc1->nextRowDataIndex = (pTofc1->nextRowDataIndex + 1) % TOFC_ROWDATA_COUNT;
}

void tofcValidRangeUpdate(tofc_t *pTofc1)
{
	int32_t rowRange[TOFC_ROWDATA_COUNT];

	//tofc Range Healthy Update
	pTofc1->lastValidRangeHealthy = true;
	for (int i = 0; i < TOFC_ROWDATA_COUNT; i++)
	{
		//check DeviceError Is Pass
		if (pTofc1->rowData[i].deviceError != VL53L0X_DEVICEERROR_RANGECOMPLETE)
		{
			pTofc1->lastValidRangeHealthy = false;
			break;
		}

		//quickMedianFilter5 init
		rowRange[i] = pTofc1->rowData[i].range;
	}

	//過濾受測物體由靜止轉為移動時可能發生的極大或極小之錯誤測距結果
	uint16_t filtedRange = (uint16_t)quickMedianFilter5(rowRange);


	//tofc lastValidRange Update
	uint32_t curMs = millis();
	//if (pTofc1->data.deviceError == VL53L0X_DEVICEERROR_RANGECOMPLETE)
	if (pTofc1->lastValidRangeHealthy)
	{
		//	pTofc1->lastValidRange = pTofc1->data.range;
		pTofc1->lastValidRange = filtedRange;
		pTofc1->lastValidRangeTime = curMs;
	}
	else if ((curMs - pTofc1->lastValidRangeTime) > pTofc1->config.validRangeKeepMs)
	{
		pTofc1->lastValidRange = VL53L0X_OUT_OF_RANGE;
		pTofc1->lastValidRangeTime = curMs;
	}
}


void tofcUpdate(void)
{
	//millis();
	for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	{
		tofc_t *pCurTofc = &tofc[i];
		if (pCurTofc->enable == false)
		{
			continue;
		}

		//tofc_t *pCurTofc;

		tofcI2cReadData(pCurTofc);
		tofcValidRangeUpdate(pCurTofc);


#ifdef DEBUG_TOF
		uint16_t debugOutDist = pCurTofc->lastValidRange;
		if (debugOutDist <= VL53L0X_MIN_OF_RANGE)
		{
			debug[2 + i] = debugOutDist;
		}
		else if(debugOutDist > VL53L0X_OUT_OF_RANGE)
		{
			//TODO #20160830%phis102 : 將debug輸出改為tof專用輸出(5通道)
			debug[2 + i] = VL53L0X_OUT_OF_RANGE;
		}
		else
		{
			debug[2 + i] = debugOutDist;
		}
		//debug[3 + i] = tofc[i].data.deviceError;
#endif
	}
}

bool tofcIsValidRange(tofc_t tofc1)
{
	uint16_t tmp = tofc1.lastValidRange;
	return tmp > tofc1.config.inValidRangeBoundaryLow && tmp < tofc1.config.inValidRangeBoundaryHigh;
}



//ALT HOLD --------------------------------------------------------------------------------------------


bool tofcIsAltitudeEnable(void)
{
	return (TOFC_DEVICE_COUNT > TOFC_ALIGN_BOTTOM && tofc[TOFC_ALIGN_BOTTOM].enable == true);
}
bool tofcGetAltitudeSensor(tofc_t* getAltTofcPtr)
{
	bool IsAltitudeEnable = tofcIsAltitudeEnable();
	if (IsAltitudeEnable)
	{
		*getAltTofcPtr = tofc[TOFC_ALIGN_BOTTOM];
	}
	return IsAltitudeEnable;
}

//cosTiltAngle = getCosTiltAngle()

int32_t tofcGetAltitudeMm(float cosTiltAngle)
{
	uint32_t calculatedAltitude = 0;
	if (tofcIsAltitudeEnable())
	{
		tofc_t altTofc = tofc[TOFC_ALIGN_BOTTOM];
		uint16_t tofcAltMm = altTofc.lastValidRange;

		// calculate tofc altitude only if the ground is in the tofc cone
		uint16_t tiltCos = cos_approx(225U / 10.0f * RAD);
		if (cosTiltAngle <= tiltCos)
			calculatedAltitude = altTofc.config.inValidRangeBoundaryLow;
		else
			// altitude = distance * cos(tiltAngle), use approximation
			calculatedAltitude = tofcAltMm * cosTiltAngle;
	}
	return calculatedAltitude;
}

int32_t calculateTofcAltHoldThrottleAdjustment(uint16_t AltHoldCm, uint16_t EstAltCm,int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
	int32_t result = 0;
	int32_t error;
	int32_t setVel;

	// Altitude P-Controller

	error = constrain(AltHoldCm - EstAltCm, -500, 500);
	error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
	setVel = constrain((pidProfile()->P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s

	// Velocity PID-Controller

	// P
	error = setVel - vel_tmp;
	result = constrain((pidProfile()->P8[PIDVEL] * error / 32), -300, +300);

	// I
	tofcErrorVelocityI += (pidProfile()->I8[PIDVEL] * error);
	tofcErrorVelocityI = constrain(tofcErrorVelocityI, -(8192 * 200), (8192 * 200));
	result += tofcErrorVelocityI / 8192;     // I in range +/-200

	// D
	result -= constrain(pidProfile()->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

	return result;
}

// Avoidance Mode -----------------------------

//update TOFC_angle for AvoidanceMode
void updateTofcStateForAvoidanceMode(void)
{
	static uint32_t previousTime;
	int32_t TOFC_AltHoldThrottleAdjustment = 0;
	uint32_t dTime;

	int32_t vel_tmp;
	float accZ_tmp;
	static float accZ_old = 0.0f;
	float vel_acc;
	static float vel = 0.0f;

	static uint16_t lastDist = 0;

	float avoidP = 0;
	uint16_t avoidAngle = 0;

	TOFC_angle[AI_ROLL] = 0;
	TOFC_angle[AI_PITCH] = 0;


	//TODO #20160926%phis117 高度與平面的避障參數命名未統一
	//TODO #20160921%phis113 若演算法已決定,需將參數設定移至GUI
	uint16_t startAvoidThres = 1200;   // dist mm 大於等於此距離不迴避(VL53L0X標準檢測距離1.2m 長距離2m)
	uint16_t startAvoidAngle = 40;    // 1/10 degree 起始迴避區段的最大傾角
	uint16_t midAvoidAngleThres = 900;   // dist mm 距離多近時迴避角度達到minAvoidAngle
	uint16_t midAvoidAngle = 80;    // 1/10 degree 中距迴避區段的最大傾角
	uint16_t maxAvoidAngleThres = 400;   // dist mm 距離多近時迴避角度達到maxAvoidAngle
	uint16_t maxAvoidAngle = 80;    // 1/10 degree 緊急迴避區段內的最大傾角
	uint16_t endAvoidThres = 20;   // dist mm  小於等於此距離不迴避(雷射檢測失敗時距離會回傳20)

								   //TODO #20160926%phis116 雷射避障模式高度設定:若演算法已決定,需將參數設定移至GUI
	uint16_t startAltAvoidThresMm = 900;
	//uint16_t midAltAvoidThresMm = 750;
	//uint16_t closeAltAvoidThresMm = 100;
	//uint16_t endAltAvoidThresMm = 100;

	//close #20160831%phis103 需改為支援多感測器
	for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	{
		if (tofc[i].enable == true)
		{
			uint16_t dist = tofc[i].lastValidRange; // mm

			if (dist < startAvoidThres && dist > endAvoidThres)
			{
				if (i == TOFC_ALIGN_BOTTOM)
				{
					uint32_t currentTime = micros();
					dTime = currentTime - previousTime;
					previousTime = currentTime;

					//加速度計資訊計算(定高時考量慣性)
					uint16_t tofcVel = 0;
					if (accSumCount) {
						accZ_tmp = (float)accSum[2] / (float)accSumCount;
					}
					else {
						accZ_tmp = 0;
					}
					vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

					//雷射速度資訊計算(定高時考量慣性)
					tofcVel = (dist - lastDist)/10 * 1000000.0f / dTime; //Cm
					lastDist = tofcVel;

					tofcVel = constrain(tofcVel, -600, 600);  // constrain tofc velocity +/- 600cm/s
					tofcVel = applyDeadband(tofcVel, 10);       // to reduce noise near zero

					float config_baro_cf_vel = 0.985;
					vel = vel_acc * config_baro_cf_vel + tofcVel * (1.0f - config_baro_cf_vel);
					vel_tmp = lrintf(vel);

					//引入althold定高PID計算油門量
					TOFC_AltHoldThrottleAdjustment = calculateTofcAltHoldThrottleAdjustment(startAltAvoidThresMm / 10, dist / 10, vel_tmp, accZ_tmp, accZ_old);
					accZ_old = accZ_tmp;

					avoidP = TOFC_AltHoldThrottleAdjustment;
					//uint16_t altError;
					//altError = constrain(startAltAvoidThresMm - dist, -500, 500); //mm
					//altError = applyDeadband(altError, 10) / 10; //Cm  remove small P parameter to reduce noise near zero position
					//avoidP = constrain((pidProfile()->P8[PIDALT] * altError / 128), -30, +30); // limit velocity to +/- 30 cm/s
					rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE] + avoidP, motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);

					debug[0] = rcCommand[THROTTLE];
					debug[1] = TOFC_AltHoldThrottleAdjustment;
				}
				else
				{
					if (dist > midAvoidAngleThres)
					{
						//緩速(前速)迴避區段 startAvoidThres ~ midAvoidAngleThres
						avoidP = (startAvoidThres - dist) / (float)(startAvoidThres - midAvoidAngleThres);
						avoidAngle = (uint16_t)(startAvoidAngle * avoidP);
					}
					else if (dist > maxAvoidAngleThres)
					{
						//中速迴避區段 midAvoidAngleThres ~ maxAvoidAngleThres
						avoidP = (midAvoidAngleThres - dist) / (float)(midAvoidAngleThres - maxAvoidAngleThres);
						avoidAngle = (uint16_t)((midAvoidAngle - startAvoidAngle) * avoidP) + startAvoidAngle;
					}
					else if (dist <= maxAvoidAngleThres)
					{
						//緊急迴避區段 endAvoidThres ~ midAvoidAngleThres
						avoidAngle = maxAvoidAngle;
					}
					else
					{
						//TODO Error
					}
					//close #20160831%phis104 注意此處0與-1需根據tof安裝位置改變
					//close #20160906%phis107 注意第5個tof用於高度感測,不處理姿態
					TOFC_angle[AI_ROLL] += tofAngleAlign[i][AI_ROLL] * avoidAngle;  // 1/10 degree
					TOFC_angle[AI_PITCH] += tofAngleAlign[i][AI_PITCH] * avoidAngle;  // 1/10 degree
				}

			}
		}
	}

	//debug[5] = TOFC_angle[AI_PITCH];
}


#endif







