

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


#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/tofc_vl53l0x.h"
#include "drivers/system.h"

//#include "io/rc_controls.h"
#include "sensors/sensors.h"
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

int16_t TOFC_angle[ANGLE_INDEX_COUNT] = { 0, 0 };    // 測距後避障邏輯輸出的期望傾角
bool tofc_debug_avoidanceMode = false;


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
		//if (i==1)
		//{
		//	gpioCfg.pin = GPIO_Pin_2;
		//}
		gpioCfg.speed = 1;

		tofcDevice_t curtofDevice;
		curtofDevice.i2cXsdnGpioCfg = gpioCfg;
		curtofDevice.i2cXsdnGpioType = tofXsdnGpioType[i];
		curtofDevice.i2cAddr = TOFC_DEVICE_START_ADDR + i;

		tofcData_t curtofData = {0};

		tofcConfig_t curtofcCfg = {
			.validRangeKeepMs = 200,
			.minValidRange = VL53L0X_MIN_OF_RANGE,
			.maxValidRange = VL53L0X_OUT_OF_RANGE
		};

		tofc_t curtof = {
			.enable = false,
			.device = curtofDevice,
			.data = curtofData,
			.config = curtofcCfg,

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

//update TOFC_angle for AvoidanceMode
void updateTofcStateForAvoidanceMode(void)
{
	TOFC_angle[AI_ROLL] = 0;
	TOFC_angle[AI_PITCH] = 0;

	uint16_t startAvoidThres = 2000;   // dist mm 大於此距離不迴避(VL53L0X標準檢測距離1.2m 長距離2m)
	uint16_t startAvoidAngle = 40;    // 1/10 degree 起始迴避區段的最大傾角
	uint16_t midAvoidAngleThres = 1000;   // dist mm 距離多近時迴避角度達到minAvoidAngle
	uint16_t midAvoidAngle = 80;    // 1/10 degree 中距迴避區段的最大傾角
	uint16_t maxAvoidAngleThres = 400;   // dist mm 距離多近時迴避角度達到maxAvoidAngle
	uint16_t maxAvoidAngle = 80;    // 1/10 degree 緊急迴避區段內的最大傾角

	uint16_t endAvoidThres = 20;   // dist mm  小於此距離不迴避(雷射檢測失敗時距離會回傳20)

	//close #20160831%phis103 需改為支援多感測器
	for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	{
		if (tofc[i].enable == true)
		{
			uint16_t dist = tofc[i].data.range; // mm
			float avoidP = 0;
			uint16_t avoidAngle = 0;
			if (dist < startAvoidThres && dist > endAvoidThres)
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
				//TODO #20160906%phis107 注意第5個tof用於高度感測,不處理姿態
				TOFC_angle[AI_ROLL] += tofAngleAlign[i][AI_ROLL] * avoidAngle;  // 1/10 degree
				TOFC_angle[AI_PITCH] += tofAngleAlign[i][AI_PITCH] * avoidAngle;  // 1/10 degree
			}
		}
	}

	//debug[5] = TOFC_angle[AI_PITCH];
}


void tofcUpdate(void)
{
	//millis();
	for (int i = 0; i < TOFC_DEVICE_COUNT; i++)
	{
		if (tofc[i].enable == false)
		{
			continue;
		}

		uint8_t in_addr = 0x29;//0x29 0x2C
		in_addr = tofc[i].device.i2cAddr;
		uint8_t VL53L0X_REG_buf[12];
		uint32_t curMs = millis();

		for (int i = 0; i < 12; i++)
			VL53L0X_REG_buf[i] = 0;
		i2cRead(in_addr, VL53L0X_REG_RESULT_RANGE_STATUS, 12, VL53L0X_REG_buf);

		uint16_t dist = makeuint16____(VL53L0X_REG_buf[11], VL53L0X_REG_buf[10]);
		tofc[i].data.range = dist;
		tofc[i].data.rangeStatus = VL53L0X_REG_buf[0];
		tofc[i].data.deviceError = (VL53L0X_REG_buf[0] & 0x78) >> 3;
		tofc[i].data.signalRate = (uint32_t)makeuint16____(VL53L0X_REG_buf[7], VL53L0X_REG_buf[6]);
		tofc[i].data.ambientRate = makeuint16____(VL53L0X_REG_buf[9], VL53L0X_REG_buf[8]);
		tofc[i].data.effectiveSpadRtnCount = makeuint16____(VL53L0X_REG_buf[3], VL53L0X_REG_buf[2]);

		if (tofc[i].data.deviceError == VL53L0X_DEVICEERROR_RANGECOMPLETE || (tofc[i].lastValidRangeTime - curMs) > tofc[i].config.validRangeKeepMs)
		{
			tofc[i].lastValidRange = dist;
			tofc[i].lastValidRangeTime = curMs;
		}

#ifdef DEBUG_TOF
		if (dist <= VL53L0X_MIN_OF_RANGE)
		{
			debug[2 + i] = dist;
		}
		else if(dist > 3000)
		{
			//TODO #20160830%phis102 : 將debug輸出改為tof專用輸出(5通道)
			debug[2 + i] = 3000;
		}
		else
		{
			debug[2 + i] = dist;
		}
		//debug[3 + i] = tofc[i].data.deviceError;
#endif
		//i2cWrite(in_addr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP);
	}
}

bool tofcIsValidRange(tofc_t tofc1)
{
	uint16_t tmp = tofc1.lastValidRange;
	return tmp > tofc1.config.minValidRange && tmp <= tofc1.config.maxValidRange;
}

//ALT HOLD
bool tofcIsAltitudeEnable(void)
{
	return (TOFC_DEVICE_COUNT > TOFC_ALIGN_BOTTOM && tofc[TOFC_ALIGN_BOTTOM].enable == true);
}

//cosTiltAngle = getCosTiltAngle()
int32_t tofcGetAltitudeCm(float cosTiltAngle)
{
	uint32_t calculatedAltitude = 0;
	if (tofcIsAltitudeEnable())
	{
		tofc_t altTofc = tofc[TOFC_ALIGN_BOTTOM];
		uint16_t tofcAltMm = altTofc.lastValidRange;

		// calculate tofc altitude only if the ground is in the tofc cone
		uint16_t tiltCos = cos_approx(225U / 10.0f * RAD);
		if (cosTiltAngle <= tiltCos)
			calculatedAltitude = altTofc.config.minValidRange;
		else
			// altitude = distance * cos(tiltAngle), use approximation
			calculatedAltitude = tofcAltMm * cosTiltAngle;
	}
	return calculatedAltitude / 10;
}

#endif







