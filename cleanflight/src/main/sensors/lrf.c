

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>
//#include "build_config.h"

#include "debug.h"

//#include "common/maths.h"
#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"


#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/lrf_vl53l0x.h"

//#include "io/rc_controls.h"
#include "sensors/sensors.h"



#include "sensors/lrf.h"

#ifdef LRF

//#define VL53L0X

//TODO #20160906%phis106 未來改為5個感測器
lrf_t lrf[LRF_DEVICE_COUNT];
uint16_t lrfXsdnGpio[LRF_DEVICE_COUNT]			= { GPIO_Pin_0	,GPIO_Pin_1	,GPIO_Pin_4	,GPIO_Pin_5 };
GPIO_TypeDef* lrfXsdnGpioType[LRF_DEVICE_COUNT] = { GPIOA		,GPIOA		,GPIOB		,GPIOB };


uint16_t lrfAngleAlign[LRF_DEVICE_COUNT][ANGLE_INDEX_COUNT] = {
	// [Lrf index: 0~5] [Roll: 0 / Pitch: 1]
	[LRF_ALIGN_FRONT] = {0,-1}
	,[LRF_ALIGN_RIGHT] = {-1,0}
	,[LRF_ALIGN_BACK] = {0,1}
	,[LRF_ALIGN_LEFT] = {1,0}
	//,[LRF_ALIGN_BOTTOM] = {0,0}
};

int16_t LRF_angle[ANGLE_INDEX_COUNT] = { 0, 0 };    // 測距後避障邏輯輸出的期望傾角
bool lrf_debug_avoidanceMode = false;


void lrfInit(void)
{
	
	//VL53L0X 參數設定
	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		//GPIO_Pin_1 = PA1 藍線 ,GPIO_Pin_0 = PA0 白線
		gpio_config_t gpioCfg;
		gpioCfg.mode = GPIO_Mode_OUT;
		gpioCfg.pin = lrfXsdnGpio[i];
		//if (i==1)
		//{
		//	gpioCfg.pin = GPIO_Pin_2;
		//}
		gpioCfg.speed = 1;

		lrfDevice_t curLrfDevice;
		curLrfDevice.i2cXsdnGpioCfg = gpioCfg;
		curLrfDevice.i2cXsdnGpioType = lrfXsdnGpioType[i];
		curLrfDevice.i2cAddr = LRF_DEVICE_START_ADDR + i;

		lrfData_t curLrfData = {0};
		lrf_t curLrf = {
			.device = curLrfDevice,
			.data = curLrfData,
			.enable = true
		};
		//curLrf.device = curLrfDevice;
		//curLrf.data = curLrfData;
		//curLrf.enable = true;
		lrf[i] = curLrf;
	}

	//LRF裝置初始化
	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		lrfDevice_t curLrfDevice = lrf[i].device;
		//lrf_vl53l0x_Init(i);
		lrf[i].enable = lrf_vl53l0x_i2c_init(curLrfDevice.i2cXsdnGpioType, curLrfDevice.i2cXsdnGpioCfg, curLrfDevice.i2cAddr);
	}
	sensorsSet(SENSOR_LRF);
}


uint16_t makeuint16____(uint16_t lsb, uint16_t msb)
{
	return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

//update LRF_angle for AvoidanceMode
void updateLrfStateForAvoidanceMode(void)
{
	LRF_angle[AI_ROLL] = 0;
	LRF_angle[AI_PITCH] = 0;

	uint16_t startAvoidThres = 1200;   // dist mm 大於此距離不迴避(VL53L0X標準檢測距離1.2m)
	uint16_t startAvoidAngle = 40;    // 1/10 degree 起始迴避區段的最大傾角
	uint16_t midAvoidAngleThres = 700;   // dist mm 距離多近時迴避角度達到minAvoidAngle
	uint16_t midAvoidAngle = 80;    // 1/10 degree 中距迴避區段的最大傾角
	uint16_t maxAvoidAngleThres = 200;   // dist mm 距離多近時迴避角度達到maxAvoidAngle
	uint16_t maxAvoidAngle = 80;    // 1/10 degree 緊急迴避區段內的傾角

	uint16_t endAvoidThres = 20;   // dist mm  小於此距離不迴避(雷射檢測失敗時距離會回傳20)

	//close #20160831%phis103 需改為支援多感測器
	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		if (lrf[i].enable == true)
		{
			uint16_t dist = lrf[i].data.range; // mm
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
				//close #20160831%phis104 注意此處0與-1需根據LRF安裝位置改變
				//TODO #20160906%phis107 注意第5個LRF用於高度感測,不處理姿態
				LRF_angle[AI_ROLL] += lrfAngleAlign[i][AI_ROLL] * avoidAngle;  // 1/10 degree
				LRF_angle[AI_PITCH] += lrfAngleAlign[i][AI_PITCH] * avoidAngle;  // 1/10 degree
			}
		}
	}

	//debug[5] = LRF_angle[AI_PITCH];
}

void lrfUpdate(void)
{
	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		if (lrf[i].enable == false)
		{
			continue;
		}

		uint8_t in_addr = 0x30;//0x29 0x2C
		in_addr = lrf[i].device.i2cAddr;
		uint8_t VL53L0X_REG_buf[12];
		for (int i = 0; i < 12; i++)
			VL53L0X_REG_buf[i] = 0;
		i2cRead(in_addr, VL53L0X_REG_RESULT_RANGE_STATUS, 12, VL53L0X_REG_buf);

		uint16_t dist = makeuint16____(VL53L0X_REG_buf[11], VL53L0X_REG_buf[10]);
		lrf[i].data.range = dist;

		if (dist <= VL53L0X_MIN_OF_RANGE)
		{
			debug[2 + i] = dist;
		}
		else if(dist > 3000)
		{
			//TODO #20160830%phis102 : 將debug輸出改為LRF專用輸出(5通道)
			debug[2 + i] = 3000;
		}
		else
		{
			debug[2 + i] = dist;
		}

		i2cWrite(in_addr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP);
	}
	

}


#endif







