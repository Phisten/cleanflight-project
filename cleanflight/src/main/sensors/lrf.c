

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
lrf_t lrf[LRF_DEVICE_COUNT];

int16_t LRF_angle[ANGLE_INDEX_COUNT] = { 0, 0 };    // it's the angles that must be applied for GPS correction
bool lrf_debug_avoidanceMode = false;

void lrfInit(void)
{
	//laserRanges_t laserRange;

	//hcsr04_init(sonarHardware, &sonarRange);

	//gpio_config_t gpioCfg;
	//gpioCfg.mode = GPIO_Mode_OUT;
	//gpioCfg.pin = GPIO_Pin_1; //PA1

	//uint8_t addr = 0x30;

	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		lrf_vl53l0x_Init(i);
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
	uint16_t midAvoidAngle = 50;    // 1/10 degree 中距迴避區段的最大傾角
	uint16_t maxAvoidAngleThres = 200;   // dist mm 距離多近時迴避角度達到maxAvoidAngle
	uint16_t maxAvoidAngle = 50;    // 1/10 degree 緊急迴避區段內的傾角

	uint16_t endAvoidThres = 20;   // dist mm  小於此距離不迴避(雷射檢測失敗時距離會回傳20)

	//TODO #20160831%phis103 需改為支援多感測器
	uint16_t dist = lrf[0].data.range; // mm
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
		//TODO #20160831%phis104 注意此處0與-1需根據LRF安裝位置改變
		LRF_angle[AI_ROLL] += 0 * avoidAngle;  // 1/10 degree
		LRF_angle[AI_PITCH] += -1 * avoidAngle;  // 1/10 degree
	}
	debug[5] = LRF_angle[AI_PITCH];
}

void lrfUpdate(void)
{
	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		uint8_t in_addr = 0x30;//0x29 0x2C
		uint8_t VL53L0X_REG_buf[12];
		for (int i = 0; i < 12; i++)
			VL53L0X_REG_buf[i] = 0;
		i2cRead(in_addr, VL53L0X_REG_RESULT_RANGE_STATUS, 12, VL53L0X_REG_buf);

		uint16_t dist = makeuint16____(VL53L0X_REG_buf[11], VL53L0X_REG_buf[10]);
		lrf[i].data.range = dist;

		if (dist <= 20)
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
		//if (lrf.Enable[i])
		//{
		//}
	}
	

}


#endif







