

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
#include "drivers/tof_vl53l0x.h"
#include "drivers/system.h"

//#include "io/rc_controls.h"
#include "sensors/sensors.h"



#include "sensors/tof.h"

#ifdef TOF

//#define VL53L0X

//TODO #20160906%phis106 ���ӧאּ5�ӷP����
tof_t tof[TOF_DEVICE_COUNT];
uint16_t tofXsdnGpio[TOF_DEVICE_COUNT]			= { GPIO_Pin_0	,GPIO_Pin_1	,GPIO_Pin_4	,GPIO_Pin_5 };
GPIO_TypeDef* tofXsdnGpioType[TOF_DEVICE_COUNT] = { GPIOA		,GPIOA		,GPIOB		,GPIOB };

uint32_t tofCurrentTimeMs = 0;

uint16_t tofAngleAlign[TOF_DEVICE_COUNT][ANGLE_INDEX_COUNT] = {
	// [tof index: 0~4] [Roll: 0 / Pitch: 1]
	[tof_ALIGN_FRONT] = {0,-1}
	,[tof_ALIGN_RIGHT] = {-1,0}
	,[tof_ALIGN_BACK] = {0,1}
	,[tof_ALIGN_LEFT] = {1,0}
	,[tof_ALIGN_BOTTOM] = {0,0}
};

int16_t tof_angle[ANGLE_INDEX_COUNT] = { 0, 0 };    // ���Z���׻��޿��X������ɨ�
bool tof_debug_avoidanceMode = false;


uint16_t makeuint16____(uint16_t lsb, uint16_t msb)
{
	return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}



void tofInit(void)
{
	
	//VL53L0X �ѼƳ]�w
	for (int i = 0; i < TOF_DEVICE_COUNT; i++)
	{
		//GPIO_Pin_1 = PA1 �Žu ,GPIO_Pin_0 = PA0 �սu
		gpio_config_t gpioCfg;
		gpioCfg.mode = GPIO_Mode_OUT;
		gpioCfg.pin = tofXsdnGpio[i];
		//if (i==1)
		//{
		//	gpioCfg.pin = GPIO_Pin_2;
		//}
		gpioCfg.speed = 1;

		tofDevice_t curtofDevice;
		curtofDevice.i2cXsdnGpioCfg = gpioCfg;
		curtofDevice.i2cXsdnGpioType = tofXsdnGpioType[i];
		curtofDevice.i2cAddr = TOF_DEVICE_START_ADDR + i;

		tofData_t curtofData;
		tof_t curtof = {
			.device = curtofDevice,
			.data = curtofData,
			.enable = true
		};
		//curtof.device = curtofDevice;
		//curtof.data = curtofData;
		//curtof.enable = true;
		tof[i] = curtof;
	}

	//tof�˸m��l��
	for (int i = 0; i < TOF_DEVICE_COUNT; i++)
	{
		tofDevice_t curtofDevice = tof[i].device;
		//tof_vl53l0x_Init(i);
		tof[i].enable = tof_vl53l0x_i2c_init(curtofDevice.i2cXsdnGpioType, curtofDevice.i2cXsdnGpioCfg, curtofDevice.i2cAddr);
	}
	sensorsSet(SENSOR_tof);
}

//update tof_angle for AvoidanceMode
void updatetofStateForAvoidanceMode(void)
{
	tof_angle[AI_ROLL] = 0;
	tof_angle[AI_PITCH] = 0;

	uint16_t startAvoidThres = 2000;   // dist mm �j�󦹶Z�����j��(VL53L0X�з��˴��Z��1.2m ���Z��2m)
	uint16_t startAvoidAngle = 40;    // 1/10 degree �_�l�j�װϬq���̤j�ɨ�
	uint16_t midAvoidAngleThres = 1000;   // dist mm �Z���h��ɰj�ר��׹F��minAvoidAngle
	uint16_t midAvoidAngle = 80;    // 1/10 degree ���Z�j�װϬq���̤j�ɨ�
	uint16_t maxAvoidAngleThres = 400;   // dist mm �Z���h��ɰj�ר��׹F��maxAvoidAngle
	uint16_t maxAvoidAngle = 80;    // 1/10 degree ���j�װϬq�����̤j�ɨ�

	uint16_t endAvoidThres = 20;   // dist mm  �p�󦹶Z�����j��(�p�g�˴����ѮɶZ���|�^��20)

	//close #20160831%phis103 �ݧאּ�䴩�h�P����
	for (int i = 0; i < TOF_DEVICE_COUNT; i++)
	{
		if (tof[i].enable == true)
		{
			uint16_t dist = tof[i].data.range; // mm
			float avoidP = 0;
			uint16_t avoidAngle = 0;
			if (dist < startAvoidThres && dist > endAvoidThres)
			{
				if (dist > midAvoidAngleThres)
				{
					//�w�t(�e�t)�j�װϬq startAvoidThres ~ midAvoidAngleThres
					avoidP = (startAvoidThres - dist) / (float)(startAvoidThres - midAvoidAngleThres);
					avoidAngle = (uint16_t)(startAvoidAngle * avoidP);
				}
				else if (dist > maxAvoidAngleThres)
				{
					//���t�j�װϬq midAvoidAngleThres ~ maxAvoidAngleThres
					avoidP = (midAvoidAngleThres - dist) / (float)(midAvoidAngleThres - maxAvoidAngleThres);
					avoidAngle = (uint16_t)((midAvoidAngle - startAvoidAngle) * avoidP) + startAvoidAngle;
				}
				else if (dist <= maxAvoidAngleThres)
				{
					//���j�װϬq endAvoidThres ~ midAvoidAngleThres
					avoidAngle = maxAvoidAngle;
				}
				else
				{
					//TODO Error

				}
				//close #20160831%phis104 �`�N���B0�P-1�ݮھ�tof�w�˦�m����
				//TODO #20160906%phis107 �`�N��5��tof�Ω󰪫׷P��,���B�z���A
				tof_angle[AI_ROLL] += tofAngleAlign[i][AI_ROLL] * avoidAngle;  // 1/10 degree
				tof_angle[AI_PITCH] += tofAngleAlign[i][AI_PITCH] * avoidAngle;  // 1/10 degree
			}
		}
	}

	//debug[5] = tof_angle[AI_PITCH];
}


void tofUpdate(void)
{
	//millis();
	for (int i = 0; i < TOF_DEVICE_COUNT; i++)
	{
		if (tof[i].enable == false)
		{
			continue;
		}

		uint8_t in_addr = 0x30;//0x29 0x2C
		in_addr = tof[i].device.i2cAddr;
		uint8_t VL53L0X_REG_buf[12];
		for (int i = 0; i < 12; i++)
			VL53L0X_REG_buf[i] = 0;
		i2cRead(in_addr, VL53L0X_REG_RESULT_RANGE_STATUS, 12, VL53L0X_REG_buf);

		uint16_t dist = makeuint16____(VL53L0X_REG_buf[11], VL53L0X_REG_buf[10]);
		tof[i].data.range = dist;
		tof[i].data.rangeStatus = VL53L0X_REG_buf[0];
		tof[i].data.deviceError = (VL53L0X_REG_buf[0] & 0x78) >> 3;
		tof[i].data.signalRate = (uint32_t)makeuint16____(VL53L0X_REG_buf[7], VL53L0X_REG_buf[6]);
		tof[i].data.ambientRate = makeuint16____(VL53L0X_REG_buf[9], VL53L0X_REG_buf[8]);
		tof[i].data.effectiveSpadRtnCount = makeuint16____(VL53L0X_REG_buf[3], VL53L0X_REG_buf[2]);


#ifdef DEBUG_tof
		if (dist <= VL53L0X_MIN_OF_RANGE)
		{
			debug[2 + i] = dist;
		}
		else if(dist > 3000)
		{
			//TODO #20160830%phis102 : �Ndebug��X�אּtof�M�ο�X(5�q�D)
			debug[2 + i] = 3000;
		}
		else
		{
			debug[2 + i] = dist;
		}
		//debug[3 + i] = tof[i].data.deviceError;
#endif


		//i2cWrite(in_addr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP);
	}
	

}


#endif







