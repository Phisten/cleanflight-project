

#include <stdbool.h>
#include <stdint.h>
//#include <math.h>

#include <platform.h>
//#include "build_config.h"

//#include "common/maths.h"
//#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"


#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/lrf_vl53l0x.h"

//#include "io/rc_controls.h"
#include "sensors/sensors.h"

#include "debug.h"


#include "sensors/lrf.h"

//int16_t sonarMaxRangeCm;
//int16_t sonarMaxAltWithTiltCm;
//int16_t sonarCfAltCm; // Complimentary Filter altitude
//STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;
//float sonarMaxTiltCos;
#ifdef LRF

//#define VL53L0X

//uint8_t lrfDeviceAddr[] = { 0x30 };
//int lrfDeviceCount = 1;

//LaserRangeFinder access functions

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
void lrfUpdate(void)
{
	//TODO 
	for (int i = 0; i < LRF_DEVICE_COUNT; i++)
	{
		uint8_t in_addr = 0x30;//0x29 0x2C
		uint8_t VL53L0X_REG_buf[12];
		for (int i = 0; i < 12; i++)
			VL53L0X_REG_buf[i] = 0;
		i2cRead(in_addr, VL53L0X_REG_RESULT_RANGE_STATUS, 12, VL53L0X_REG_buf);

		uint16_t dist = makeuint16____(VL53L0X_REG_buf[11], VL53L0X_REG_buf[10]);
		if (dist <= 20)
		{

		}
		else if(dist > 3000)
		{
			debug[2 + i] = 3000;
		}
		else
		{
			debug[2 + i] = dist;
		}

		i2cWrite(in_addr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP);
		if (lrf.Enable[i])
		{
		}
	}
	

}


#endif







