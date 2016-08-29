#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/system.h"

#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/lrf_vl53l0x.h"

#include "debug.h"

#include "sensors/lrf.h"

lrf_t lrf;

void lrf_vl53l0x_Init(int lrfIndex)
{

	//TODO 初始化參數應封裝至lrf結構
	int i = lrfIndex;

	gpio_config_t gpioCfg;
	gpioCfg.mode = GPIO_Mode_OUT;
	gpioCfg.pin = GPIO_Pin_1; //PA1
	gpioInit(GPIOA, &gpioCfg);

	digitalLo(GPIOA, gpioCfg.pin);
	delay(100);
	digitalHi(GPIOA, gpioCfg.pin);
	delay(100);

	uint8_t in_addr = 0x29; //default
	i2cWrite(in_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, 0x30);

	//TODO 未解決執行階段錯誤
	//LRF_DEVICE_DEAFULT_ADDR + i + 1
	//lrf.Enable[i] = true;
	//lrf.device[i]->I2cDevAddr = LRF_DEVICE_DEAFULT_ADDR + i + 1;
	//lrf.device[i]->comms_type = VL53L0X_COMMS_I2C;	/*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
	//lrf.device[i]->comms_speed_khz = 400;           /*!< Comms speed [kHz] : typically 400kHz for I2C           */
	//lrf.i2cXsdnGpioType[i] = GPIOA;
	//lrf.i2cXsdnGpioCfg[i].pin = GPIO_Pin_1 + i;
	//lrf.i2cXsdnGpioCfg[i].mode = GPIO_Mode_OUT;


	//GPIO_TypeDef* gpioType = lrf.i2cXsdnGpioType[i];
	//gpio_config_t* gpioCfg = &lrf.i2cXsdnGpioCfg[i];
	//gpioInit(gpioType, gpioCfg);
	//digitalLo(gpioType, gpioCfg->pin);
	//delay(100);
	//digitalHi(gpioType, gpioCfg->pin);
	//delay(100);

	//uint8_t in_addr = lrf.device[i]->I2cDevAddr; //default
	//i2cWrite(in_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, lrf.device[i]->I2cDevAddr);
	//delay(100);
	//i2cWrite(in_addr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP ); // | VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
}

