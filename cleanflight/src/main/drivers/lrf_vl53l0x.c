#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"

#include "drivers/system.h"

#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"


#include "sensors/lrf.h"
#include "drivers/lrf_vl53l0x.h"

char* VL53L0X_DeviceErrorString[] = {
	VL53L0X_STRING_DEVICEERROR_NONE , //  0  NoError  ���m?
	VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE , //VCSEL = �����ĭ��o�g�E�����]Vertical-Cavity Surface-Emitting Laser)
	VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE , //watchdog �ݪ����p�ɾ��O�@�عq���w�馡���p�ɸ˸m�A��t�Ϊ��D�{���o�ͬY�ǿ��~�ƥ��
	//		   �A�ݪ����p�ɾ��N�|��t�εo�X���]�B���s�}�����������H���A�Ϩt�αq�a�����A�^�_�쥿�`�B�@���A�C
	VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND ,
	VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET , // MSRC = Minimum Signal Rate Check function
	VL53L0X_STRING_DEVICEERROR_SNRCHECK , // SNR = Signal-to-noise ratio
	VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK ,
	VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK ,
	VL53L0X_STRING_DEVICEERROR_TCC , //Target CentreCheck. = 8
	VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY ,
	VL53L0X_STRING_DEVICEERROR_MINCLIP ,
	VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE , //�q������ = 11
	VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW , //ALGO=algorithm(�t��k) ���T�w
	VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW ,
	VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD
};
int VL53L0X_DeviceErrorCount = 15;

void lrf_vl53l0x_Init(int lrfIndex)
{
	UNUSED(lrfIndex);
	//TODO ��l�ưѼ����ʸ˦�lrf���c
	//int i = lrfIndex;

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

	//TODO ���ѨM���涥�q���~
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


bool lrf_vl53l0x_i2c_init(GPIO_TypeDef* gpioType, gpio_config_t gpioCfg , uint8_t i2cAddr)
{
	gpioInit(gpioType, &gpioCfg);
	uint16_t xsdnPin = gpioCfg.pin;

	digitalLo(gpioType, xsdnPin);
	delay(10);
	digitalHi(gpioType, xsdnPin);
	delay(10);

	uint8_t in_addr = VL53L0X_DEVICE_DEAFULT_ADDR; //default
	if (i2cWrite(in_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, i2cAddr))
	{
		//uint8_t SLAVE_DEVICE_ADDRESS;
		//i2cRead(i2cAddr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, 1, &SLAVE_DEVICE_ADDRESS);

		//if (SLAVE_DEVICE_ADDRESS == i2cAddr)
		//{
			// enable VL53L0X  backtoback  range mode
			i2cWrite(i2cAddr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK | VL53L0X_REG_SYSRANGE_MODE_START_STOP);
			return true;
		//}
	}

	return false;
}

void lrf_vl53l0x_long_range_mode()
{


}