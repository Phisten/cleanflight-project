#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"

#include "drivers/system.h"

#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"


#include "sensors/tofc.h"
#include "drivers/tofc_vl53l0x.h"

char* VL53L0X_DeviceErrorString[] = {
	VL53L0X_STRING_DEVICEERROR_NONE , //  0  NoError  閒置?
	VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE , //VCSEL = 垂直腔面發射激光器（Vertical-Cavity Surface-Emitting Laser)
	VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE , //watchdog 看門狗計時器是一種電腦硬體式的計時裝置，當系統的主程式發生某些錯誤事件時
	//		   ，看門狗計時器就會對系統發出重設、重新開機或關閉的信號，使系統從懸停狀態回復到正常運作狀態。
	VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND ,
	VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET , // MSRC = Minimum Signal Rate Check function
	VL53L0X_STRING_DEVICEERROR_SNRCHECK , // SNR = Signal-to-noise ratio
	VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK ,
	VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK ,
	VL53L0X_STRING_DEVICEERROR_TCC , //Target CentreCheck. = 8
	VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY ,
	VL53L0X_STRING_DEVICEERROR_MINCLIP ,
	VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE , //量測完成 = 11
	VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW , //ALGO=algorithm(演算法) 不確定
	VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW ,
	VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD
};
int VL53L0X_DeviceErrorCount = 15;

bool tofc_vl53l0x_i2c_init(GPIO_TypeDef* gpioType, gpio_config_t gpioCfg , uint8_t i2cAddr)
{
	gpioInit(gpioType, &gpioCfg);
	uint16_t xsdnPin = gpioCfg.pin;

	delay(10);
	digitalLo(gpioType, xsdnPin);
	delay(10);
	digitalHi(gpioType, xsdnPin);
	delay(10);

	uint8_t in_addr = VL53L0X_DEVICE_DEAFULT_ADDR; //default
	uint8_t SLAVE_DEVICE_ADDRESS = 0xFF;
	//設定新Addr
	if (i2cWrite(in_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, i2cAddr))
	{
		delay(10);
		//要求返回Addr
		if (i2cRead(i2cAddr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, 1, &SLAVE_DEVICE_ADDRESS))
			//確認Addr是否成功設置
			if (SLAVE_DEVICE_ADDRESS == i2cAddr) 
			{
				//啟動連續測距
				delay(10);
				return i2cWrite(i2cAddr, VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK | VL53L0X_REG_SYSRANGE_MODE_START_STOP);
			}
	}
	return false;
}

void tofc_vl53l0x_long_range_mode()
{


}