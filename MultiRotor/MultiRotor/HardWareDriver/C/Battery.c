/*


Battery.c file
编写者：WJ
作者E-mail:cqrg01@163.com
编译环境：MDK  Version: v5.15
初版时间: 2016-5-13
功能：
1.电池检测AD初始化
2.供低压检测用，提供片内温度传感器驱动
------------------------------------
*/

#include "Battery.h"
#include "config.h"
//#include "UART1.h"
//#include "stdio.h"
//#include "CommApp.h"
//#include "ReceiveData.h"

extern ADC_HandleTypeDef hadc1;

//实例化一个电压信息结构体
Bat_Typedef Battery;

uint8_t FLY_ENABLE=0;//飞行使能端  7/-5    14/15


void BatteryCheckInit()
{
	
	HAL_ADCEx_Calibration_Start(&hadc1);//ADC校准

	Battery.BatReal = 3.95;//单位为v 电池实际电压  校准电压时修改
	Battery.ADinput = 1.98;//单位为v R15和R17连接处电压 校准电压时修改
	Battery.ADRef   = 3.26;//单位为v 单片机供电电压   校准电压时修改
	Battery.Bat_K   = Battery.BatReal / Battery.ADinput; //计算电压计算系数
	Battery.BatteryADmin = 2000;//电压门限AD值
	printf("电压检测AD初始化完成...\r\n");
}

uint16_t Get_Adc()
{
	uint32_t temp_Adc;
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		/* Start Conversation Error */
	}
	if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
	}
	else
	{
		/* ADC conversion completed */
		/*##-5- Get the converted value of regular channel  ########################*/
		temp_Adc= HAL_ADC_GetValue(&hadc1);
	}
	return temp_Adc;
}

uint16_t Get_Adc_Average(uint8_t times)
{
	uint32_t temp_val = 0;
	uint8_t t;
	for (t = 0; t < times; t++)
	{
		temp_val += Get_Adc();
	}
	return temp_val / times;
}

//返回电池电压AD值
int GetBatteryAD()
{
 return Get_Adc_Average(8);
}

//检测电池电压
void BatteryCheck(void)
{
	Battery.BatteryAD  = GetBatteryAD();            //电池电压检测
	Battery.BatteryVal = Battery.Bat_K * (Battery.BatteryAD / 4096.0) * Battery.ADRef; //实际电压值计算
if(FLY_ENABLE)
		{
			if(Battery.BatteryAD <= Battery.BatteryADmin)
			{
					Battery.alarm=1;
			}
			else
					Battery.alarm=0;
		}
		else
		{
			if((Battery.BatteryVal < BAT_ALARM_VAL)&&(Battery.BatteryVal > BAT_CHG_VAL))	//低于3.7v 且大于充电检测电压 BAT_CHG_VAL
				Battery.alarm=1;
			else
				Battery.alarm=0;
		}
				if(Battery.BatteryVal < BAT_CHG_VAL) //on charge
		{
			Battery.chargeSta = 1; 
		}
		else 					
			Battery.chargeSta = 0;
}
