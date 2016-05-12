/*------------------------------------
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \|     |//  `.
            /  \|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
           佛祖保佑       永无BUG

文件名：Led.h
编写者：WJ
作者E-mail：cqrg01@163.com
编译环境：MDK 5.0
版本号：1.0
初版时间:2016-5-10
修改时间：
说明：
1.飞控上led IO口初始化
2.初始化默认关闭所有LED灯
------------------------------------*/
#include "Led.h"
//#include "config.h"
////#include "UART1.h"

LED_t LEDCtrl;
//接口显存
LEDBuf_t LEDBuf;

/****************************************************************************
*  函数名称:  Led初始化函数
*  功    能:  1.配置Led接口IO输出方向
              2.关闭所有Led(开机默认方式)
*  入口参数:
*  返    回:
*  依    赖:
*  作    者:  WJ
*  时    间:  2016-5-10 20:34:08
*  修改时间:
*  说    明:  Led接口：
              Led1-- > PB12
              Led2-- > PB13
              对应IO = 0，灯亮
****************************************************************************/

void LedInit(void)
{
  LedA_off;
  LedB_off;
  printf("状态LED灯初始化完成...\r\n");
}

//底层更新 ，10Hz
void LEDReflash(void)
{
  if (LEDBuf.bits.A)
    LedA_on;
  else
    LedA_off;

  if (LEDBuf.bits.B)
    LedB_on;
  else
    LedB_off;
}

//事件驱动层
void LEDFSM(void)
{
//  static uint16_t cnt=0;
//  uint8_t event=0;

  switch (LEDCtrl.event)
  {
  case E_READY:
    if (++LEDCtrl.cnt >= 3)   //0 1 2 in loop, 0 on ,else off
      LEDCtrl.cnt = 0;
    if (LEDCtrl.cnt == 0)
      LEDBuf.byte = LA | LB;
    else
      LEDBuf.byte = 0;
    break;
  case E_CALI:
    LEDBuf.byte = LA | LB;
    break;
  case E_BAT_LOW:
    if (++LEDCtrl.cnt >= 3)   //0 1  in loop
      LEDCtrl.cnt = 0;
    if (LEDCtrl.cnt == 0)
      LEDBuf.byte = 0x03;
    else
      LEDBuf.byte = 0;
    break;
  case E_CALI_FAIL:
    if (++LEDCtrl.cnt >= 4)
      LEDCtrl.cnt = 0;
    if (LEDCtrl.cnt < 2)
      LEDBuf.byte = LA ;
    else
      LEDBuf.byte = LB;
    break;
  case E_LOST_RC:
    if (++LEDCtrl.cnt >= 4)
      LEDCtrl.cnt = 0;
    LEDBuf.byte = 1 << LEDCtrl.cnt ;
//        if(LEDCtrl.cnt==0)
//            LEDBuf.byte =LA;
//        else
//            LEDBuf.byte =0;
    break;
  case E_AUTO_LANDED:
    LEDBuf.byte = 0x03;
    break;

  case E_BatChg:
    LEDBuf.byte = 0x00;
    break;

  }

  LEDReflash();
}
