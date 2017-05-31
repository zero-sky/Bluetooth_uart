//AT 命令处理
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "at.h"


/************************************************************************/
/* 功能：回复数据
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_cmd_reply(uint8_t *p_data, uint8_t len)
{
	for (uint32_t i = 0; i < len; i++)
	{
			//发送串口数据，若失败则一直发送
		while(app_uart_put(p_data[i]) != NRF_SUCCESS); 
	}	
}

/************************************************************************/
/* 功能：AT命令区分
 * 描述：根据不同的AT命令，使能不同的功能
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_cmd_switch(uint8_t *p)
{
	const char * p_dat= (const char*)p;

	if(strncasecmp(p_dat,"BAUD",4) == 0)			//波特率
	{
		
	}
	else if(strncasecmp(p_dat,"PARI",4) == 0)		//校验位
	{
		
	}
	else if(strncasecmp(p_dat,"STOP",4) == 0)		//停止位
	{}
	else if(strncasecmp(p_dat,"MODE",4) == 0)		//开闭串口透传
	{}
	else if(strncasecmp(p_dat,"NAME",4) == 0)		//设备名称
	{}
	else if(strncasecmp(p_dat,"RENEW",5) == 0)		//恢复默认设置
	{}
	else if(strncasecmp(p_dat,"RESET",4) == 0) 		//复位重启
	{}
	else if(strncasecmp(p_dat,"POLE",4) == 0) 		//主从模式
	{}
	else if(strncasecmp(p_dat,"PASS",4) == 0) 		//配对密码
	{}
	else if(strncasecmp(p_dat,"TYPE",4) == 0)		//鉴权类型
	{}
	else if(strncasecmp(p_dat,"ADDR",4) == 0) 		//MAC地址
	{}
	else if(strncasecmp(p_dat,"CONNL",4) == 0) 		//查询最后一次连接成功的从设备
	{}
	else if(strncasecmp(p_dat,"CON",3) == 0) 		//连接指定的从设备	
	{}
	else if(strncasecmp(p_dat,"CLEAR",4) == 0) 		//清除主设备配对信息
	{}
	else if(strncasecmp(p_dat,"RADD",4) == 0) 		//作为主设备时成功接过的MAC地址
	{}		
	else if(strncasecmp(p_dat,"VERS",4) == 0) 		//版本信息
	{}
	else if(strncasecmp(p_dat,"TCON",4) == 0)		//主模式下尝试连接时间
	{}
	else if(strncasecmp(p_dat,"RSSI",4) == 0)		//读取信号强度值，为负值
	{}
	else if(strncasecmp(p_dat,"TXPW",4) == 0)		//发射信号强度
	{}
	else if(strncasecmp(p_dat,"TIBE",4) == 0)		//ibeacon基站广播间隔
	{}
	else if(strncasecmp(p_dat,"IMME",4) == 0) 		//查询工作类型
	{}
}
/************************************************************************/
/* 功能：AT命令启动
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
void at_cmd_start(uint8_t *p)
{
	if(p[2] == '\0')		//发送AT，测试命令
	{
		char *p="OK";
		at_cmd_reply((uint8_t*)p,3);
	}
	else if(p[2] == '+')
	{
		at_cmd_switch(&p[3]);	//AT+，第3个开始是有效数据
	}
}

/************************************************************************/
/* 功能：16进制转ASCII,用于生产设备名字
 * 描述：并不是跟着码表转，而是0xAB转成A,B两个字符
 * 形参：16进制，高位字符，低位字符
 * 返回：     */
/************************************************************************/
void hex_to_ascii(uint8_t dat, char *h, char *l)
{
	uint8_t hex_h,hex_l;
	hex_h = (dat>>4)&0x0F;				//四位转一下
	hex_l = dat&0x0F;
	
	if(hex_h<=9)
	{
		*h = hex_h+48;
	}
	else if(hex_h>9)
	{
		*h = hex_h+55;
	}
	
	if(hex_l<=9)
	{
		*l = hex_l+48;
	}
	else if(hex_l>9)
	{
		*l = hex_l+55;
	}
}
