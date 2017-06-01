//AT 命令处理
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_uart.h"
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
/* 功能：回复数据格式
 * 描述：前3个字符必是OK+，其后未定
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_cmd_reply_header(uint8_t *p, uint8_t len)
{
	p[0] = 'O';
	p[1] = 'K';
	p[2] = '+';
	at_cmd_reply(p,len);
}

/************************************************************************/
/* 功能：波特率设置
 * 描述：只支持有限的波特率
 * 形参：AT指令(去除AT+)
 * 返回：     */
/************************************************************************/
static void at_baud(uint8_t *p)
{
	uint8_t reply[10],len;
	if(p[0] == '?')
	{
		int baud = 0;
		switch(NRF_UART0->BAUDRATE)
		{
			case UART_BAUDRATE_BAUDRATE_Baud9600: baud = 0;break;
			case UART_BAUDRATE_BAUDRATE_Baud19200:baud = 1;break;
			case UART_BAUDRATE_BAUDRATE_Baud38400:baud = 2;break;
			case UART_BAUDRATE_BAUDRATE_Baud57600:baud = 3;break;
			case UART_BAUDRATE_BAUDRATE_Baud115200:baud= 4;break;
			default:break;
		}
		reply[3] = 'G';
		reply[7] = baud+48;		//数字转ASCII
	}
	else
	{
		//下发的是ascii
		switch(p[0])
		{
			case '0':nrf_uart_baudrate_set(NRF_UART0,NRF_UART_BAUDRATE_9600);break;
			case '1':nrf_uart_baudrate_set(NRF_UART0,NRF_UART_BAUDRATE_19200);break;
			case '2':nrf_uart_baudrate_set(NRF_UART0,NRF_UART_BAUDRATE_38400);break;
			case '3':nrf_uart_baudrate_set(NRF_UART0,NRF_UART_BAUDRATE_57600);break;
			case '4':nrf_uart_baudrate_set(NRF_UART0,NRF_UART_BAUDRATE_115200);break;
			default:break;
		}
		//因修改了波特率，回复的数据可能是乱码。
		reply[3] = 'S';
		reply[7] = p[0];		//数字转ASCII
	}
	
	reply[4] = 'e';
	reply[5] = 't';
	reply[6] = ':';
	reply[8] = '\0';
	len = 9;
	at_cmd_reply_header(reply,len);
}
/************************************************************************/
/* 功能：校验位
 * 描述：本芯片只支持偶校验
 * BUG：当串口PC助手的校验位和芯片的校验位不一样时，PC串口发送一次数据后，再改成一样的设置
 * 芯片依旧无法收到数据
 * 例如，芯片无校验，PC偶校验。PC发送一次数据后再改成无校验，芯片已收不到数据。
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_pari(uint8_t *p)
{
	uint8_t reply[10],len;
	if(p[0] == '?')
	{
		switch((NRF_UART0->CONFIG)&0x0E)
		{
			case 0x0E:reply[7] = '2';break;
			default:reply[7] = '0';break;
		}
		reply[3] = 'G';
	}
	else
	{
		switch(p[0])
		{
			case '0':nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_EXCLUDED,NRF_UART_HWFC_DISABLED);break;
			case '2':nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_INCLUDED,NRF_UART_HWFC_DISABLED);break;
		}
		reply[3] = 'S';
		reply[7] = p[0];		//数字转ASCII
	}
	reply[4] = 'e';
	reply[5] = 't';
	reply[6] = ':';
	reply[8] = '\0';
	len = 9;
	at_cmd_reply_header(reply,len);
}
/************************************************************************/
/* 功能：停止位
 * 描述：芯片只支持1停止位
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_stop(uint8_t *p)
{
	uint8_t reply[10],len;
	if(p[0] == '?')
	{
		reply[3] = 'G';
		reply[7] = '0';
		
		reply[4] = 'e';
		reply[5] = 't';
		reply[6] = ':';
		reply[8] = '\0';
		len = 9;
		at_cmd_reply_header(reply,len);
	}
}
/************************************************************************/
/* 功能：开启关闭串口
 * 描述：关闭串口后，必须重新烧写程序才能设置。因此该功能不开放
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_mode(uint8_t *p)
{}
/************************************************************************/
/* 功能：设备名称
 * 描述：只能修改前缀，后缀是根据MAC地址生成的，不能更改
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_name(uint8_t *p)
{
}
/************************************************************************/
/* 功能：回复默认设置
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_renew(uint8_t *p)
{
	uint8_t reply[10],len;
	if(p[0] == '?')
	{
		reply[3] = 'R';
		reply[4] = 'E';
		reply[5] = 'N';
		reply[6] = 'E';
		reply[7] = 'W';
		reply[8] = '\0';
		len = 9;
		at_cmd_reply_header(reply,len);
	
		nrf_uart_baudrate_set(NRF_UART0,NRF_UART_BAUDRATE_9600);
		nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_EXCLUDED,NRF_UART_HWFC_DISABLED);		
	}	
}
/************************************************************************/
/* 功能：复位重启
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_reset(uint8_t *p)
{}
/************************************************************************/
/* 功能：主从模式
 * 描述：目前只有从模式
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_pole(uint8_t *p)
{}
/************************************************************************/
/* 功能：MAC地址
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_addr(uint8_t *p)
{
	if(p[0] != '?')		return;
	
	ble_gap_addr_t p_addr;
	char h,l;
	uint8_t reply[21],len,err_code;
	
	reply[3] = 'L';
	reply[4] = 'A';
	reply[5] = 'D';
	reply[6] = 'D';
	reply[7] = ':';
	
	err_code = sd_ble_gap_address_get(&p_addr);
	APP_ERROR_CHECK(err_code);
	for(uint8_t i=0; i<6; i++)
	{
		hex_to_ascii(*(p_addr.addr+i), &h,  &l);
	
		reply[8+2*i] = h;
		reply[9+2*i] = l;		
	}
	reply[20] = '\0';
	len = 21;
	at_cmd_reply_header(reply,len);
}
/************************************************************************/
/* 功能：版本信息
 * 描述：程序版本
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_vers(uint8_t *p)
{}
/************************************************************************/
/* 功能：读取信号强度
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_rssi(uint8_t *p)
{}
/************************************************************************/
/* 功能：发射信号强度
 * 描述：
 * 形参：
 * 返回：     */
/************************************************************************/
static void at_txpw(uint8_t *p)
{}
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
		at_baud(&p[4]);
	}
	else if(strncasecmp(p_dat,"PARI",4) == 0)		//校验位
	{
		at_pari(&p[4]);
	}
	else if(strncasecmp(p_dat,"STOP",4) == 0)		//停止位
	{
		at_stop(&p[4]);
	}
	else if(strncasecmp(p_dat,"MODE",4) == 0)		//开闭串口透传
	{
		at_mode(&p[4]);
	}
	else if(strncasecmp(p_dat,"NAME",4) == 0)		//设备名称
	{
		at_name(&p[4]);
	}
	else if(strncasecmp(p_dat,"RENEW",5) == 0)		//恢复默认设置
	{
		at_renew(&p[5]);
	}
	else if(strncasecmp(p_dat,"RESET",5) == 0) 		//复位重启
	{
		at_reset(&p[5]);
	}
	else if(strncasecmp(p_dat,"POLE",4) == 0) 		//主从模式
	{
		at_pole(&p[4]);
	}
	else if(strncasecmp(p_dat,"ADDR",4) == 0) 		//MAC地址
	{
		at_addr(&p[4]);
	}
	else if(strncasecmp(p_dat,"VERS",4) == 0) 		//版本信息
	{
		at_vers(&p[4]);
	}
	else if(strncasecmp(p_dat,"RSSI",4) == 0)		//读取信号强度值，为负值
	{
		at_rssi(&p[4]);
	}
	else if(strncasecmp(p_dat,"TXPW",4) == 0)		//发射信号强度
	{
		at_txpw(&p[4]);
	}
	
#if 0
	else if(strncasecmp(p_dat,"PASS",4) == 0) 		//配对密码
	{}
	else if(strncasecmp(p_dat,"TYPE",4) == 0)		//鉴权类型
	{}
	else if(strncasecmp(p_dat,"CONNL",4) == 0) 		//查询最后一次连接成功的从设备
	{}
	else if(strncasecmp(p_dat,"CON",3) == 0) 		//连接指定的从设备	
	{}
	else if(strncasecmp(p_dat,"CLEAR",4) == 0) 		//清除主设备配对信息
	{}
	else if(strncasecmp(p_dat,"RADD",4) == 0) 		//作为主设备时成功接过的MAC地址
	{}		
	else if(strncasecmp(p_dat,"TCON",4) == 0)		//主模式下尝试连接时间
	{}
	else if(strncasecmp(p_dat,"TIBE",4) == 0)		//ibeacon基站广播间隔
	{}
	else if(strncasecmp(p_dat,"IMME",4) == 0) 		//查询工作类型
	{}
#endif
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
