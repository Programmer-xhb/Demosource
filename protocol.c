#include "protocol.h"
#include "usart.h"
#include "check.h"
#include "iwdg.h"




m_protocol_dev_typedef m_ctrl_dev;	//定义modbus控制器
#define FLASH_USER_START_ADDR   ((uint32_t)0x08006000)   /*flash存储起始地址 */
u8 address;
u8 Current_Lx_Add = 1;						//记录上位机读取数据时，当前轮询的FFU
u8 Start_Lx_Add ;									//记录上位机读取数据时，开始轮询的FFU地址
/*
********************************************************************************************************
** 函数名称:	   m_result mb_init    
** 功能:		     初始化modbus        
** 版本:         V1.0  
** 日期：        
** 输入参数:     checkmode：校验模式：0,校验和;1,异或;2,CRC8;3,CRC16
** 输出参数:     0,成功;其他,错误代码
** 说明:         无
********************************************************************************************************
*/
void mb_init()
{
	m_ctrl_dev.rxlen=0;
	m_ctrl_dev.frameok=0;
	m_ctrl_dev.rx_len = 0;
	m_ctrl_dev.S_flag=0;
	if(((u8)FlashRead(FLASH_USER_START_ADDR )) != 0xFF)
	{
		address = (u8)FlashRead(FLASH_USER_START_ADDR );
	}else{
		address = 0xFF;
		FlashWr(address);
		
	}
	GPIO_Config();
  USART1_Config();//串口1参数配置初始化
	USART2_Config();//串口2参数配置初始化
	USART1_RX_DMA_Config();
  USART2_RX_DMA_Config();
}

/*
********************************************************************************************************
** 函数名称:	   Mod_fun3   
** 功能:		     3号功能码处理        
** 版本:         V1.0  
** 日期：        
** 输入参数:     无
** 输出参数:     无
** 说明:         无
********************************************************************************************************
*/
void Mod_fun3()
{
	u16 crc = 0;
	if((  m_ctrl_dev.rxbuf[2] == 0xff) && (m_ctrl_dev.rxbuf[3] == 0xff))
	{
		m_ctrl_dev.send_count = 0;
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = address;   //路由器地址
						
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0x03;
						
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0xFF;
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0XFF;
						
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = address / 256;
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = address % 256;
						
		crc = mc_check_crc16(&m_ctrl_dev.send_buf[0],m_ctrl_dev.send_count); 
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = crc / 256;
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = crc % 256;
						
		USART2_send_data(m_ctrl_dev.send_buf, m_ctrl_dev.send_count);
		m_ctrl_dev.send_count = 0;
	}else if(m_ctrl_dev.rxlen == 4){	
		USART2_send_data(m_ctrl_dev.FFU_Data, FFU_Buffer_SIZE);
	}else{
		m_ctrl_dev.Lx_Flag = 1;																			//停止轮询
		USART1_send_data(m_ctrl_dev.rxbuf, m_ctrl_dev.rxlen);
		m_ctrl_dev.Time_Flag = 1;																		//开始计数
		m_ctrl_dev.Time_Count = 0;                    							// 计数清零
	}
}

/*
********************************************************************************************************
** 函数名称:	   Mod_fun6  
** 功能:		     6号功能码处理        
** 版本:         V1.0  
** 日期：        
** 输入参数:     无
** 输出参数:     无
** 说明:         无
********************************************************************************************************
*/
void Mod_fun6()
{
	u32 Modify_add;														
	m_ctrl_dev.Lx_Flag = 1;																			//停止轮询
	if((  m_ctrl_dev.rxbuf[2] == 0xff) && (m_ctrl_dev.rxbuf[3] == 0xff))
	{
		Modify_add = (m_ctrl_dev.rxbuf[4]<<8) + m_ctrl_dev.rxbuf[5];
		if((Modify_add > 0) && (Modify_add <= 255))
		{
			FlashWr(Modify_add);
			address = Modify_add;
		}
		USART2_send_data(m_ctrl_dev.rxbuf, m_ctrl_dev.rxlen);			
	}
	else
	{	
		USART1_send_data( m_ctrl_dev.rxbuf, m_ctrl_dev.rxlen);	
	}
	m_ctrl_dev.Time_Flag = 1;																		//开始计数
	m_ctrl_dev.Time_Count = 0;                    							// 计数清零
}


/*
********************************************************************************************************
** 函数名称:	   Mod_fun16  
** 功能:		     16号功能码处理        
** 版本:         V1.0  
** 日期：        
** 输入参数:     无
** 输出参数:     无
** 说明:         无
********************************************************************************************************
*/
void Mod_fun16()
{
	m_ctrl_dev.Lx_Flag = 1;																				//停止轮询
	USART1_send_data( m_ctrl_dev.rxbuf, m_ctrl_dev.rxlen);		
	m_ctrl_dev.Time_Flag = 1;																		  //开始计数
	m_ctrl_dev.Time_Count = 0;                    // 计数清零
}

/*
********************************************************************************************************
** 函数名称:	   Lx_Search    
** 功能:		     轮询路由器下的FFU 
** 版本:         V1.0  
** 日期：        
** 输入参数:     无
** 输出参数:     无
** 说明:         无
********************************************************************************************************
*/
void Lx_Search(void)
{
		u16 i,j;
		u16 crc = 0;
		u16 rccrc = 0;
	  u8 sendcount = 0;                                              //发送数据包计数
		Start_Lx_Add = Current_Lx_Add;
		for(i = Start_Lx_Add;i <=FFU_Count;i++)
		{
			m_ctrl_dev.send_count = 0;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = address;      //路由器地址
				
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0x03;
			
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = i;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 01;
				
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 00;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0x03;
				
			crc = mc_check_crc16(&m_ctrl_dev.send_buf[0],m_ctrl_dev.send_count); 
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = crc / 256;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = crc % 256;
			
			Current_Lx_Add = i;																						//记录当前轮询到的FFU地址
			
			while(sendcount < PackLost_Count)														//轮询三次，三次没有接收到回复数据判断为掉线未知
			{
				if(m_ctrl_dev.frameok == 1) return;                       //上行接收到数据，跳出轮询函数
				if(m_ctrl_dev.S_flag == 1) break;													//收到一帧数据跳出轮询
				USART1_send_data(m_ctrl_dev.send_buf, m_ctrl_dev.send_count);
				sendcount++;
				delay_ms(Lx_Time);
			}
			if((m_ctrl_dev.S_flag == 1) && (i == m_ctrl_dev.rx_buf[2]))  //收到当前轮询的FFU的数据返回
			{
				crc=mc_check_crc16(&m_ctrl_dev.rx_buf[0],m_ctrl_dev.rx_len-2);       //计算校验值
				rccrc=m_ctrl_dev.rx_buf[m_ctrl_dev.rx_len-2]*256 + m_ctrl_dev.rx_buf[m_ctrl_dev.rx_len-1];    //收到的校验值
				if(crc == rccrc)
				{
					for(j = 0;j < EveryPack_DataCount;j++)
					{
						m_ctrl_dev.FFU_Data[(i-1) * EveryPack_DataCount + j] = m_ctrl_dev.rx_buf[j];  //将接收的数据赋值给FFU数据缓冲区
					}
				}
				m_ctrl_dev.S_flag = 0;                                     //下行接收flag置0
				m_ctrl_dev.rx_len = 0;																		 //下行接收数据计数清零
				sendcount = 0;																						 //发送数据包计数清零
			}else{																											 //没有收到数据，FFU掉线未知状态
				for(j = 0;j < EveryPack_DataCount;j++)
				{
					m_ctrl_dev.FFU_Data[(i-1) * EveryPack_DataCount + j] = 0x00;  						 //掉线未知FFU数据赋值为0
				}
				sendcount = 0;																						 //发送数据包计数清零
			}
			if(Current_Lx_Add == 30)  Current_Lx_Add = 1;								
			m_ctrl_dev.send_count = 0;
		}
}

/*
********************************************************************************************************
** 函数名称:	   Modbus_Event    
** 功能:		     modbus事件处理函数       
** 版本:         V1.0  
** 日期：        
** 输入参数:     无
** 输出参数:     无
** 说明:         无
********************************************************************************************************
*/
void Modbus_Event()
{
	u16 crc;
	u16 rccrc;
	if((m_ctrl_dev.S_flag == 1) && (m_ctrl_dev.rx_buf[1] != 0x03))
	{
		Shang_Chuang();
	}
	if(m_ctrl_dev.frameok == 1)
	{
		crc=mc_check_crc16(&m_ctrl_dev.rxbuf[0],m_ctrl_dev.rxlen-2);       //计算校验值
		rccrc=m_ctrl_dev.rxbuf[m_ctrl_dev.rxlen-2]*256 + m_ctrl_dev.rxbuf[m_ctrl_dev.rxlen-1];    //收到的校验值
		if(crc == rccrc)
		{
			if(m_ctrl_dev.rxbuf[0] == address)    //判断是否是发给本机的数据
			{
				switch( m_ctrl_dev.rxbuf[1])
				{
					case 0x03:
						Mod_fun3();								//3号功能码处理	
						break;
					case 0x06:
						Mod_fun6();								//6号功能码处理	
						break;
					case 0x16:
						Mod_fun16();							//16号功能码处理	
						break;
				}				
			}
		}
		m_ctrl_dev.rxlen = 0;
		m_ctrl_dev.frameok = 0;
	}
}

