#include "protocol.h"
#include "usart.h"
#include "check.h"
#include "iwdg.h"




m_protocol_dev_typedef m_ctrl_dev;	//����modbus������
#define FLASH_USER_START_ADDR   ((uint32_t)0x08006000)   /*flash�洢��ʼ��ַ */
u8 address;
u8 Current_Lx_Add = 1;						//��¼��λ����ȡ����ʱ����ǰ��ѯ��FFU
u8 Start_Lx_Add ;									//��¼��λ����ȡ����ʱ����ʼ��ѯ��FFU��ַ
/*
********************************************************************************************************
** ��������:	   m_result mb_init    
** ����:		     ��ʼ��modbus        
** �汾:         V1.0  
** ���ڣ�        
** �������:     checkmode��У��ģʽ��0,У���;1,���;2,CRC8;3,CRC16
** �������:     0,�ɹ�;����,�������
** ˵��:         ��
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
  USART1_Config();//����1�������ó�ʼ��
	USART2_Config();//����2�������ó�ʼ��
	USART1_RX_DMA_Config();
  USART2_RX_DMA_Config();
}

/*
********************************************************************************************************
** ��������:	   Mod_fun3   
** ����:		     3�Ź����봦��        
** �汾:         V1.0  
** ���ڣ�        
** �������:     ��
** �������:     ��
** ˵��:         ��
********************************************************************************************************
*/
void Mod_fun3()
{
	u16 crc = 0;
	if((  m_ctrl_dev.rxbuf[2] == 0xff) && (m_ctrl_dev.rxbuf[3] == 0xff))
	{
		m_ctrl_dev.send_count = 0;
		m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = address;   //·������ַ
						
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
		m_ctrl_dev.Lx_Flag = 1;																			//ֹͣ��ѯ
		USART1_send_data(m_ctrl_dev.rxbuf, m_ctrl_dev.rxlen);
		m_ctrl_dev.Time_Flag = 1;																		//��ʼ����
		m_ctrl_dev.Time_Count = 0;                    							// ��������
	}
}

/*
********************************************************************************************************
** ��������:	   Mod_fun6  
** ����:		     6�Ź����봦��        
** �汾:         V1.0  
** ���ڣ�        
** �������:     ��
** �������:     ��
** ˵��:         ��
********************************************************************************************************
*/
void Mod_fun6()
{
	u32 Modify_add;														
	m_ctrl_dev.Lx_Flag = 1;																			//ֹͣ��ѯ
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
	m_ctrl_dev.Time_Flag = 1;																		//��ʼ����
	m_ctrl_dev.Time_Count = 0;                    							// ��������
}


/*
********************************************************************************************************
** ��������:	   Mod_fun16  
** ����:		     16�Ź����봦��        
** �汾:         V1.0  
** ���ڣ�        
** �������:     ��
** �������:     ��
** ˵��:         ��
********************************************************************************************************
*/
void Mod_fun16()
{
	m_ctrl_dev.Lx_Flag = 1;																				//ֹͣ��ѯ
	USART1_send_data( m_ctrl_dev.rxbuf, m_ctrl_dev.rxlen);		
	m_ctrl_dev.Time_Flag = 1;																		  //��ʼ����
	m_ctrl_dev.Time_Count = 0;                    // ��������
}

/*
********************************************************************************************************
** ��������:	   Lx_Search    
** ����:		     ��ѯ·�����µ�FFU 
** �汾:         V1.0  
** ���ڣ�        
** �������:     ��
** �������:     ��
** ˵��:         ��
********************************************************************************************************
*/
void Lx_Search(void)
{
		u16 i,j;
		u16 crc = 0;
		u16 rccrc = 0;
	  u8 sendcount = 0;                                              //�������ݰ�����
		Start_Lx_Add = Current_Lx_Add;
		for(i = Start_Lx_Add;i <=FFU_Count;i++)
		{
			m_ctrl_dev.send_count = 0;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = address;      //·������ַ
				
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0x03;
			
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = i;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 01;
				
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 00;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = 0x03;
				
			crc = mc_check_crc16(&m_ctrl_dev.send_buf[0],m_ctrl_dev.send_count); 
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = crc / 256;
			m_ctrl_dev.send_buf[m_ctrl_dev.send_count++] = crc % 256;
			
			Current_Lx_Add = i;																						//��¼��ǰ��ѯ����FFU��ַ
			
			while(sendcount < PackLost_Count)														//��ѯ���Σ�����û�н��յ��ظ������ж�Ϊ����δ֪
			{
				if(m_ctrl_dev.frameok == 1) return;                       //���н��յ����ݣ�������ѯ����
				if(m_ctrl_dev.S_flag == 1) break;													//�յ�һ֡����������ѯ
				USART1_send_data(m_ctrl_dev.send_buf, m_ctrl_dev.send_count);
				sendcount++;
				delay_ms(Lx_Time);
			}
			if((m_ctrl_dev.S_flag == 1) && (i == m_ctrl_dev.rx_buf[2]))  //�յ���ǰ��ѯ��FFU�����ݷ���
			{
				crc=mc_check_crc16(&m_ctrl_dev.rx_buf[0],m_ctrl_dev.rx_len-2);       //����У��ֵ
				rccrc=m_ctrl_dev.rx_buf[m_ctrl_dev.rx_len-2]*256 + m_ctrl_dev.rx_buf[m_ctrl_dev.rx_len-1];    //�յ���У��ֵ
				if(crc == rccrc)
				{
					for(j = 0;j < EveryPack_DataCount;j++)
					{
						m_ctrl_dev.FFU_Data[(i-1) * EveryPack_DataCount + j] = m_ctrl_dev.rx_buf[j];  //�����յ����ݸ�ֵ��FFU���ݻ�����
					}
				}
				m_ctrl_dev.S_flag = 0;                                     //���н���flag��0
				m_ctrl_dev.rx_len = 0;																		 //���н������ݼ�������
				sendcount = 0;																						 //�������ݰ���������
			}else{																											 //û���յ����ݣ�FFU����δ֪״̬
				for(j = 0;j < EveryPack_DataCount;j++)
				{
					m_ctrl_dev.FFU_Data[(i-1) * EveryPack_DataCount + j] = 0x00;  						 //����δ֪FFU���ݸ�ֵΪ0
				}
				sendcount = 0;																						 //�������ݰ���������
			}
			if(Current_Lx_Add == 30)  Current_Lx_Add = 1;								
			m_ctrl_dev.send_count = 0;
		}
}

/*
********************************************************************************************************
** ��������:	   Modbus_Event    
** ����:		     modbus�¼�������       
** �汾:         V1.0  
** ���ڣ�        
** �������:     ��
** �������:     ��
** ˵��:         ��
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
		crc=mc_check_crc16(&m_ctrl_dev.rxbuf[0],m_ctrl_dev.rxlen-2);       //����У��ֵ
		rccrc=m_ctrl_dev.rxbuf[m_ctrl_dev.rxlen-2]*256 + m_ctrl_dev.rxbuf[m_ctrl_dev.rxlen-1];    //�յ���У��ֵ
		if(crc == rccrc)
		{
			if(m_ctrl_dev.rxbuf[0] == address)    //�ж��Ƿ��Ƿ�������������
			{
				switch( m_ctrl_dev.rxbuf[1])
				{
					case 0x03:
						Mod_fun3();								//3�Ź����봦��	
						break;
					case 0x06:
						Mod_fun6();								//6�Ź����봦��	
						break;
					case 0x16:
						Mod_fun16();							//16�Ź����봦��	
						break;
				}				
			}
		}
		m_ctrl_dev.rxlen = 0;
		m_ctrl_dev.frameok = 0;
	}
}

