#include "nrf24l01.h"


nrf24l01::nrf24l01( char spin, GPIO CSn,GPIO IRQn ,GPIO CEx)
{
    CE = CEx;
    IRQ = IRQn;
    CS = CSn;
    spi1.init(SPI_MODE_4,SPI_DATAWIDTH_8);
    spi1.open(4); //�ٶȵȼ�Ϊ2
    CSn.set(1);
    CE = 0;

}

nrf24l01::~nrf24l01()
{
}

char nrf24l01::check(void)
{
	char buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	char i;
	write_buf(NRF_WRITE_REG+TX_ADDR,buf,5); //д��5���ֽڵĵ�ַ.	
    read_buf(TX_ADDR,buf,5);                //����д��ĵ�ַ  

	for(i=0;i<5;i++)if((uint8_t)buf[i]!=0XA5)break;	 							   
	if(i!=5)return 0;//���24L01����	
	return 1;		 //��⵽24L01
}

//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
char nrf24l01::write_reg(char reg,char value)
{
	char status;	
   	CS=0;                 //ʹ��SPI����
  	status =spi1.transfer(reg);//���ͼĴ����� 
  	spi1.transfer(value);      //д��Ĵ�����ֵ
  	CS=1;                 //��ֹSPI����	   
  	return(status);       			//����״ֵ̬    

}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
char nrf24l01::read_reg(char reg)
{
    char reg_val;	    
 	CS = 0;          //ʹ��SPI����		
  	spi1.transfer(reg);   //���ͼĴ�����
  	reg_val=spi1.transfer(0XFF);//��ȡ�Ĵ�������
  	CS = 1;          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}

//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
char nrf24l01::read_buf(char reg,char *pBuf,char len)
{
	char status,char_ctr;	       
  	CS = 0;           //ʹ��SPI����
  	status=spi1.transfer(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(char_ctr=0;char_ctr<len;char_ctr++)pBuf[char_ctr]=spi1.transfer(0XFF);//��������
  	CS=1;       //�ر�SPI����
  	return status;        //���ض�����״ֵ̬    
}

//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
char nrf24l01::write_buf(char reg, char *pBuf, char len)
{
	char status,char_ctr;	    
 	CS = 0;          //ʹ��SPI����
  	status = spi1.transfer(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(char_ctr=0; char_ctr<len; char_ctr++)spi1.transfer(*pBuf++); //д������	 
  	CS = 1;       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}


//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
char nrf24l01::TxPacket(char *txbuf)
{
	char sta;
 	//SPI2_SetSpeed(SPI_SPEED_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	CE=0;
  	write_buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	CE=1;//��������	   
	while(IRQ.read()!=0);//�ȴ��������
	sta=read_reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	write_reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		write_reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��    
}


//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:1��������ɣ��������������
char nrf24l01::RxPacket(char *rxbuf)
{
	char sta;		    							   
	
	sta=read_reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	write_reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		read_buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		write_reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 1; 
	}	   
	return 0;//û�յ��κ�����    
}


//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void nrf24l01::set_RX_mode(void)
{
	CE=0;	  
  	write_buf(NRF_WRITE_REG+RX_ADDR_P0,(char*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	write_reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	write_reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	write_reg(NRF_WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		  
  	write_reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	write_reg(NRF_WRITE_REG+RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  	write_reg(NRF_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	CE = 1; //CEΪ��,�������ģʽ     
}


//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void nrf24l01::set_TX_mode()
{
	CE=0;	    
  	write_buf(NRF_WRITE_REG+TX_ADDR,(char*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	write_buf(NRF_WRITE_REG+RX_ADDR_P0,(char*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	write_reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	write_reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	write_reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	write_reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  	write_reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	write_reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	CE=1;//CEΪ��,10us����������    
}
