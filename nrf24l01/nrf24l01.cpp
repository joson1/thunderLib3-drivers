#include "nrf24l01.h"


nrf24l01::nrf24l01( char spin, GPIO CSn,GPIO IRQn ,GPIO CEx)
{
    CE = CEx;
    IRQ = IRQn;
    CS = CSn;
    spi1.init(SPI_MODE_4,SPI_DATAWIDTH_8);
    spi1.open(4); //速度等级为2
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
	write_buf(NRF_WRITE_REG+TX_ADDR,buf,5); //写入5个字节的地址.	
    read_buf(TX_ADDR,buf,5);                //读出写入的地址  

	for(i=0;i<5;i++)if((uint8_t)buf[i]!=0XA5)break;	 							   
	if(i!=5)return 0;//检测24L01错误	
	return 1;		 //检测到24L01
}

//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
char nrf24l01::write_reg(char reg,char value)
{
	char status;	
   	CS=0;                 //使能SPI传输
  	status =spi1.transfer(reg);//发送寄存器号 
  	spi1.transfer(value);      //写入寄存器的值
  	CS=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值    

}
//读取SPI寄存器值
//reg:要读的寄存器
char nrf24l01::read_reg(char reg)
{
    char reg_val;	    
 	CS = 0;          //使能SPI传输		
  	spi1.transfer(reg);   //发送寄存器号
  	reg_val=spi1.transfer(0XFF);//读取寄存器内容
  	CS = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}

//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
char nrf24l01::read_buf(char reg,char *pBuf,char len)
{
	char status,char_ctr;	       
  	CS = 0;           //使能SPI传输
  	status=spi1.transfer(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(char_ctr=0;char_ctr<len;char_ctr++)pBuf[char_ctr]=spi1.transfer(0XFF);//读出数据
  	CS=1;       //关闭SPI传输
  	return status;        //返回读到的状态值    
}

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
char nrf24l01::write_buf(char reg, char *pBuf, char len)
{
	char status,char_ctr;	    
 	CS = 0;          //使能SPI传输
  	status = spi1.transfer(reg);//发送寄存器值(位置),并读取状态值
  	for(char_ctr=0; char_ctr<len; char_ctr++)spi1.transfer(*pBuf++); //写入数据	 
  	CS = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}


//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
char nrf24l01::TxPacket(char *txbuf)
{
	char sta;
 	//SPI2_SetSpeed(SPI_SPEED_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	CE=0;
  	write_buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	CE=1;//启动发送	   
	while(IRQ.read()!=0);//等待发送完成
	sta=read_reg(STATUS);  //读取状态寄存器的值	   
	write_reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		write_reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败    
}


//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:1，接收完成；其他，错误代码
char nrf24l01::RxPacket(char *rxbuf)
{
	char sta;		    							   
	
	sta=read_reg(STATUS);  //读取状态寄存器的值    	 
	write_reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		read_buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		write_reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 1; 
	}	   
	return 0;//没收到任何数据    
}


//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void nrf24l01::set_RX_mode(void)
{
	CE=0;	  
  	write_buf(NRF_WRITE_REG+RX_ADDR_P0,(char*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	write_reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	write_reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	write_reg(NRF_WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	write_reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	write_reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	write_reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	CE = 1; //CE为高,进入接收模式     
}


//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void nrf24l01::set_TX_mode()
{
	CE=0;	    
  	write_buf(NRF_WRITE_REG+TX_ADDR,(char*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	write_buf(NRF_WRITE_REG+RX_ADDR_P0,(char*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	write_reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	write_reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	write_reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	write_reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
  	write_reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	write_reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式,开启所有中断
	CE=1;//CE为高,10us后启动发送    
}
