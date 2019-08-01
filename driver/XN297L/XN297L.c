//============================================================================//
//  * @file            RF.c
//  * @author         Shi Zheng 
//  * @version        V1.0
//  * @date           24/4/2015
//  * @brief          RFPN006 communication interface
//  * @modify user:   Shizheng
//  * @modify date:   24/4/2015
//============================================================================//
#include "XN297L.H"

#ifndef XN297L_SPI
    #define XN297L_SPI SPI2
#endif

#ifdef XN297L_SPI
SPIBaseDefine *g_poSpiXn297lHandle = NULL;
#endif


const uint8_t TX_ADDRESS_DEF[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};    		//RF 地址：接收端和发送端需一致


//void SPI_software_init(void)
//{
//    GPIO_Init( GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT);                       //IRQ  input pulling high without interrupt
//    GPIO_Init( GPIOB, GPIO_Pin_1, GPIO_Mode_Out_PP_Low_Fast);                   //CE   output Low pulling push    

//    GPIO_Init( GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP_High_Fast);                  //CSN  output High pulling push
//    GPIO_Init( GPIOB, GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Fast);                   //SCK  output Low  pulling push 
//    GPIO_Init( GPIOB, GPIO_Pin_6, GPIO_Mode_Out_PP_High_Fast);                  //MOSI output High pulling push
//    GPIO_Init( GPIOB, GPIO_Pin_7, GPIO_Mode_In_PU_No_IT);                       //MISO input pull high
//}

uint8_t spi2_rw_byte( uint8_t wdata)
{
    uint8_t abData;
    if(SpiWriteReadByte(g_poSpiXn297lHandle,wdata,&abData) < 0)
        return 0;
    else
        return abData;
}

void delay_nop_us(uint32_t n)
{
    uint8_t i;
    while(n)
    {
        i = 20;
        while(i--);
        n--;
    }
}

/******************************************************************************/
//            RF_WriteReg
//                Write Data(1 Byte Address ,1 byte data)
/******************************************************************************/
void RF_WriteReg( uint8_t reg,  uint8_t wdata)
{
    CSN_LOW;
    SPI_RW(reg);
    SPI_RW(wdata);
    CSN_HIGH;
}


/******************************************************************************/
//            RF_ReadReg
//                Read Data(1 Byte Address ,1 byte data return)
/******************************************************************************/
 uint8_t	ucRF_ReadReg( uint8_t reg)
{
     uint8_t tmp;
    
    CSN_LOW;
    SPI_RW(reg);
    tmp = SPI_RW(0);
    CSN_HIGH;
    
    return tmp;
}

/******************************************************************************/
//            RF_WriteBuf
//                Write Buffer
/******************************************************************************/
void RF_WriteBuf( uint8_t reg, uint8_t *pBuf, uint8_t length)
{
     uint8_t j;
    CSN_LOW;    
    SPI_RW(reg);
    for(j = 0;j < length; j++)
    {
        SPI_RW(pBuf[j]);
    }    
    CSN_HIGH;
}

/******************************************************************************/
//            RF_ReadBuf
//                Read Data(1 Byte Address ,length byte data read)
/******************************************************************************/
void RF_ReadBuf( uint8_t reg, unsigned char *pBuf,  uint8_t length)
{
    uint8_t byte_ctr;

    CSN_LOW;                    		                               			
    SPI_RW(reg);       		                                                		
    for(byte_ctr=0;byte_ctr<length;byte_ctr++)
    	pBuf[byte_ctr] = SPI_RW(0);                                                 		
    CSN_HIGH;                                                                   		
}



/******************************************************************************/
//            RF_TxMode
//                Set RF into TX mode
/******************************************************************************/
void RF_TxMode(void)
{
    CE_LOW;
    RF_WriteReg(W_REGISTER + CONFIG,  0X8E);							// 将RF设置成TX模式    
    delay_ms(10);    
    CE_HIGH;
    delay_ms(10);
}


/******************************************************************************/
//            RF_RxMode
//            将RF设置成RX模式，准备接收数据
/******************************************************************************/
void RF_RxMode(void)
{
    CE_LOW;
    RF_WriteReg(W_REGISTER + CONFIG,  0X8F );							// 将RF设置成RX模式  
    delay_ms(10);
    CE_HIGH;											// Set CE pin high 开始接收数据
    delay_ms(10);
}

void restart_rx(void)
{
    CE_LOW;
    delay_ms(1);
    CE_HIGH;
}

/******************************************************************************/
//            RF_GetStatus
//            read RF IRQ status,3bits return
/******************************************************************************/
uint8_t ucRF_GetStatus(void)
{
    return ucRF_ReadReg(STATUS)&0x70;								//读取RF的状态 
}

/******************************************************************************/
//            RF_ClearStatus
//                clear RF IRQ
/******************************************************************************/
void RF_ClearStatus(void)
{
    RF_WriteReg(W_REGISTER + STATUS,0x70);							//清除RF的IRQ标志 
}

/******************************************************************************/
//            RF_ClearFIFO
//                clear RF TX/RX FIFO
/******************************************************************************/
void RF_ClearFIFO(void)
{
    RF_WriteReg(FLUSH_TX, 0);			                                		//清除RF 的 TX FIFO		
    RF_WriteReg(FLUSH_RX, 0);                                                   		//清除RF 的 RX FIFO	
}

/******************************************************************************/
//            RF_SetChannel
//                Set RF TX/RX channel:Channel
/******************************************************************************/
void RF_SetChannel( uint8_t Channel)
{    
    //CE_LOW;
    RF_WriteReg(W_REGISTER + RF_CH, Channel);
}

void rf_write_address(uint8_t *pbuf)
{
    uint8_t i;
    uint8_t data[5] = {0};
    uint8_t exit = 0;
    
    CE_LOW;
    while (!exit)
    {
        RF_WriteBuf(W_REGISTER + TX_ADDR,   pbuf, sizeof(TX_ADDRESS_DEF));	// Writes TX_Address to PN006
        delay_nop_us(10);
        RF_WriteBuf(W_REGISTER + RX_ADDR_P0,pbuf, sizeof(TX_ADDRESS_DEF));	// RX_Addr0 same as TX_Adr for Auto.Ack   
        delay_nop_us(10);
        
        RF_ReadBuf(R_REGISTER + RX_ADDR_P0,   data, sizeof(TX_ADDRESS_DEF));
        for (i=0;i<5;i++)
        {
            if(data[i] == pbuf[i])
            {                
                exit = 1;
            }
            else
            {               
                exit = 0;
                break;
            }
        }   
    }
}

void rf_set_power(uint8_t rf_power)
{    
     RF_WriteReg(W_REGISTER + RF_SETUP,  rf_power);
}

/******************************************************************************/
//            发送数据：
//            参数：
//              1. ucPayload：需要发送的数据首地址
//              2. length:  需要发送的数据长度
//              Return:
//              1. MAX_RT: TX Failure  (Enhance mode)
//              2. TX_DS:  TX Successful (Enhance mode)
//              note: Only use in Tx Mode
//              length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/
uint8_t ucRF_TxData( uint8_t *ucPayload,  uint8_t length)
{
    uint8_t   Status_Temp;
    
    RF_WriteBuf(W_TX_PAYLOAD, ucPayload, length);                               		//write data to txfifo        
   /* CE_HIGH;                                                                    		//rf entery tx mode start send data 
    delay_10us(200);                                                              		//keep ce high at least 16us
    CE_LOW;                                                                     		//rf entery stb3
   */
    delay_ms(2);
 
   
    while(IRQ_STATUS);                                                          		//waite irq pin low 
    Status_Temp = ucRF_ReadReg(STATUS) & 0x70;                                                  //读取发送完成后的status
 
    RF_WriteReg(W_REGISTER + STATUS, Status_Temp);                                 		//清除Status
    RF_WriteReg(FLUSH_TX,0);                                                   			//清 FIFO
    
    return Status_Temp;
}

void RF_Tx_TransmintData( uint8_t *ucTXPayload,  uint8_t length)
{    
//    if(!ucRF_GetStatus())                                                                               // rf 处于空闲状态才发送数据
//    {       
            //RF_WriteBuf(W_TX_PAYLOAD, ucTXPayload, length);                               		//write data to txfifo      
            RF_WriteBuf(W_ACK_PAYLOAD, ucTXPayload, length);                               		//write data to txfifo              
            CE_HIGH;                                                                    		//rf entery tx mode start send data 
//    }
}

/******************************************************************************/
//            ucRF_DumpRxData
//            读出接收到的数据：
//            参数：
//              1. ucPayload：存储读取到的数据的Buffer
//              2. length:    读取的数据长度
//              Return:
//              1. 0: 没有接收到数据
//              2. 1: 读取接收到的数据成功
//              note: Only use in Rx Mode
//              length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/
uint8_t ucRF_DumpRxData( uint8_t *ucPayload,  uint8_t length)
{
    if(IRQ_STATUS)
    {
      return 0;                                                                 		//若IRQ PIN为高，则没有接收到数据
    }
    CE_LOW;
    RF_ReadBuf(R_RX_PAYLOAD, ucPayload, length);                                		//将接收到的数据读出到ucPayload，且清除rxfifo
    RF_WriteReg(FLUSH_RX, 0);	
    RF_WriteReg(W_REGISTER + STATUS, 0x70);                                     		//清除Status
    CE_HIGH;                                                                    		//继续开始接收
    
    return 1;
}

void rf_io_init(void)
{  
    //CE 控制脚初始化
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13); 
    
    //2.4G POWER EN
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_bit_set(GPIOC, GPIO_PIN_2);
}


////////////////////////////////////////////////////////////////////////////////

//          以下部分与RF通信相关，不建议修改
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
//            PN006_Initial
//                Initial RF
/******************************************************************************/
void RF_Init(void)
{
    uint8_t feature = 0x00;
#if(DATA_RATE == DR_1M)   
    uint8_t  BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46};                               //1M速率配置
//    uint8_t  BB_cal_data[]    = {0x2A,0xEC,0x6F,0x9C,0x46};                               //1M速率配置
    uint8_t  RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t  RF_cal2_data[]   = {0x45,0x21,0xef,0x2C,0x5A,0x50};
    uint8_t  Dem_cal_data[]   = {0x01};  
    uint8_t  Dem_cal2_data[]  = {0x0b,0xDF,0x02};  
#elif(DATA_RATE == DR_250K)
   uint8_t   BB_cal_data[]    = { 0x12,0xec,0x6f,0xa1,0x46};                          //250K速率配置
   uint8_t    RF_cal_data[]    = {0xF6,0x37,0x5C};
   uint8_t   RF_cal2_data[]   = {0xd5,0x21,0xeb,0x2c,0x5a,0x40};
   uint8_t    Dem_cal_data[]   = {0x1F};  
   uint8_t    Dem_cal2_data[]  = {0x0b,0xdf,0x02};
#endif
    
    
//    SPI_init();
    rf_io_init();
    g_poSpiXn297lHandle = SpiOpen(G24_SPI_IDX);
    CE_LOW;
                    
    RF_WriteReg(RST_FSPI, 0x5A);								//Software Reset    			
    RF_WriteReg(RST_FSPI, 0XA5);
    
    RF_WriteReg(FLUSH_TX, 0);									// CLEAR TXFIFO		    			 
    RF_WriteReg(FLUSH_RX, 0);									// CLEAR  RXFIFO
    RF_WriteReg(W_REGISTER + STATUS, 0x70);							// CLEAR  STATUS	
    RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);							// Enable Pipe0
    RF_WriteReg(W_REGISTER + SETUP_AW,  0x03);							// address witdth is 5 bytes
    RF_WriteReg(W_REGISTER + RF_CH,    DEFAULT_CHANNEL);                                        // 2478M HZ
    RF_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);						// 8 bytes
    RF_WriteBuf(W_REGISTER + TX_ADDR,   ( uint8_t*)TX_ADDRESS_DEF, sizeof(TX_ADDRESS_DEF));	// Writes TX_Address to PN006
    RF_WriteBuf(W_REGISTER + RX_ADDR_P0,( uint8_t*)TX_ADDRESS_DEF, sizeof(TX_ADDRESS_DEF));	// RX_Addr0 same as TX_Adr for Auto.Ack   
    RF_WriteBuf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
    RF_WriteReg(W_REGISTER + DYNPD, 0x00);					
    //RF_WriteReg(W_REGISTER + FEATURE, 0x00);
    RF_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);						//DBM  		
    RF_WriteReg(ACTIVATE, 0x73);
    
#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)      
    RF_WriteReg(W_REGISTER + SETUP_RETR,0x01);							//  3 retrans... 	
    RF_WriteReg(W_REGISTER + EN_AA,     0x01);							// Enable Auto.Ack:Pipe0  	
#elif(TRANSMIT_TYPE == TRANS_BURST_MODE)                                                                
    RF_WriteReg(W_REGISTER + SETUP_RETR,0x00);							// Disable retrans... 	
    RF_WriteReg(W_REGISTER + EN_AA,     0x00);							// Disable AutoAck 
#endif

    if(PAYLOAD_WIDTH <33)											
    {
        feature =  0x40;							//切换到32byte模式       IRQ 用作PA控制 bit6 = 1
    }
    else
    {
        feature =  0x58;							//切换到64byte模式	   
    }
    
#if(EN_DYNPLOAD == 1)
    feature |= 0x04;
    RF_WriteReg(W_REGISTER + DYNPD, 0x01);
#endif

#if(EN_ACK_PAYLOAD == 1)
    feature |= 0x02;
#endif  
    
    RF_WriteReg(W_REGISTER + FEATURE, feature);


    CE_LOW;

}




/******************************************************************************/
//            		进入载波模式
/******************************************************************************/
void RF_Carrier( uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46}; 
    uint8_t RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t RF_cal2_data[]   = {0x45,0x21,0xEF,0xAC,0x5A,0x50};
    uint8_t Dem_cal_data[]   = {0xE1}; 								
    uint8_t Dem_cal2_data[]  = {0x0B,0xDF,0x02};      
    
    RF_WriteReg(RST_FSPI, 0x5A);								//Software Reset    			
    RF_WriteReg(RST_FSPI, 0XA5);
    
    CE_LOW;
    delay_ms(200);
    RF_WriteReg(W_REGISTER + RF_CH, ucChannel_Set);						//单载波频点	   
    RF_WriteReg(W_REGISTER + RF_SETUP, RF_POWER);      						//dbm
    RF_WriteBuf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
    delay_ms(5);	
}

/***************************************end of file ************************************/
