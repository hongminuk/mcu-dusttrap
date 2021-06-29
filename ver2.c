#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ff.h"
#include "diskio.h"
#include "define.h"
#define CPU_CLOCK_HZ 8000000UL
#define F_CPU 8000000UL

typedef struct sd_dat                  
{
	char *pressure;
	char *note;
	char *tim;
	char *status;
}dat_struct;

typedef enum
{
	POWER_ON,			//[0] POWER ON
	READ_PRESSURE,		//[1] READ SENSOR VALUE
	VALVE_OPEN,
	VALVE_CLOSE,
	ON_TIME_RESET,
	OFF_TIME_RESET,
	VALVE_CH_RESET,
	SLAVE_CH_RESET,
	START_POINT_RESET,
	STOP_POINT_RESET,
	MSD_DETECTION,
	MSD_NON_DETECTION,
	NEW_FILE_OPEN

}SYS_STATE;


SYS_STATE ST;

FRESULT fr;
FILINFO Finfo;
FIL fo;						/* File object */
FATFS fs;					/* File system object */
char arr[] = "TIME\tSTATUS\tNOTE\tPRESSURE\r\n";
char buf[128] = "";


/////////////////////////////////////////////////////////////////////////

enum state_machine{
    SYSTEM_INIT,
	MASTER_VALVE_SET,
	SLAVE_WAITING,
    SENSOR_CHECKING,
    VALVE_ON,
    ON_TIME_SET,
    OFF_TIME_SET,
    VALVE_CH_SET,
    START_POINT_SET,
    STOP_POINT_SET
}STATE;

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////     

void IO_INIT(void);
void INT_INIT(void);
void ADC_INIT(void);
void TIM_INIT(void);
void SYS_INIT(void);
void TIM_INIT(void);
void TIM_SDWR_INIT(void);
unsigned char MSD_INIT(void);
unsigned char EXT_INIT(unsigned char MODE_PARAMETER);
unsigned char READ_ADDR(void);
void TIM_SDWR_INIT();

/////////////////////////////////////////////////////////////////////////

void SD_BUF_PRT(void);
void VALVE_OP(unsigned char TIM_PUL_MODE_FLAG);
void SENSOR_CHECK(unsigned char fnd_on);
int D2AQ(int adc);	//Digiter -> AQUA?
unsigned char ON_TIME_SET_OP(void);
unsigned char OFF_TIME_SET_OP(void);
void VALVE_CH_SET_OP(void);
void SLA_VALVE_CH_SET_OP(void);
unsigned char START_POINT_SET_OP(void);
unsigned char STOP_POINT_SET_OP(void);
unsigned char SDWR_OP(void);
unsigned char FND_PARSING(int num);
void FND_OP(int num);
void WD_ON(unsigned char tim_parameter);
char WD_CHECK(void);

void TWI_MW(unsigned char addr, unsigned char dat);
void TWI_SW(unsigned char addr, unsigned char dat);
unsigned char TWI_MR(unsigned char addr);
unsigned char TWI_SR(unsigned char addr);
void TWI_ERROR(void);

unsigned char DAY_MONTH(void);
void CHECK_TIM(void);
void SAVE_TIM(void);
void EEPROM_WR(unsigned int ADDR, unsigned char SIZE_OF_DAT, unsigned int DAT);
unsigned int EEPROM_RD(unsigned int ADDR, unsigned char SIZE_OF_DAT);
void TWI_INIT(void);
unsigned char twi_master_read(unsigned char addr);
void twi_master_write(unsigned char addr, unsigned char dat);
unsigned char twi_slave_read(unsigned char addr);
void twi_slave_write(unsigned char addr, unsigned char dat);

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
volatile dat_struct dat_buf;
volatile unsigned int SLA_VAL_CH[17] = {3,};
volatile unsigned int TOTAL_VALVE = 0;
volatile int ADC_AVG = 0;
volatile long ADC_SUM = 0;
volatile int MMAQ = 0;
volatile unsigned char SLA_STATE = 0;
volatile unsigned char VALVE_CH = DEFAULT_VALVE_CH;
volatile unsigned int ON_TIME = DEFAULT_ON_TIME;
volatile unsigned int OFF_TIME = DEFAULT_OFF_TIME;
volatile unsigned char valve_index = 0;
volatile unsigned char I2C_ADDR = 0;
volatile unsigned char I2C_SL_NUM = 0;
volatile unsigned char I2C_DAT_BUF = 0;
volatile unsigned char I2C_FLAG = 0;
volatile unsigned char ADC_FLAG = 0;
volatile unsigned char FND_DOT0 = 0;
volatile unsigned char FND_DOT1 = 0;
volatile unsigned char FND_DOT2 = 0;

volatile unsigned char SET_SW_FLAG = 0;
volatile unsigned char UP_SW_FLAG = 0;
volatile unsigned char OK_SW_FLAG = 0;
volatile unsigned char DOWN_SW_FLAG = 0;

volatile unsigned char TIM_SDWR_FLAG = 0;
volatile unsigned char TIM_PULSING_FLAG = 0;
volatile unsigned char MODE_TOGGLE_SW_FLAG = 0;
volatile unsigned char SD_DETECTION_FLAG = 0;
volatile unsigned char NEW_FILE_FLAG = 0;

volatile unsigned char FND_D0 = FND_0;
volatile unsigned char FND_D1 = FND_0;
volatile unsigned char FND_D2 = FND_0;
volatile unsigned char FND_D3 = FND_0;
volatile unsigned char FND_BLINK_FLAG = 0;

volatile unsigned char SEC_CNT = 0;         //125 count = 1sec
volatile unsigned char WD_SEC = 0;
volatile unsigned char WD_FLAG = 0;
volatile unsigned char START_OP_POINT = DEFAULT_START_OP_POINT;
volatile unsigned char STOP_OP_POINT = DEFAULT_STOP_OP_POINT; 
volatile unsigned char FIL_NUM = 1;
volatile unsigned char MIN = 0;
volatile unsigned char HOUR = 0;
volatile unsigned char DAY = 14;
volatile unsigned char MONTH = 1;
volatile unsigned int YEAR = 2015;
volatile unsigned int SLAVE_CHECK_BUF;
volatile unsigned int SLAVE_DEV;
volatile unsigned int SLAVE_ADDR_CHECK;
volatile unsigned int SLA_CH_SUM = 3;
volatile unsigned int TWI_DATA_BUF;
volatile unsigned int valve_on_flag = 0;
volatile unsigned int SLAVE_TIM_PUL_FLAG = 0;
volatile unsigned int SD_WRITTING_FLAG = 0;


volatile unsigned int valve_count = 0;




/////////////////////////////////////////////////////////////////////////   
ISR(INT4_vect)
{
	SET_SW_FLAG = 1;
	//TIM_PUL_MODE_FLAG = 1;
}
ISR(INT5_vect)
{
	UP_SW_FLAG = 1;
}
ISR(INT6_vect)
{
	OK_SW_FLAG = 1;
}
ISR(INT7_vect)
{
	DOWN_SW_FLAG = 1;
}


ISR(TIMER0_COMP_vect)			//for Watch Dog
{
    if((++SEC_CNT)==125)
    {
        SEC_CNT = 0;
        WD_SEC++;				//1sec
    }
    if(WD_SEC == WD_TIM_SET)
    {
        WD_SEC = 0;
        WD_FLAG = 1;
    }    
}



ISR(TIMER1_COMPA_vect)
{
    static unsigned int TIM_CNT = 0;
    
    TIM_CNT++;
    if(TIM_CNT == 1)
    {
        TIM_CNT = 0;
		TIM_SDWR_FLAG = 1;
		MIN++;
	
		if(MIN == 60)
		{
			MIN = 0;
			HOUR++;
			if(SD_DETECTION_FLAG == 1)
			{
				//SD_DETECTION_FLAG=0;
				NEW_FILE_FLAG = 1;
					
						MSD_LED_OFF;
						f_close(&fo);
						f_mount(0,"",0);
						if((SD_DETECTION_FLAG=MSD_INIT()))
						{
							ST = NEW_FILE_OPEN;
							NEW_FILE_FLAG = 0;
							if(!SDWR_OP())
							{
								SD_DETECTION_FLAG = 0;
								f_close(&fo);
								f_mount(0,"",0);
								MSD_LED_OFF;
							}
						}
			}

			if(HOUR == 24)
			{
				HOUR = 0;
				
				if(SD_DETECTION_FLAG == 1)
				{
					FIL_NUM++;
					//SD_DETECTION_FLAG=0;
					NEW_FILE_FLAG = 1;

					
						MSD_LED_OFF;
						f_close(&fo);
						f_mount(0,"",0);
						if((SD_DETECTION_FLAG=MSD_INIT()))
						{
							ST = NEW_FILE_OPEN;
							NEW_FILE_FLAG = 0;
							if(!SDWR_OP())
							{
								SD_DETECTION_FLAG = 0;
								f_close(&fo);
								f_mount(0,"",0);
								MSD_LED_OFF;
							}
						}

				}

				/*
				DAY++;
				if(DAY == (DAY_MONTH()+1))
				{
					DAY = 1;
					MONTH++;

					if(MONTH == 13)
					{
						MONTH = 1;
						YEAR++;
					}
				}
				*/
			}
		}
		//SAVE_TIM();
	
		if(SD_DETECTION_FLAG)			
		{
			
			if(ST == READ_PRESSURE)
			{				
				if(SDWR_OP() != 1)
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
		}

		/*if(MIN == TIM_SDWR_PERIOD)
		{
			//SAVE_TIM();
			if(SD_DETECTION_FLAG)
			{
				if(SDWR_OP() != 1)
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
		}*/
    }
    
}

//Question 1) When exactly TIMER2_COMP_INTERRUPT occur?
//				=> TIMER2_COMP_INTERRUPT occur every 3.2msec
ISR(TIMER2_COMP_vect)
{	
    static unsigned char FND_INDEX = 0;		//FND_INDEX 
    static unsigned char BLINK_CNT = 0;		//BLINK_CNT 
    
	//_delay_ms(5);

    if((!FND_BLINK_FLAG)||(BLINK_CNT<30))							//FND_BLINK_FLAG
    {
        switch(FND_INDEX++)					//FND_INDEX 
        {
            case 0:
                FND = FND_D0;
				FND_C = (FND_C&0x0f)| FND_C0;
                if(FND_DOT0)
                    DOT_ON;
                else
                    DOT_OFF;            
                break;
            case 1:
                FND = FND_D1;
				FND_C = (FND_C&0x0f)|FND_C1;
                if(FND_DOT1)
                    DOT_ON;
                else
                    DOT_OFF;
                break;
            case 2:
                FND = FND_D2;
				FND_C = (FND_C&0x0f)|FND_C2;
                if(FND_DOT2)
                    DOT_ON;
                else
                    DOT_OFF;
                break;
            case 3:
                FND = FND_D3;
				FND_C = (FND_C&0x0f)| FND_C3;
                DOT_OFF;        
                break;
            default:
                break;
        }
    }
    else
    {
        FND_INDEX++; 
        ALL_FND_OFF;
    }

    if(FND_INDEX == 4)					//if(FND_INDEX == 4)
    {										//if((!FND_BLINK_FLAG)||(BLINK_CNT<30))  ==> Without condition
        if(FND_BLINK_FLAG)		
        {			
			//FND_OP(4000+BLINK_CNT);			//minuk
			//_delay_ms(50);//minuk				//BLINK_CNT	
						
            BLINK_CNT++;

            if(BLINK_CNT==40)
                BLINK_CNT = 0;              
        }
        else
            BLINK_CNT = 0;           
        
		FND_INDEX = 0;				
										
    }
}



ISR(ADC_vect)
{
    ADC_FLAG = 1;
}

ISR(TWI_vect)
{	
	I2C_FLAG = 1;

	FND = FND_4;
    TWCR = 0x44;
	WD_ON(I2C_WD);
    while(((TWCR&0x80)==0x00) || ((TWSR&0xf8) != 0x60))

	FND = FND_5;
    TWCR = 0xc4;
    while(((TWCR&0x80) == 0x00) || ((TWSR&0xf8) != 0x80));FND = FND_6;
    I2C_DAT_BUF = TWDR;
    TWCR = 0xc4;
    while(((TWCR&0x80) == 0x00) || ((TWSR&0xf8) != 0xa0));FND = FND_7;

    TWCR = 0xc4;
}



/////////////////////////////////////////////////////////////////////////  

int main(void)
{   

    STATE = SYSTEM_INIT;
	

    while(1)
	{
		switch(STATE)
        {
            case SYSTEM_INIT :                                           
            {
                SYS_INIT();
				
				if(I2C_ADDR == MASTER)				STATE = SENSOR_CHECKING;					   
				else
				{
					SLA_VAL_CH[I2C_ADDR] = 3;
					STATE = SLAVE_WAITING;
				}
                break;
            }
			case MASTER_VALVE_SET:			//STOP_POINT_SET mode가 되면 STOP_POINT_SET_OP()!	//STOP_POINT_SET_OP()에서 SET_SW_FLAG[SW0]을 누르면
			{								//MASTER_VALVE_SET모드가 되고, 동시에 밸브채널 설정을한다.
					SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
					
					if ( I2C_ADDR == MASTER)
					{
						VALVE_CH_SET_OP();		//np01		//Master_Board => VALVE_CH_SET_OP()
						SLA_VALVE_CH_SET_OP();				//Slave_Board => SLA_VAVLE_CH_SET_OP()			??????
						STATE = SENSOR_CHECKING;
						break;
					}
							//if( I2C_ADDR == SLAVE) ==> SLAVE_WAITING
					STATE = SLAVE_WAITING;
					break;
			}
			case SLAVE_WAITING:
			{		
					
					FND_OP((I2C_ADDR*100)+SLA_VAL_CH[I2C_ADDR]);
					TWI_DATA_BUF = twi_slave_read(I2C_ADDR);
					
					switch(TWI_DATA_BUF)
					{		
						case SLAVE_CH_SEND:
						{
							twi_slave_write(I2C_ADDR,SLAVE_CH_ASK);			
							TWI_DATA_BUF = twi_slave_read(I2C_ADDR);			
							SLA_VAL_CH[I2C_ADDR] = TWI_DATA_BUF;
							STATE = SLAVE_WAITING;
							break;
						}
						case VALVE_START:
						{
							while(1)
							{
									twi_slave_write(I2C_ADDR,SLAVE_READY);
									TWI_DATA_BUF  = twi_slave_read(I2C_ADDR);
									
									if(valve_index<10)		VALVE = valve_index|0xf0;			
									else	           		VALVE = ((valve_index%10)<<4)|0x0f;
            					    
									_delay_ms(ON_TIME);
        							ALL_VALVE_OFF;

									if ( TWI_DATA_BUF == VALVE_ON)
									{
										valve_index++;
										FND_OP(valve_index);
									}
									else if ( TWI_DATA_BUF == SLAVE_SLEEP)
									{
										valve_index = 0;
										break;
									}
																
							}
							
							STATE = SLAVE_WAITING;
							break;
						}
						default:
						{
							break;
						}
					}				


					FND_OP((I2C_ADDR*100)+SLA_VAL_CH[I2C_ADDR]);
					_delay_ms(100);
					TWI_DATA_BUF = 0;
					
				STATE = SLAVE_WAITING; 
				break;
			}
			case VALVE_ON :
            {
                SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
                VALVE_OP(TIM_PULSING_FLAG);
				STATE = SENSOR_CHECKING;
			}
			case SENSOR_CHECKING:
			{				
						if(SET_SW_FLAG)
		                {
		                    ALL_FND_OFF;
		                    TCCR2 = 0;

		                    _delay_ms(1000);

		                    TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
		                    
							SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
		                    EIFR = 0xff;
		                    STATE = ON_TIME_SET;                        
		                }
		                else
		                {
		                   ST = READ_PRESSURE;
                    		SENSOR_CHECK(1);
							SD_BUF_PRT();           
		               	}
				break;
			}
			case ON_TIME_SET:
			{
				SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
                ALL_LED_OFF;

                if(ON_TIME_SET_OP())
                    STATE = OFF_TIME_SET;
                else							//(WD_FLAG = 1) ==> return 0 ==> ON_TIME_SET_OP() ==> 0 
                    STATE = SENSOR_CHECKING;
                break;
			}
			case OFF_TIME_SET:
			{
				 SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
                ALL_LED_OFF;
                if(OFF_TIME_SET_OP())
                    STATE = START_POINT_SET;
                else
                    STATE = SENSOR_CHECKING;
                break;
			}
			case START_POINT_SET:
			{
				SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
                ALL_LED_OFF;
                if(START_POINT_SET_OP())
                    STATE = STOP_POINT_SET;
                else

                    STATE = SENSOR_CHECKING;
                break;
			}
			case STOP_POINT_SET:
			{
				SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
                ALL_LED_OFF;
                if(STOP_POINT_SET_OP())
                    STATE = MASTER_VALVE_SET;
                else
                    STATE = SENSOR_CHECKING;
                break;
			}
			default:
            {
                SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;

				if (I2C_ADDR == MASTER )               STATE = SENSOR_CHECKING;
                else									STATE = SLAVE_WAITING;
				break;
            }
		}
                
       }  
	return 0;
}

unsigned char ON_TIME_SET_OP(void)		
{
    unsigned int TIME_BUF = ON_TIME;
    FND_DOT2 = 1;
    FND_OP(ON_TIME/100);

    while(1)
    {
	//np3
        WD_ON(KEY_WD);
        while(!(SET_SW_FLAG|UP_SW_FLAG|OK_SW_FLAG|DOWN_SW_FLAG))
        {
            if(WD_FLAG)
            {										//np4
                FND_DOT2 = 0;				//fnd_dot2 ?
                ALL_FND_OFF;				//all_fnd_off
                TCCR0 = 0;					//timer/counter0 off
                TCCR2 = 0;					//timer/counter2 off

                _delay_ms(1000);			//about 1sec ==> 

                TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);	//timer/counter2 on
                FND_OP(0);				//101
                return 0;    
            }            
        }
        if(UP_SW_FLAG)        //UP_SW
        {
            if(TIME_BUF < 900)
            {
                TIME_BUF += 100;
                FND_OP(TIME_BUF/100);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);                
            }               
        }
        else if(DOWN_SW_FLAG)   //DOWN_SW
        {
            if(TIME_BUF > 100)
            {
                TIME_BUF -= 100;
                FND_OP(TIME_BUF/100);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);
            }
        }
        else if(OK_SW_FLAG)   //OK_SW
        {
		//np2
            ON_TIME = TIME_BUF;				//TIME_BUF는 UP_SW_FLAG와 DOWN_SW_FLAG에 의해서 결정
            FND_BLINK_FLAG = 1;				
            _delay_ms(2000);	//minuk
            FND_BLINK_FLAG = 0;
			ST = ON_TIME_RESET;
			if(SD_DETECTION_FLAG)
			{
				if(!SDWR_OP())
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
        }
        else if(SET_SW_FLAG)   //SET_SW
        {
            FND_DOT2 = 0;
            ALL_FND_OFF;
            
			TCCR2 = 0;
            _delay_ms(1000);
            TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
            
			SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;   
            EIFR = 0xff;    
            FND_OP(0);
            return 1;                        
        }
        _delay_ms(200);
        SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
        FAULT_LED_OFF;
        EIFR = 0xff;       
    }
}
/////////////////////////////////////////////////////////////////////////
unsigned char OFF_TIME_SET_OP(void)
{
    unsigned char TIME_BUF = OFF_TIME;
    FND_OP(OFF_TIME);
    while(1)
    {
        WD_ON(KEY_WD);
        while(!(SET_SW_FLAG|UP_SW_FLAG|OK_SW_FLAG|DOWN_SW_FLAG))
        {
            if(WD_FLAG)
            {                
				ALL_FND_OFF;
                TCCR0 = 0;
                TCCR2 = 0;
                
				_delay_ms(1000);
                
				TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);     
                FND_OP(0);
                return 0;    
            }
        }
        if(UP_SW_FLAG)        //UP_SW
        {
            if(TIME_BUF < 100)
            {
                TIME_BUF += 5;
                FND_OP(TIME_BUF);          
            }
            else                                 
            {
                FAULT_LED_ON;
                _delay_ms(800);
            }               
        }
        else if(DOWN_SW_FLAG)   //DOWN_SW
        {
            if(TIME_BUF > 5)
            {
                TIME_BUF -= 5;
                FND_OP(TIME_BUF);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);
            }
        }
        else if(OK_SW_FLAG)   //OK_SW
        {
            OFF_TIME = (unsigned long)TIME_BUF;
            FND_BLINK_FLAG = 1;
            _delay_ms(2000);
            FND_BLINK_FLAG = 0;
			ST = OFF_TIME_RESET;
			if(SD_DETECTION_FLAG)
			{
				if(!SDWR_OP())
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
        }
        else if(SET_SW_FLAG)   //SET_SW
        {
            ALL_FND_OFF;
            TCCR2 = 0;
            _delay_ms(1000);
            TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
            SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
            EIFR = 0xff;
            FND_OP(0);
            return 1;                        
        }        
        _delay_ms(200);
        SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
        FAULT_LED_OFF;
        EIFR = 0xff;       
    }
}

/////////////////////////////////////////////////////////////////////////
void VALVE_CH_SET_OP(void)
{
//np
							//3
   unsigned char CH_BUF = DEFAULT_VALVE_CH;
				//VALVE CH_ SET		

    FND_OP(SLA_VAL_CH[0]);
    while(1)
    {
        WD_ON(KEY_WD);			//워치독 온
        
		while(!(SET_SW_FLAG|UP_SW_FLAG|OK_SW_FLAG|DOWN_SW_FLAG))	//스위치 눌린거 없을때XXXX what? 
        {
            if(WD_FLAG)	//그리고 워치독 플래그가 들어올때
            {
                ALL_FND_OFF;
	            //#define ALL_FND_OFF     (FND = 0x00)
		 
			    TCCR0 = 0;
                TCCR2 = 0;

                _delay_ms(1000);
                
				TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);  
                FND_OP(0);
                return;    
            }
        }
        if(UP_SW_FLAG)        //UP_SW
        {
            if(CH_BUF < 20)
            {
                CH_BUF += 1;
                FND_OP(CH_BUF);
            }
            else
            {
                FAULT_LED_ON;
             	//#define FAULT_LED_ON    (PORTG &= 0x0f)
			    _delay_ms(800);                
            }                
        }
        else if(DOWN_SW_FLAG)   //DOWN_SW
        {
            if(CH_BUF > 1)
            {
                CH_BUF -= 1;
                FND_OP(CH_BUF);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);
            }
        }
        else if(OK_SW_FLAG)   //OK_SW
        {
			//np1
            SLA_VAL_CH[0] = CH_BUF;		//SW0(OK_SW)를 누르면 CH_BUF를 SLA_VAL_CH[0]에 값을 넣음.
			
			ST = VALVE_CH_RESET;		//ST = VALVE_CH_RESET
											//==> LOAD /SD_CARD 
			if(SD_DETECTION_FLAG)
			{
				if(!SDWR_OP())
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
            FND_BLINK_FLAG = 1;
            _delay_ms(2000);
            FND_BLINK_FLAG = 0;
        }
        else if(SET_SW_FLAG)   //SET_SW
        {
            ALL_FND_OFF;
            TCCR2 = 0;
            _delay_ms(1000);
            TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
            SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
            EIFR = 0xff;		//External Interrupt Flag Register = 0xff;
								//To clear External Interrupt EIFR = 0xff. 
								//why?		
            FND_OP(0);
            return;                        
        }
        _delay_ms(200);
        SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
        FAULT_LED_OFF;
        EIFR = 0xff;     
    }  
}

/////////////////////////////////////////////////////////////////////////

unsigned char START_POINT_SET_OP(void)
{
    unsigned char point = START_OP_POINT;
    FND_OP(point);
    while(1)
    {
        WD_ON(KEY_WD);
        while(!(SET_SW_FLAG|UP_SW_FLAG|OK_SW_FLAG|DOWN_SW_FLAG))
        {
            if(WD_FLAG)
            {
                ALL_FND_OFF;
                TCCR0=0;
                TCCR2 = 0;
                
				_delay_ms(1000);
                
				TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);  
                FND_OP(0);
                return 0;    
            }
        }
        if(UP_SW_FLAG)        //UP_SW
        {
            if(point < WARNING_HIGH_POINT)
            {
                point += 5;
                FND_OP(point);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);                
            }                
        }
        else if(DOWN_SW_FLAG)   //DOWN_SW
        {
            if(point > STOP_OP_POINT+5)
            {
                point -= 5;
                FND_OP(point);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);
            }
        }
        else if(OK_SW_FLAG)   //OK_SW
        {
            START_OP_POINT = point;
            FND_BLINK_FLAG = 1;
            _delay_ms(2000);
            FND_BLINK_FLAG = 0;
			ST = START_POINT_RESET;
			if(SD_DETECTION_FLAG)
			{
				if(!SDWR_OP())
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
        }
        else if(SET_SW_FLAG)   //SET_SW
        {
            ALL_FND_OFF;
            TCCR2 = 0;
            _delay_ms(1000);
            TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
            SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
            EIFR = 0xff;    
            FND_OP(0);
            return 1;                        
        }
        _delay_ms(200);
        SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
        FAULT_LED_OFF;
        EIFR = 0xff;     
    }  
}
/////////////////////////////////////////////////////////////////////////
unsigned char STOP_POINT_SET_OP(void)
{
    unsigned char point = STOP_OP_POINT;
    FND_OP(point);
    while(1)
    {
        WD_ON(KEY_WD);
       
	    while(!(SET_SW_FLAG|UP_SW_FLAG|OK_SW_FLAG|DOWN_SW_FLAG))
        {
            if(WD_FLAG)
            {
                ALL_FND_OFF;
                TCCR0 = 0;
                TCCR2 = 0;

                _delay_ms(1000);
 
                TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
                FND_OP(0);
                return 0;    
            }
        }

        if(UP_SW_FLAG)        //UP_SW
        {
            if(point < START_OP_POINT-5)
            {
                point += 5;
                FND_OP(point);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);                
            }                
        }
        else if(DOWN_SW_FLAG)   //DOWN_SW
        {
            if(point > WARNING_LOW_POINT)
            {
                point -= 5;
                FND_OP(point);
            }
            else
            {
                FAULT_LED_ON;
                _delay_ms(800);
            }
        }
        else if(OK_SW_FLAG)   //OK_SW
        {
            STOP_OP_POINT = point;		//User adjust the configuration 

            FND_BLINK_FLAG = 1;
            _delay_ms(2000);
            FND_BLINK_FLAG = 0;

			ST = STOP_POINT_RESET;
			if(SD_DETECTION_FLAG)
			{
				if(!SDWR_OP())
				{
					SD_DETECTION_FLAG = 0;
					f_close(&fo);
					f_mount(0,"",0);
					MSD_LED_OFF;
				}
			}
        }
        else if(SET_SW_FLAG)   //SET_SW			//STOP_POINT_SET_OP()에서 리턴1이되려면 SET_SW_FLAG [SW0] ! 
        {
            ALL_FND_OFF;
            TCCR2 = 0;
            _delay_ms(1000);
            TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
            SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
 
            EIFR = 0xff;    
 
            FND_OP(0);
            return 1;                        
        }
        _delay_ms(200);
        SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
        FAULT_LED_OFF;
        EIFR = 0xff;   
    }
}
/////////////////////////////////////////////////////////////////////////

void SLA_VALVE_CH_SET_OP(void)
{
    unsigned char CH_BUF = 3;
	unsigned char SLA_DEV_INDEX = 0;
	unsigned int SLA_CH_SUM_BUF = 0;
	
	for ( int i = 0; i < 17 ; i++)
	{
		if (SLAVE_ADDR_CHECK & 1<<(i))			//np01
		{
				SLA_DEV_INDEX = (i+1)*100;
				CH_BUF = SLA_VAL_CH[i+1];
				FND_OP(SLA_DEV_INDEX+CH_BUF);
				_delay_ms(100);	

    		while(1)
    		{
        		WD_ON(KEY_WD);
        		while(!(SET_SW_FLAG|UP_SW_FLAG|OK_SW_FLAG|DOWN_SW_FLAG))
        		{
            		if(WD_FLAG)
            		{
                		ALL_FND_OFF;
                		TCCR0 = 0;
                		TCCR2 = 0;

                		_delay_ms(1000);

                		TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);  
                		FND_OP(0);
                		return;    
            		}
        		}
        		if(UP_SW_FLAG)        //UP_SW
        		{
            		if(CH_BUF < 20)
            		{
                		CH_BUF += 1;
                		FND_OP(SLA_DEV_INDEX+CH_BUF);
            		}
            		else
            		{
                		FAULT_LED_ON;
                		_delay_ms(800);                
            		}                
        		}
		        else if(DOWN_SW_FLAG)   //DOWN_SW
		        {
		            if(CH_BUF > 1)
		            {
		                CH_BUF -= 1;
		                FND_OP(SLA_DEV_INDEX+CH_BUF);
		            }
		            else
		            {
		                FAULT_LED_ON;
		                _delay_ms(800);
		            }
		        }
		        else if(OK_SW_FLAG)   //OK_SW
		        {
					ST = VALVE_CH_RESET;
					if(SD_DETECTION_FLAG)
					{
					while(NEW_FILE_FLAG);
						if(!SDWR_OP())
						{
						
							SD_DETECTION_FLAG = 0;
							f_close(&fo);
							f_mount(0,"",0);
							MSD_LED_OFF;
						}
					}
		            FND_BLINK_FLAG = 1;
		            _delay_ms(2000);
		            FND_BLINK_FLAG = 0;
			
					SLA_VAL_CH[i+1] = CH_BUF;
					for ( int j = 0 ; j < 17; j++)
					{
						SLA_CH_SUM_BUF += SLA_VAL_CH[j];
					}
					SLA_CH_SUM  =  SLA_CH_SUM_BUF;
					for ( int slave_index = 0 ; slave_index < 16 ; slave_index++)
							{
								if (SLAVE_ADDR_CHECK & 1<<(slave_index))	
								{
									twi_master_write(slave_index+1,SLAVE_CH_SEND);
																_delay_ms(100);
								
									TWI_DATA_BUF = twi_master_read(slave_index+1);
									if ( TWI_DATA_BUF == SLAVE_CH_ASK )
									{
									_delay_ms(100);
										twi_master_write(slave_index+1,CH_BUF);
									}
									else	;
								}
							}
							
						ST = SLAVE_CH_RESET;
						if(SD_DETECTION_FLAG)
						{			
							if(!SDWR_OP())
							{
								SD_DETECTION_FLAG = 0;
								f_close(&fo);
								f_mount(0,"",0);
								MSD_LED_OFF;
							}
						}	
					SLA_CH_SUM_BUF = 0;
					CH_BUF = 0;
		        }
		        else if(SET_SW_FLAG)   //SET_SW
		        {
		            ALL_FND_OFF;
		            TCCR2 = 0;
		            _delay_ms(1000);
		            TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
		            SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
		            EIFR = 0xff;    
		            FND_OP(0);
		            return;                        
		        }
		        _delay_ms(200);
		        SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;
		        FAULT_LED_OFF;
		        EIFR = 0xff;     
		    } 
			}
		} 
}

void VALVE_OP(unsigned char TIM_PUL_MODE_FLAG)
{
    unsigned char tim_cnt=0;

	if ( I2C_ADDR == MASTER)
	{
		if ( SLAVE_ADDR_CHECK == 0 )	 SLA_CH_SUM = SLA_VAL_CH[0];

		while(valve_index < SLA_CH_SUM)
    	{
				FND_OP((unsigned int)valve_index+1);
				ST = VALVE_OPEN;
				if(SD_DETECTION_FLAG)
				{
					while(NEW_FILE_FLAG);
					if(!SDWR_OP())
					{
						SD_DETECTION_FLAG = 0;
						f_close(&fo);
						f_mount(0,"",0);
						MSD_LED_OFF;
					}
				}
					
				if (valve_index < SLA_VAL_CH[0])		
				{
				
	    			if(valve_index<10) 			VALVE = valve_index|0xf0;	
					else            			VALVE = ((valve_index%10)<<4)|0x0f;

						_delay_ms(ON_TIME);
        				ALL_VALVE_OFF;
        		}		
				else	//master valve_ch OP
				{
							for ( int slave_index = 0 ; slave_index < 16 ; slave_index++)
							{
								if (SLAVE_ADDR_CHECK & 1<<(slave_index))	
								{								
										if ( valve_on_flag == 0)
										{		
											twi_master_write(slave_index+1,VALVE_START);
											valve_on_flag = 1;
										}	
											_delay_ms(100);
											
											TWI_DATA_BUF = twi_master_read(slave_index+1);
											
											if ( TWI_DATA_BUF == SLAVE_READY)
											{	
												FND_OP((unsigned int)valve_index+1);
																
												if (valve_index+1 == SLA_CH_SUM)
												{	
													twi_master_write(slave_index+1,SLAVE_SLEEP);
													valve_on_flag = 0;																				
												}
												else	twi_master_write(slave_index+1,VALVE_ON);
											}
								}
							}
				}
						for(tim_cnt=0; tim_cnt<OFF_TIME-1; tim_cnt++)
						{
							if(tim_cnt == (OFF_TIME-1)/2)
								SENSOR_CHECK(1);
				            _delay_ms(1000);    
				        }

						ST = VALVE_CLOSE;

						if(SD_DETECTION_FLAG)
						{
							while(NEW_FILE_FLAG);
							if(!SDWR_OP())
							{
								SD_DETECTION_FLAG = 0;
								f_close(&fo);
								f_mount(0,"",0);
								MSD_LED_OFF;
							}
						}
				        
						if((++valve_index) == SLA_CH_SUM)
				            valve_index = 0;
            
				        _delay_ms(400);    
           																					
						//SET_SW_FLAG = 1;
						if(TIM_PUL_MODE_FLAG)					//ncp0
						{
							//STATE = ON_TIME_SET;	//minuk
							
							if(SET_SW_FLAG == 1)
							{
								//STATE = ON_TIME_SET;
								SET_SW_FLAG = 0;
								OFF_TIME_SET_OP();
								//haha = 1;						
							}

							if(0x00 == (PINE&0x08))
							{
								TIM_PULSING_FLAG = 0;			
								WARNING_LED_OFF;		//hw0
								
								FND_BLINK_FLAG = 1;
								_delay_ms(2000);
								FND_BLINK_FLAG = 0;
							
								break;
							} 
						}
						else
						{
							if(MMAQ < STOP_OP_POINT)
				            
							break;
						}              
		}                
    } 	
	ALL_FND_OFF;
    TCCR2 = 0;
    _delay_ms(1000);
    TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);      
    SET_SW_FLAG=UP_SW_FLAG=OK_SW_FLAG=DOWN_SW_FLAG=0;	
   
}

void SD_BUF_PRT(void)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
{
	sprintf(dat_buf.tim,"%02d:%02d\t", HOUR,MIN);

	switch(ST)
	{
		case  POWER_ON :					//[0] POWER ON
			sprintf(dat_buf.status, "POWER_ON\t");
			sprintf(dat_buf.note, "MASTER VALV_CH=%d, SLAVE DEV NUM=%d , EXT=%d, MSD=%d\t",SLA_VAL_CH[MASTER],I2C_SL_NUM,I2C_ADDR,SD_DETECTION_FLAG);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  READ_PRESSURE :				//[1] READ SENSOR VALUE
			sprintf(dat_buf.status, "READ_PRESSURE\t");
			sprintf(dat_buf.note, "-\t");
			sprintf(dat_buf.pressure, "%dmmH2O\r\n",MMAQ);
			break;
		case  VALVE_OPEN :					//[2] VALVE OPERATING
			sprintf(dat_buf.status, "VALVE_OPEN\t");
			if(TIM_PULSING_FLAG)
				sprintf(dat_buf.note, "VALV=%d, MODE_TC\t",valve_index+1);
			else
				sprintf(dat_buf.note, "VALV=%d, %d MODE_DP\t",valve_index+1,valve_count);
			sprintf(dat_buf.pressure, "%dmmH2O\r\n",MMAQ);
			break;
		case  VALVE_CLOSE :					//[2] VALVE OPERATING
			sprintf(dat_buf.status, "VALVE_CLOSE\t");
			if(TIM_PULSING_FLAG)
				sprintf(dat_buf.note, "VALV=%d, MODE_TC\t",valve_index+1);
			else
				sprintf(dat_buf.note, "VALV=%d, MODE_DP\t",valve_index+1);
			sprintf(dat_buf.pressure, "%dmmH2O\r\n",MMAQ);
			break;
		case  ON_TIME_RESET :
			sprintf(dat_buf.status, "ON_TIME_RESET\t");
			sprintf(dat_buf.note, "ON_TIME_VALUE = %dmsec\t",ON_TIME);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  OFF_TIME_RESET :
			sprintf(dat_buf.status, "OFF_TIME_RESET\t");
			sprintf(dat_buf.note, "OFF_TIME_VALUE = %dsec\t",OFF_TIME);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  VALVE_CH_RESET :
			sprintf(dat_buf.status, "VALVE_CH_RESET\t");
			sprintf(dat_buf.note, "VALVE_CH_SET_VALUE=%d\t",SLA_VAL_CH[MASTER]);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  SLAVE_CH_RESET :
			sprintf(dat_buf.status, "VALVE_CH_RESET\t");
			for ( int i = 0 ; i < 17 ; i++)
			{
				if (SLAVE_ADDR_CHECK & 1<<(i))
				{
					sprintf(dat_buf.note, "SLAVE_VALVE_CH_SET_VALUE=%d\t",SLA_VAL_CH[i+1]);
				}
			}
			sprintf(dat_buf.pressure, "-\r\n");
			break; 
		case  START_POINT_RESET :
			sprintf(dat_buf.status, "OP_START_POINT_RESET\t");
			sprintf(dat_buf.note, "OP_START_VALUE=%dmmH2O\t",START_OP_POINT);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  STOP_POINT_RESET :
			sprintf(dat_buf.status, "OP_STOP_POINT_RESET\t");
			sprintf(dat_buf.note, "OP_STOP_VALUE=%dmmH2O\t",STOP_OP_POINT);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  MSD_DETECTION :
			sprintf(dat_buf.status, "MSD_DETECTION\t");
			sprintf(dat_buf.note, "MSD=%d\t",SD_DETECTION_FLAG);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case  MSD_NON_DETECTION :
			sprintf(dat_buf.status, "MSD_DETECTION_FAIL\t");
			sprintf(dat_buf.note, "MSD=%d\t",SD_DETECTION_FLAG);
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		case NEW_FILE_OPEN :
			sprintf(dat_buf.status, "NEW_FILE_CREATED\t");
			//sprintf(dat_buf.note, "VALV_CH=%d, EXT=%d, MSD=%d\t",VALVE_CH,I2C_ADDR,SD_DETECTION_FLAG);
			for ( int i = 0 ; i < 17 ; i ++)
			{
				TOTAL_VALVE += SLA_VAL_CH[i];
			}
			sprintf(dat_buf.note, "VALV_CH=%d, EXT=%d, MSD=%d\t",TOTAL_VALVE,I2C_ADDR,SD_DETECTION_FLAG);
			TOTAL_VALVE = 0;
			sprintf(dat_buf.pressure, "-\r\n");
			break;
		default :
			
		break;
	}
}
/////////////////////////////////////////////////////////////////////////
unsigned char SDWR_OP(void)
{
	unsigned int bw;
	SD_WRITTING_FLAG = 1;
	TIM_SDWR_FLAG = 0;

	SD_BUF_PRT();
	cli();	
	fr = f_write(&fo,dat_buf.tim,strlen(dat_buf.tim),&bw);
	fr = f_write(&fo,dat_buf.status,strlen(dat_buf.status),&bw);
	fr = f_write(&fo,dat_buf.note,strlen(dat_buf.note),&bw);
	fr = f_write(&fo,dat_buf.pressure,strlen(dat_buf.pressure),&bw);
	fr = f_sync(&fo);
	SD_WRITTING_FLAG  = 0;
	
	sei();
	if(fr == FR_OK)
		return 1;
	else
		return 0;
}
/////////////////////////////////////////////////////////////////////////

void TWI_INIT(void)
{
	TWBR = 0x72;
	TWSR = 0x00;
}

void TWI_ERROR(void)
{
	TWCR=((1<<TWINT)|(1<<TWEN)|(1<<TWSTO)); 	//TWI - Stop 
	TWCR=(0<<TWEN); 							//TWI - Aus

	return ;
}
void TWI_MW(unsigned char addr, unsigned char dat)
{
   		TWCR = 0xa4;                    // START
        
		WD_ON(I2C_WD);
		
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x08))
		{
			if(WD_CHECK())		//WD_FLAG == 1 ==> return 1 
			{
				TWI_ERROR();
				return;
			}
		}
        TWDR = addr << 1;               // ADDRESS
        TWCR = 0x84;
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x18))
		{
			if(WD_CHECK())
			{
				TWI_ERROR();
				return;
			}
		}
        TWDR = dat;                     // WRITE
        TWCR = 0x84;
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x28))
		{
			if(WD_CHECK())
			{
				TWI_ERROR();
				return;
			}
		}
        TWCR = 0x94;
}
/////////////////////////////////////////////////////////////////////////

void TWI_SW(unsigned char addr, unsigned char dat)
{
    TWAR = addr << 1;
        TWCR = 0x44;                    // SLAVE
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0xa8));
        TWDR = dat;
        TWCR = 0xc4;
		WD_ON(I2C_WD);
        while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0xc0))
		{
			if(WD_CHECK())
			{
				TWI_ERROR();
				return;
			}
		}
        TWCR = 0xc4;
}

/////////////////////////////////////////////////////////////////////////
unsigned char TWI_MR(unsigned char addr)
{
     unsigned char dat;
        
        TWCR = 0xa4;                    // START
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x08))
		{
    		if(WD_CHECK())
			{
				TWI_ERROR();
				return 0;
			}
		}
        TWDR = (addr << 1) | 1;
        TWCR = 0x84;
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x40))
        {
			if(WD_CHECK())
			{
				TWI_ERROR();
				return 0;
			}
		}
		
		TWCR = 0x84;
		WD_ON(I2C_WD);
        while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x58))
		{
			if(WD_CHECK())
			{
				TWI_ERROR();
				return 0;
			}
		}
        dat = TWDR;
        TWCR = 0x94;
        return dat;
}

/////////////////////////////////////////////////////////////////////////
unsigned char TWI_SR(unsigned char addr)
{
    unsigned char dat;
        
        TWAR = addr << 1;
        TWCR = 0x44;                    // SLAVE
        
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x60));
        TWCR = 0xc4;
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x80))
		{
			if(WD_CHECK())
			{
				TWI_ERROR();
				return 0;
			}
		}
        dat = TWDR;                     // READ
        TWCR = 0xc4;
        WD_ON(I2C_WD);
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0xa0))
		{
			if(WD_CHECK())
			{
				TWI_ERROR();
				return 0;
			}
		}
        TWCR = 0xc4;
        return dat;
}

unsigned char twi_master_read(unsigned char addr)
{
        unsigned char dat;
        
        TWCR = 0xa4;                    // START
        
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x08));
        TWDR = (addr << 1) | 1;
        TWCR = 0x84;
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x40));
		TWCR = 0x84;
        while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x58));
        dat = TWDR;
        TWCR = 0x94;
        return dat;
}

void twi_master_write(unsigned char addr, unsigned char dat)
{
        TWCR = 0xa4;                    // STAR
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x08));
        TWDR = addr << 1;               // ADDRESS
        TWCR = 0x84;
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x18));
        TWDR = dat;                     // WRITE
        TWCR = 0x84;
 
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x28));
        TWCR = 0x94;
}

unsigned char twi_slave_read(unsigned char addr)
{
        unsigned char dat;
        
        TWAR = addr << 1;
        TWCR = 0x44;                    // SLAVE
        
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x60));
        TWCR = 0xc4;
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0x80));
        dat = TWDR;                     // READ
        TWCR = 0xc4;
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0xa0));
        TWCR = 0xc4;
        return dat;
}
void twi_slave_write(unsigned char addr, unsigned char dat)
{
        TWAR = addr << 1;
        TWCR = 0x44;                    // SLAVE
		while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0xa8));
        TWDR = dat;
        TWCR = 0xc4;
        while(((TWCR & 0x80) == 0x00) || ((TWSR & 0xf8) != 0xc0));
        TWCR = 0xc4;
}

/////////////////////////////////////////////////////////////////////////
void SYS_INIT(void)
{
   unsigned int index=0;
    
    cli();
    IO_INIT();
    INT_INIT();
    ADC_INIT();
    TIM_INIT();
	TWI_INIT();

	dat_buf.pressure = (char*)malloc(sizeof(char)*10);
	dat_buf.tim = (char*)malloc(sizeof(char)*20);
	dat_buf.status = (char*)malloc(sizeof(char)*30);
	dat_buf.note = (char*)malloc(sizeof(char)*20);
	sprintf(dat_buf.note,"%d개\r\n",VALVE_CH);
	sei();
	
    for(index=0; index<=9999; index+=1111)
    {
        PORTG = (~PORTG)&0x18;
        FND_OP(index);
		_delay_ms(200); 
    }


   	SET_SW_FLAG = UP_SW_FLAG = OK_SW_FLAG = DOWN_SW_FLAG=0;

	if((SD_DETECTION_FLAG=MSD_INIT()))
	{
		FND_OP(1000+EXT_INIT(READ_ADDR()));		
	}
	else
	{
		
		FND_OP(EXT_INIT(READ_ADDR()));
	}

	_delay_ms(3000);

	FND_BLINK_FLAG = 1;
	_delay_ms(1000);
	FND_BLINK_FLAG = 0;
	_delay_ms(1000);
	PC_VCC_ON;

	ST = POWER_ON;
		
	if(SD_DETECTION_FLAG)
	{	
		SDWR_OP();
		TIM_SDWR_INIT();		
	}
}
/////////////////////////////////////////////////////////////////////////
unsigned char EXT_INIT(unsigned char addr)
{							//0000xxxx

		unsigned char SLA_CHECK_INDEX=0;
		unsigned char I2C_DAT_BUF = 0;
	
	I2C_ADDR = addr;
				//MASTER = 0;
				//if(return 0)
	if(addr == MASTER)								// MASTER MODE
	{
		for(SLA_CHECK_INDEX=1; SLA_CHECK_INDEX <= 16; SLA_CHECK_INDEX++)
		{
			
			TWI_MW(SLA_CHECK_INDEX,CONNECTED);
		
			_delay_ms(100);		
		
			I2C_DAT_BUF = TWI_MR(SLA_CHECK_INDEX);
			
		
			_delay_ms(100);
			if( I2C_DAT_BUF == CONNECTED_OK )
			{			
				I2C_SL_NUM++;	
				SLAVE_ADDR_CHECK |= 1<<(SLA_CHECK_INDEX-1);  // slave addr bit
			}
		
			FND_OP(SLA_CHECK_INDEX);
		}
		return(100+I2C_SL_NUM);
	}	
	else											// SLAVE MODE
	{		
			
			I2C_DAT_BUF = twi_slave_read(I2C_ADDR);
			

			if ( I2C_DAT_BUF == CONNECTED)
			{
				twi_slave_write(I2C_ADDR,CONNECTED_OK);
			}

	}	
	return I2C_ADDR;		
}
/////////////////////////////////////////////////////////////////////////
void IO_INIT(void)
{
    DDRA = 0xff;
    PORTA = 0xFF;   //ALL_VALVE_OFF
               
    DDRB =	0xf7;
    PORTB = 0x00;
    
    DDRC = 0xff;
    PORTC = FND_N;
    
    DDRD &= 0x0f;
    
    DDRE = 0x04;
    PORTE = PORTE|0x04;
    
    DDRG = 0xff;
    PORTG = 0x18;
}
/////////////////////////////////////////////////////////////////////////
void INT_INIT(void)
{
    EICRB = 0xAA;
    EIMSK = 0xF0;
}
/////////////////////////////////////////////////////////////////////////
void ADC_INIT(void)
{
    ADMUX = 0x2f;   //아트메가 differential 사용
    //ADMUX = 0x00;  
    ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

/////////////////////////////////////////////////////////////////////////
void TIM_INIT(void)
{
    OCR2 = 25;
    TIMSK = (1<<OCIE2);
    TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
}
/////////////////////////////////////////////////////////////////////////
unsigned char MSD_INIT(void)
{	
	
	cli();
	FND_C |= 0xf0;
	UINT bw;
	char fil_name[30] = "";

	sprintf(fil_name,"%03d.txt",FIL_NUM);
	
	if((fr=f_mount(&fs, "0:", 0)))
	{
		sei();
		return 0;
		//FND_OP(1000+(int)fr);					//Check! SD Card!
		//while(1);
	}
	
	do
	{
		fr=f_open(&fo, fil_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
		if(fr == FR_OK)
		{
			f_write(&fo,arr,strlen(arr),&bw);
			f_sync(&fo);
			//새파일 생성
		}
		else if(fr == FR_EXIST)
		{
			FIL_NUM++;
			sprintf(fil_name,"%03d.txt",FIL_NUM);
			//내용 추가 
		}
		else
		{
			//sei();
			//FND_OP(100+(int)fr);					//Check! SD Card!
			//while(1);
			sei();
			return 0;
		}
	}while(fr == FR_EXIST);
	
	//f_close(&fo);
	//f_mount(0,"",0);
	sei();
	MSD_LED_ON;
	return 1;
	
}
/////////////////////////////////////////////////////////////////////////
void SENSOR_CHECK(unsigned char fnd_on)
{    
    unsigned int index = 0;  
	int adc_off_cal;  

    ADC_SUM=0;
    for(index=0; index<300; index++)
    {
        ADCSRA |= (1<<ADSC) ;
        while(!ADC_FLAG);
		ADC_FLAG = 0;
        if((ADCW&0x8000))
            ADC_SUM += (0xfffffc00|(ADCW>>6));
        else
            ADC_SUM += (ADCW>>6);
    }
   
	MMAQ = (ADC_SUM / 300 - 86) * 0.7843971231;		//minuk
	
	//parameter	if(SENSOR_CHECK(1) => FND_OP(MMAQ);
    if(fnd_on)
        FND_OP(MMAQ);

	if(!TIM_PULSING_FLAG)	//SENSOR_CHECK()에서 차압펄싱모드인경우 
	{
    	if((MMAQ >= WARNING_HIGH_POINT) || (MMAQ <= WARNING_LOW_POINT))
		{
        	WARNING_LED_ON;
    		TIM_PULSING_FLAG = 1;	//타임펄싱플래그를 1로 SET
		}
	}

	if((MMAQ >= START_OP_POINT)||(TIM_PULSING_FLAG))
    {
		STATE = VALVE_ON;
		FAULT_LED_ON;
	}

	if( (MMAQ >= START_OP_POINT) && (MMAQ < WARNING_HIGH_POINT) && (!TIM_PULSING_FLAG))
	{
		valve_count++;
		FND_OP(1000+valve_count);	//minuk
		_delay_ms(500);	
	}
	else	FAULT_LED_OFF;
    _delay_ms(500);   
}
/////////////////////////////////////////////////////////////////////////
void FND_OP(int num)
{
    int buf=num;
    
    FND_D0 = FND_PARSING(buf/1000);
    
    if(buf<0)
    {
        FND_D0 = FND_M;
        buf = -buf;  
    }        
      
    buf %= 1000;    
    FND_D1 = FND_PARSING(buf/100);
    
    buf %= 100;    
    FND_D2 = FND_PARSING(buf/10);
    
    buf %= 10;    
    FND_D3 = FND_PARSING(buf);
}
/////////////////////////////////////////////////////////////////////////
unsigned char FND_PARSING(int num)
{
	switch(num)
	{
		case 0:
			return FND_0;
			
		case 1:
			return FND_1;
		
		case 2:
			return FND_2;
		
		case 3:
			return FND_3;
		
		case 4:
			return FND_4;
		
		case 5:
			return FND_5;
		
		case 6:
			return FND_6;
		
		case 7:
			return FND_7;
		
		case 8:
			return FND_8;
		
		default:
			return FND_9;
			
	}
}
/////////////////////////////////////////////////////////////////////////
void WD_ON(unsigned char tim_parameter)
{
    SEC_CNT = 0;
    WD_SEC = 0;
    WD_FLAG = 0;

	if(tim_parameter == KEY_WD)
	{
		OCR0 = 250;	
	}
	else if(tim_parameter == I2C_WD)
	{
		OCR0 = 4;	
	}
    
    TIMSK |= (1<<OCIE0);		//Output Compare Interrupt Enable

    TCCR0 = (1<<WGM01)|(1<<CS02)|(1<<CS01); //31250 count = 1sec    
			//CTC		//110 // clk/256
}
/////////////////////////////////////////////////////////////////////////
char WD_CHECK(void)
{
	if(WD_FLAG)
    {
        TCCR0 = 0;
        TCCR2 = 0;
        TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS20);
        return 1;    
    }
	else 
		return 0;
}

/////////////////////////////////////////////////////////////////////////
void TIM_SDWR_INIT(void)
{
    OCR1A =  31250;      //31250 count = 1sec
    TIMSK |= (1<<OCIE1A);
    TCCR1A = 0x00;
    TCCR1B = (1<<WGM12)|(1<<CS12);  
    TCCR1C = 0x00;
}
/////////////////////////////////////////////////////////////////////////
int D2AQ(int adc)
{
    return adc*0.93;        //  1.30969283==1mmAq    
}
/////////////////////////////////////////////////////////////////////////
unsigned char READ_ADDR(void)
{
	//PORTD => Pull_Up

	if((PIND&0xf0)==0xf0)	//PORTD => Nothing
		return 0;
	else
		return (PIND>>4)&0x0f;	//상위4개만 검출? 왜?
								//확장보드는 최대 15개까지 확장가능
}
/////////////////////////////////////////////////////////////////////////
void CHECK_TIM(void)
{
	if(EEPROM_RD(0,2) != 0xffff)
	{
		YEAR = EEPROM_RD(0,2);
		MONTH = EEPROM_RD(2,1);
		DAY = EEPROM_RD(3,1);
		HOUR = EEPROM_RD(4,1);
		MIN = EEPROM_RD(5,1);
	}
	else
	{
		SAVE_TIM();	
	}
}
/////////////////////////////////////////////////////////////////////////
void SAVE_TIM(void)
{
	EEPROM_WR(0,2,YEAR);
	EEPROM_WR(2,1,MONTH);
	EEPROM_WR(3,1,DAY);
	EEPROM_WR(4,1,HOUR);
	EEPROM_WR(5,1,MIN);	
}
/////////////////////////////////////////////////////////////////////////
void EEPROM_WR(unsigned int ADDR, unsigned char SIZE_OF_DAT, unsigned int DAT)
{
	unsigned char index=0;

	for(index=SIZE_OF_DAT; index>0; index--)
	{
		while(EECR&(1<<EEWE));  //쓰기 작업중이면 루프
	
		EEAR = ADDR+(2-index); 	    	//쓸 주소값 입력
		EEDR = DAT>>(8*(index-1));		//쓸 데이터 입력
		EECR |= (1<<EEMWE);     		//EEWE  준비
		EECR |= (1<<EEWE);      		//쓰기명령
	}
	return;
}
/////////////////////////////////////////////////////////////////////////
unsigned int EEPROM_RD(unsigned int ADDR, unsigned char SIZE_OF_DAT)
{
	unsigned char index=0;
	unsigned int buf=0;

	for(index=SIZE_OF_DAT; index>0; index--)
	{
		while(EECR&(1<<EEWE));
	
		EEAR = ADDR+(2-index);
		EECR |= (1<<EERE);

		buf |= EEDR<<(8*(index-1));
	}
	
	return buf;
}
/////////////////////////////////////////////////////////////////////////
unsigned char DAY_MONTH(void)
{
	switch(MONTH)
	{
		case 1:	case 3:	case 5:	case 7:	case 8:	case 10: case 12:
		{
			return 31;
		}
		case 2:
		{
			return 28;
		}
		default:
		{
			return 30;
		}
	}
}
