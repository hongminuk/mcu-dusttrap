#define VALVE PORTA
#define VALVE_CH0 0xf0
#define VALVE_CH1 0xf1
#define VALVE_CH2 0xf2
#define VALVE_CH3 0xf3
#define VALVE_CH4 0xf4
#define VALVE_CH5 0xf5
#define VALVE_CH6 0xf6
#define VALVE_CH7 0xf7
#define VALVE_CH8 0xf8
#define VALVE_CH9 0xf9
#define VALVE_CH10 0x0f
#define VALVE_CH11 0x1f
#define VALVE_CH12 0x2f
#define VALVE_CH13 0x3f
#define VALVE_CH14 0x4f
#define VALVE_CH15 0x5f
#define VALVE_CH16 0x6f
#define VALVE_CH17 0x7f
#define VALVE_CH18 0x8f
#define VALVE_CH19 0x9f
#define ALL_VALVE_OFF  (VALVE = 0xff)

//define.h
#define FND             PORTC
#define FND_C           PORTB
#define FND_M			0x40
#define FND_0           0x3f

#define FND_1           0x06
#define FND_2           0x5b
#define FND_3           0x4f
#define FND_4           0x66
#define FND_5           0x6d
#define FND_6           0x7d
#define FND_7           0x27
#define FND_8           0x7f
#define FND_9           0x67
#define FND_N           0x00
#define FND_C0          0xef
#define FND_C1          0xdf
#define FND_C2          0xbf
#define FND_C3          0x7f										//Then  what is "PORTC 8.bit" ? => The far right FND_DOT
#define DOT_ON          (PORTC |= 0x80)		//PORTC 8.bit => set
#define DOT_OFF         (PORTC &= 0x7f)		//PORTC 8.bit => clear

#define ALL_FND_OFF     (FND = 0x00)



#define DEFAULT_ON_TIME             100     //msec						//1
#define DEFAULT_OFF_TIME            5       //sec						//2
#define DEFAULT_TIM_PULSING_P       10     //sec			//
#define DEFAULT_VALVE_CH            3									//5

#define DEFAULT_START_OP_POINT      160     //mmAq 150		//차압pul	//3
#define DEFAULT_STOP_OP_POINT       100     //mmAq	100		//차압pul	//4
#define WARNING_HIGH_POINT          200		//200			//TIM_PUL
#define WARNING_LOW_POINT           -100					//TIM_PUL

#define FAULT_LED_ON    (PORTG &= 0x0f)

#define FAULT_LED_OFF   (PORTG |= 0x10)
#define MSD_LED_ON  (PORTG &= 0x17)
#define MSD_LED_OFF (PORTG |= 0x08)

#define WARNING_LED_ON  (PORTE &= 0xFB)
								//B == 11111011
#define WARNING_LED_OFF (PORTE |= 0x04)
								//00000100

#define ALL_LED_ON      (PORTG &= 0x07)
#define ALL_LED_OFF     (PORTG |= 0x18)

#define PC_VCC_ON		PORTG |= 0x01
#define PC_VCC_OFF		PORTG &= 0xfe


//cv
#define WD_TIM_SET      5       // 5sec when KEY WD, 0.1sec when I2C WD

#define TIM_SDWR_PERIOD	1		// min

#define SENSOR_OFFSET   134

#define MASTER			0
#define SLAVE			1

#define KEY_WD			0
#define I2C_WD			1

#define CONNECTED		0xAA	//0b10101010
#define CONNECTED_OK	0xAB	//0b10101011

#define SLAVE_READY     0xAC
#define SLAVE_CH_ASK	0xAD
#define	SLAVE_CH_SEND   0xAE
#define VALVE_START     0xAF
#define SLAVE_SLEEP		0xBA





