 
//----------------------------------------------------:Includes
#include <p30fxxxx.h>       // generic header file for dsPIC
#include <adc10.h>          // 10bit ADC module library functions
#include "uart.h"											// Used UART Config
#include "i2c.h"											// Used I2C Function Library

/* Setup Configuration For ET-BASE dsPIC30F4011 */
_FOSC(CSW_FSCM_OFF & XT_PLL16);								// Disable Clock Switching,Enable Fail-Salf Clock
                                                            // Clock Source = Primary XT + (PLL x 16)
_FWDT(WDT_OFF);												// Disable Watchdog 
_FBORPOR(PBOR_OFF & PWRT_64 & MCLR_EN);					    // Disable Brown-Out ,Power ON = 64mS,Enable MCLR
_FGS(CODE_PROT_OFF);										// Code Protect OFF
/* End Configuration For ET-BASE dsPIC30F4011 */

//----------------------------------------------------:Global variables
unsigned int ADC_DriveX,ADC_DriveY;			// Drive
unsigned int ADC_CamX,ADC_CamY;				// Camera
unsigned int ADC_JointX,ADC_JointY;			// Joint

unsigned char *wrptr;
unsigned char tx_data[] = {'M','I','C','R','O','C','H','I','P','\0'};


void init_i2c(void);										// Initial I2C 
void i2c_rx();

char buf[50];

int carbon;
char packet_tx[10];
char temp[8];
char rx_data;


int speed=0;
int speed1=0;
int speed2=0;

int joint_angle=0;
int joint1_angle=0;
int joint2_angle=0;



char packet[50];
float packet_use[50];
int per_duty;  // int per_duty
int dutycycle; 
int ll;

unsigned char ax12_buf[10];	  								// "servo Buffer"

unsigned char readRegis(unsigned char reg);
void convert_speed();
void convert_position();


int run_print;
	int lk;

void go_robotis(int id,int pos,int sp);						// Servo Driving Function


unsigned char temper[8];
char temp_uart[8];
int o=0;


int convert_param(char dat);

//----------------------------------------------------:U1RX ISR
// UART1 Receiver Interrupt Service Routine
void _ISR _U1RXInterrupt(void)
{
 	while(DataRdyUART1())
	{

		rx_data = ReadUART1();
		if(rx_data != '\n' && rx_data != ' ')
		{
			temp_uart[o] = rx_data;
			//sprintf(buf,"%c",temp_uart);
			//putsUART2(buf);
			o++;
		}
		else if(rx_data == '\n')
		{
			//sprintf(buf,"C %c%c%c%c%c%c\r\n",temp_uart[1],temp_uart[2],temp_uart[3],temp_uart[4],temp_uart[5],temp_uart[6],temp_uart[7]);
			//putsUART2(buf);
			o = 0;
			//sprintf(buf,"T %d %d %d %d %d %d %d %d ",temper[0],temper[1],temper[2],temper[3],temper[4],temper[5],temper[6],temper[7]);
			//	putsUART2(buf);
		}
	} 
 
	_U1RXIF = 0;    // Clear interrupt flag RX
}


int convert_param(char dat)
{
	int c;
	int temp;		
	int pack_param;
		   switch(dat){
		   		case '0': pack_param = 0.0; break;
		   		case '1': pack_param = 1.0; break;
		   		case '2': pack_param = 2.0; break;
		   		case '3': pack_param = 3.0; break;
		   		case '4': pack_param = 4.0; break;
		   		case '5': pack_param = 5.0; break;
		   		case '6': pack_param = 6.0; break;
		   		case '7': pack_param = 7.0; break;
		   		case '8': pack_param = 8.0; break;
		   		case '9': pack_param = 9.0; break;
		   		default : pack_param = 0.0;
		   }

	return pack_param;
}


unsigned char read_tpa81(unsigned char Read_Addr, unsigned char Reg)
{
		unsigned char Get_Temp;									// Time Buffer

  		StartI2C();												// Send Start Condition
  		while(I2CCONbits.SEN);									// Wait Start Complete
    
  		// Write DS1307 ID Code = 1101000+W
  		MasterWriteI2C(Read_Addr);										// Write DS1307 ID Code,Write
 	 	while(I2CSTATbits.TBF);									// Wait Write ID Code Complete
  		//while(I2CSTATbits.ACKSTAT);								// Wait Acknowledge Complete
  		IdleI2C();												// Wait Status Clear  

  		// Write Address of RTC:ds1307 For Read
  		MasterWriteI2C(Reg);								// Write RTC Address 
  		while(I2CSTATbits.TBF);									// Wait Write Address Complete
  		//while(I2CSTATbits.ACKSTAT);								// Wait Acknowledge Complete
  		IdleI2C();												// Wait Status Clear  
       
  		// Restart For Read DS1307 Data
  		RestartI2C();												// Send Re-Start Condition
  		while(I2CCONbits.RSEN);									// Wait Re-Start Complete
     
  		// Write DS1307 ID Code = 1101000+R 
  		MasterWriteI2C(Read_Addr|0x01);										// Write DS1307 ID Code,Read
  		while(I2CSTATbits.TBF);									// Wait Write ID Code Complete
  		//while(I2CSTATbits.ACKSTAT);								// Wait Acknowledge Complete
  		IdleI2C();												// Wait Status Clear  
    
  		Get_Temp = MasterReadI2C();								// Read Time From RTC
  		while(I2CSTATbits.RBF);									// Wait Read Data Complete
  		//while(I2CSTATbits.ACKSTAT);								// Wait Acknowledge Complete
  		IdleI2C();												// Wait Status Clear 
        
  		StopI2C();												// Send Stop Condition
  		while(I2CCONbits.PEN);									// Wait Stop Complete    

  		return Get_Temp;

}


//----------------------------------------------------:delay_ms
// Delay 1 ms (XT w/PLL 1x)
void delay_ms(unsigned int ms)
{
  unsigned int i;

  for (; ms>0; ms--)
  for (i=0; i<182; i++)
    Nop();                  // delay 1 mch cycle
}



void init_uart();
void adc_init();


//----------------------------------------------------:Main
int main(void)
{
  unsigned int count, *adcptr;

   init_i2c();												// Initial I2C Function
   init_uart();
 
   TRISBbits.TRISB0 = 0;	
   LATBbits.LATB0 = 1;	


while(1)
{
	int i;
	for(i=2;i<10;i++)
	{
		temper[i-2] = read_tpa81(0XD0,i);
	}

	int temp_avg = (temper[0]+temper[1]+temper[2]+temper[3]+temper[4]+temper[5]+temper[6]+temper[7])/8;

			int co2_0 = convert_param(temp_uart[1]);						
			int co2_1 = convert_param(temp_uart[2]);
			int co2_2 = convert_param(temp_uart[3]);
			int co2_3 = convert_param(temp_uart[4]);
			int co2_4 = convert_param(temp_uart[5]);

			int co2_use = (co2_0*10000)+(co2_1*1000)+(co2_2*100)+(co2_3*10)+co2_4;
			if(co2_use > 0)
			{
				//sprintf(buf,"%d %d %d %d %d %d %d %d %d\r\n",temper[0],temper[1],temper[2],temper[3],temper[4],temper[5],temper[6],temper[7],co2_use);
				sprintf(buf,"%d %d\r\n",temp_avg,co2_use);
				putsUART2(buf);
	
			}
			else
			{
				sprintf(buf,"CO2 Error!!!\r\n");
				putsUART2(buf);
			}
				delay_ms(1000);
}
		
	return 1;
} 

/*********************************/
/* Initial UART for dsPIC30F4011 */
/* 9600,N,8,1 / 117.9648MHz Fosc */
/*********************************/
void init_uart()
{		  
  CloseUART1();												// Disable UART1 Before New Config
  CloseUART2();												// Disable UART1 Before New Config

  // Config UART1 Interrupt Control
  ConfigIntUART1(UART_RX_INT_EN &							// ENABLE RX Interrupt
    		     UART_RX_INT_PR2 &							// RX Interrupt Priority = 2
    		     UART_TX_INT_DIS &
    		     UART_TX_INT_PR3 );
  
   // Config UART1 Interrupt ontrol
  ConfigIntUART2(UART_RX_INT_DIS &							// ENABLE RX Interrupt
    		     UART_RX_INT_PR2 &							// RX Interrupt Priority = 2
    		     UART_TX_INT_DIS &
    		     UART_TX_INT_PR3 );
    		     
    		    
    		     
  // Open UART1 = Mode,Status,Baudrate              
  OpenUART1(UART_EN	&										// Enable UART(UART Mode)
            UART_IDLE_STOP &								// Disable UART in IDLE Mode 
 			UART_ALTRX_ALTTX & 								// Select U1TX=RC13,U1RX=RC14
            UART_DIS_WAKE &									// Disable Wake-Up
			UART_DIS_LOOPBACK &								// Disable Loop Back
			UART_DIS_ABAUD &								// Disable Auto Baudrate
  			UART_NO_PAR_8BIT &								// UART = 8 Bit, No Parity
 			UART_1STOPBIT,									// UART = 1 Stop Bit

	  		// Config UART1 Status
  			UART_INT_TX & 									// Select Interrupt After TX Complete
	 		UART_TX_PIN_NORMAL &							// Normal U1TX Mode
 			UART_TX_ENABLE &								// Enable U1TX
 	 		UART_INT_RX_CHAR &							// Flasg Set After RX Complete 
  			UART_ADR_DETECT_DIS &              				// Disable Check Address 
			UART_RX_OVERRUN_CLEAR,							// Clear Overrun Flag

  			// ET-BASE dsPIC30F4011 Hardware Board
  			// XTAL = 7.3728MHz
  			// Fosc = 7.3728 MHz x 16 = 117.9648 MHz
  			// Fcy(UART) = Fosc / 4 
  			//           = 117.9648 / 4 = 29.4912 MHz
  			// U1BRG = [Fcy/(16xBaud)]-1
  			//       = [29.4912 MHz / (16x9600)] - 1
  			//       = 191 = BFH			
  			191);											// ET-BASE dsPIC30F4011 UART Baudrate = 9600 BPS = 191 //  Buad Rate 19200 = 95  ,, Buadrate 38400 = 47 ,  Buadrate 115200 = 15
  			


  			// Open UART1 = Mode,Status,Baudrate              
  OpenUART2(UART_EN	&										// Enable UART(UART Mode)
            UART_IDLE_STOP &								// Disable UART in IDLE Mode 
 			UART_ALTRX_ALTTX & 								// Select U1TX=RC13,U1RX=RC14
            UART_DIS_WAKE &									// Disable Wake-Up
			UART_DIS_LOOPBACK &								// Disable Loop Back
			UART_DIS_ABAUD &								// Disable Auto Baudrate
  			UART_NO_PAR_8BIT &								// UART = 8 Bit, No Parity
 			UART_1STOPBIT,									// UART = 1 Stop Bit

	  		// Config UART1 Status
  			UART_INT_TX & 									// Select Interrupt After TX Complete
	 		UART_TX_PIN_NORMAL &							// Normal U1TX Mode
 			UART_TX_ENABLE &								// Enable U1TX
 	 		UART_INT_RX_CHAR &							// Flasg Set After RX Complete 
  			UART_ADR_DETECT_DIS &              				// Disable Check Address 
			UART_RX_OVERRUN_CLEAR,							// Clear Overrun Flag

  			// ET-BASE dsPIC30F4011 Hardware Board
  			// XTAL = 7.3728MHz
  			// Fosc = 7.3728 MHz x 16 = 117.9648 MHz
  			// Fcy(UART) = Fosc / 4 
  			//           = 117.9648 / 4 = 29.4912 MHz
  			// U1BRG = [Fcy/(16xBaud)]-1
  			//       = [29.4912 MHz / (16x9600)] - 1
  			//       = 191 = BFH			
  			15);											// ET-BASE dsPIC30F4011 UART Baudrate = 9600 BPS = 191 //  Buad Rate 19200 = 95  ,, Buadrate 38400 = 47  , Buadrate 115200 = 15
}


/*************************************/
/* Initial dsPIC30F4011 I2C Function */
/* For Interface DS1307 (I2C RTC)    */
/*************************************/
void init_i2c(void)
{  
  CloseI2C();												// Close I2C Before New Config  

  // Open I2C Function For Interface Ds1307
  OpenI2C(I2C_ON &											// Enable I2C Function
		  I2C_IDLE_STOP &									// Disable I2C in IDLE Mode
		  I2C_CLK_HLD &										// I2C Clock Hold
		  I2C_IPMI_DIS &     								// Disable I2C IPMI Mode Control
		  I2C_7BIT_ADD &									// I2C Device Address = 7 Bit
		  I2C_SLW_DIS &										// Disable I2C Slew Rate Control
		  I2C_SM_DIS &										// Disable I2C SMBUS Mode
		  I2C_GCALL_DIS &									// Disable I2C General Call(Slave) 
          I2C_STR_DIS &										// Disable SCL Clock Stretch
          I2C_ACK &											// ACK Cycle = ACK
	      I2C_ACK_DIS &										// Disable I2C Acknowledge
		  I2C_RCV_DIS &										// Disable I2C Receive
		  I2C_STOP_DIS &									// Disable I2C Stop		
		  I2C_RESTART_DIS &									// Disable I2C Restart
		  I2C_START_DIS,									// Disable I2C Start		

          // ET-BASE dsPIC30F4011 Hardware Board
		  // XTAL = 7.3728MHz
  		  // Fosc = 7.3728 MHz x 16 = 117.9648 MHz
  		  // Fcy(I2C) = Fosc / 4 
  		  //           = 117.9648 / 4 = 29.4912 MHz          
  		  // I2CBRG = [(Fcy/Fscl)-(Fcy/1,111,111)]-1
  		  //       = [(29.4912 MHz / 100KHz)-(29.4912MHz / 1,111,111)] - 1
          //       = [(294.912)-(26.542)]-1
          //       = 268.37 - 1
  		  //       = 267 = 10BH
		  267);												// I2C Baudrate(Approx. = 100 KHz)

  // Initial I2C Interrupt Control
  ConfigIntI2C(MI2C_INT_OFF &								// Disabe Master I2C Interrupt
               SI2C_INT_OFF &								// Disabe Slave I2C Interrupt
               MI2C_INT_PRI_7 &								// Set Priority Interrupt of Master = 7 
			   SI2C_INT_PRI_7 );							// Set Priority Interrupt of Slave = 7                 
}


// Reads a gyro register
unsigned char readRegis(unsigned char reg)
{
	  unsigned char value;									// Time Buffer 
	  StartI2C();												// Send Start Condition
	  while(I2CCONbits.SEN);									// Wait Start Complete
	  MasterWriteI2C(0xD0+0);								// Write L3G4200D ID Code (1101000+W)
	  while(I2CSTATbits.TBF);									// Wait Write ID Code Complete
	  IdleI2C();
	  RestartI2C();												// Send Re-Start Condition
	  while(I2CCONbits.RSEN);									// Wait Re-Start Complete
	  MasterWriteI2C(0x03);										// Write L3G4200D register
	  while(I2CSTATbits.TBF);									// Wait Write Address Complete
	  IdleI2C();
	  MasterWriteI2C(0xD0+1);										// Write L3G4200D register
	  while(I2CSTATbits.TBF);									// Wait Write Address Complete
	  IdleI2C();
	  StopI2C();												// Send Stop Condition
	  while(I2CCONbits.PEN);									// Wait Stop Complete  
	  
	  return value;												// Return Time Result  
} 

