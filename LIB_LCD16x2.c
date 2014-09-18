/*
Author    : prajin palangsantikul                     
Company   : appsofttech co.,ltd.                      
Filename  :	LIB_LCD16x2.c 
Purpose   : LCD Character 16x2 Module (4bit mode)
Ref..     :	TG_4bitLCD.c [Thorsten Godau, thorsten.godau@gmx.de]
Date      :	15/10/2005                                
*/
//----------------------------------------------------:LCD PIN
// GND -> LCD R/W
// RE0 -> LCD D4
// RE1 -> LCD D5
// RE2 -> LCD D6
// RE3 -> LCD D7
// RE4 -> LCD RS
// RE5 -> LCD E 

//----------------------------------------------------:LCD Command
//  Command |  Description
//-----------------------------------------------------
//  0x01    |  Clear display screen
//  0x02    |  Return Home (move cursor to top/left character position)
//  0x04    |  Decrement cursor (shift cursor to left)
//  0x05    |  Increment cursor (shift cursor to right)
//  0x06    |  shift display right
//  0x07    |  shift display left
//  0x08    |  Display off, cursor off
//  0x0A    |  Display off, cursor on
//  0x0C    |  Display on, cursor off
//  0x0E    |  Display on, cursor blinking
//  0x0F    |  Display on, cursor blinking
//  0x10    |  Shift cursor position to left
//  0x14    |  Shift cursor position to right
//  0x18    |  Shift the entire display to the left
//  0x1C    |  Shift the entire display to the right
//  0x80    |  Force cursor to the beginning of 1st line
//  0xC0    |  Force cursor to the beginning of 2nd line
//  0x28    |  Function set (4-bit interface, 2 lines, 5*7 Pixels)   
//  0x20    |  Function set (4-bit interface, 1 line, 5*7 Pixels)    


//----------------------------------------------------:Includes
//#include <p30fxxxx.h>           // generic header file for dsPIC


//----------------------------------------------------:Defines
#define LCD_TRIS    TRISF     // Set direction control output  
#define LCD_DATA    LATF      // Dataport of LCD-Display (D4..D7)
#define LCD_RS      _LATF4    // Register Select of LCD-Display
#define LCD_E       _LATF5    // Enable of LCD-Display
#define CTRL_REG    0         // Select instruction register
#define DATA_REG    1         // Select data register
#define BLINK       0x01      // Alias for blinking cursor
#define NOBLINK     0x00      // Alias for non blinking cursor
#define SHOW        0x02      // Alias for cursor on
#define HIDE        0x00      // Alias for cursor off
#define ON          0x04      // Alias for display on
#define OFF         0x00      // Alias for display off
                                                                                    
// Table to select DD-RAM address (including instruction and address)
// 0x00..0x0F -> Line 1, 0x40..0x4F -> Line 2
static unsigned char LOCATION[2][16] = { {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,
                                          0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F},
                                         {0xC0,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,
                                          0xC8,0xC9,0xCA,0xCB,0xCC,0xCD,0xCE,0xCF} 
                                        };

//----------------------------------------------------:Function Prototypes
static void delay(unsigned int ms);            // Timer dependend delay-routine
static void LCD_Command(unsigned char cmd);    // Command LCD

// Clear display
void LCDClrscr(void);
// Controls the display
void LCDControl(unsigned char dsp,unsigned char blink,unsigned char cursor);
// Writes a character to display
void LCDPutchar(unsigned char value);
// Prints a text to x/y position
void LCDPrintxy(unsigned char x,unsigned char y, unsigned int dly, unsigned char *text);
// Sets LCD write position
void LCDGotoxy(unsigned char x,unsigned char y);
// Initialize the LCD display
void LCDInit(void);

//----------------------------------------------------:delay
// Function : delay_ms
// Parameters : ms - unsigned int
// Returned : nothing 
static void delay(unsigned int ms)
{
    unsigned int i;

    for (; ms>0; ms--)
      for (i=0; i<128; i++)
      //for (i=0; i<728; i++)
        Nop();                  // delay 1 mch cycle
}

//----------------------------------------------------:LCD command
static void LCD_Command(unsigned char cmd)
{
	delay(20);              // Wait 20ms
	LCD_E  = 0;
	LCD_RS = CTRL_REG;      // Switch to inruction register

	// Set LCD_DATA to high nibble of Display On/Off Control
	LCD_DATA = (LCD_DATA&0xF0)|((cmd&0xF0)>>4);
	LCD_E = 1; LCD_E = 0;   // Write data to display

	// Set LCD_DATA to lower nibble of Display On/Off Control
	LCD_DATA = (LCD_DATA&0xF0)|(cmd&0x0F);
	LCD_E = 1; LCD_E = 0;   // Write data to display

	delay(1);  	            // Wait 1ms

	return;
}

//----------------------------------------------------:LCDControl
// Function    : LCDControl(dsp,blink,cursor)
// Input : unsigned char dsp    = ON     -> Display on
//                                OFF    -> Display off
//         unsigned char blink  = BLINK  -> Cursor blinks
//                                NOBLINK-> Cursor not blinks
//         unsigned char cursor = SHOW   -> Cursor visible
//                                HIDE   -> Cursor invisible
//
void LCDControl(unsigned char dsp,unsigned char blink,unsigned char cursor)
{
	unsigned char control;  // variable to generate instruction byte

	control = (0x08 + blink + cursor + dsp); // Cursor control
	LCD_Command(control);
	
	return;
}

//----------------------------------------------------:LCDClrscr
void LCDClrscr(void)
{
  LCD_Command(0x01);    // Clear screen
}  
  
//----------------------------------------------------:LCDGotoxy
// Function    : LCDGotoxy(x,y)
// Description : Sets the write position of the LCD display         
//                                                                  
//                 (1,1)         (16,1)                             
//                   |              |                               
//                   ################   -> line 1                   
//                   ################   -> line 2                   
//                   |              |                               
//                 (1,2)         (16,2)                             
// Input       : unsigned char x    -> x position (horizontal)      
//               unsigned char y    -> y position (vertical)        
//
void LCDGotoxy(unsigned char x,unsigned char y)
{
	delay(20);             	// Wait 20ms
	LCD_E  = 0;
	LCD_RS = CTRL_REG;      // Switch to inruction register

	// Set LCD_DATA to high nibble of position table value
	LCD_DATA = (LCD_DATA&0xF0)|(((LOCATION[y-1][x-1])&0xF0)>>4);
	LCD_E = 1; LCD_E = 0;       // Write data to display
	// Set LCD_DATA to lower nibble of position table value
	LCD_DATA = (LCD_DATA&0xF0)|((LOCATION[y-1][x-1])&0x0F);
	LCD_E = 1; LCD_E = 0;       // Write data to display

	delay(1);                   // Wait 1ms

	return;
}

//----------------------------------------------------:LCDPutchar
// Function    : LCDPutchar(value)                             
// Description : Writes the character value to the display
//
void LCDPutchar(unsigned char value)
{  
	LCD_RS = DATA_REG;              // Switch to data register

	// Set LCD_DATA to high nibble of value
	LCD_DATA = (LCD_DATA&0xF0)|((value&0xF0)>>4);
	LCD_E = 1; LCD_E = 0;           // Write data to display
	// Set LCD_DATA to lower nibble of value
	LCD_DATA = (LCD_DATA&0xF0)|(value&0x0F);
	LCD_E = 1; LCD_E = 0;           // Write data to display

	delay(1);                       // Wait 1ms

	return;
}

//----------------------------------------------------:LCDPrintxy
// Function    : LCDPrintxy(x,y,*text)                         
// Description : Prints text to position x/y of the display      
// Input       : unsigned char x     -> x position of the display 
//               unsigned char y     -> y position of the display  
//               unsigned char *text -> pointer to text             
//							 unsigned int dly	   -> delay time
//
void LCDPrintxy(unsigned char x,unsigned char y, unsigned int dly, unsigned char *text)
{

	LCDGotoxy(x,y);         // Set cursor position

	while( *text )          // while not end of text
  {
  	LCDPutchar(*text++);  // Write character and increment position
		delay(dly);           // time delay print text
  } 

	return;
}

//----------------------------------------------------:LCDInit
// Initialize the LCD display
void LCDInit(void)
{
    LCD_TRIS = 0x0000;      // set direction control RB output
    LCD_Command(0x33);      // set 4 bit mode
    LCD_Command(0x32);      // set 4 bit mode
    LCD_Command(0x28);      // 2 line 5x7 dot
    LCD_Command(0x01);      // clear screen lcd
    LCDControl(ON,NOBLINK,HIDE);           
}
