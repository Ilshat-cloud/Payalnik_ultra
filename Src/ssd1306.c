
#include "ssd1306.h"

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
/* Private variable */
static SSD1306_t SSD1306;

static void ssd1306_WriteCommand(uint8_t command)
{
  HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x00,1,&command,1,10);
}


uint8_t dota[128]={
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x80,
0x00,0x00,0x1F,0xC0,
0x00,0x00,0x3F,0xC0,
0x00,0x00,0x3F,0xC0,
0x00,0x00,0x3F,0xE0,
0x00,0x00,0x7F,0xE0,
0x00,0x00,0x7F,0x70,
0x00,0x00,0x7E,0x20,
0x00,0x00,0x7F,0x00,
0x00,0x00,0x7F,0x00,
0x00,0x00,0xFF,0x00,
0x00,0x01,0xFF,0x00,
0x00,0x03,0xFF,0xB0,
0x00,0x3f,0xFF,0xF0,
0x01,0xff,0xFF,0xF0,
0x03,0xff,0xFF,0xF0,
0x03,0xff,0xFD,0x90,
0x07,0xff,0xF9,0xB0,
0x07,0xff,0xF1,0xA0,
0x07,0xff,0x80,0x20,
0x07,0xff,0x00,0x00,
0x06,0xEE,0x00,0x00,
0x00,0xCC,0x00,0x00,
0x00,0xD8,0x00,0x00,
0x00,0xFC,0x80,0x00,
0x00,0x77,0xC0,0x00,
0x00,0x19,0xC0,0x00,
0x00,0x18,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00};

uint8_t ssd1306_Init(void) {
        
  
        HAL_Delay(100);
	
	/* Init LCD */
	 ssd1306_WriteCommand(0xAE); //display off
	 ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode   
	 ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	 ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	 ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
	 ssd1306_WriteCommand(0x00); //---set low column address
	 ssd1306_WriteCommand(0x10); //---set high column address
	 ssd1306_WriteCommand(0x40); //--set start line address
	 ssd1306_WriteCommand(0x81); //--set contrast control register
	 ssd1306_WriteCommand(0xFF);
	 ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
	 ssd1306_WriteCommand(0xA6); //--set normal display
	 ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
	 ssd1306_WriteCommand(0x3F); //
	 ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	 ssd1306_WriteCommand(0xD3); //-set display offset
	 ssd1306_WriteCommand(0x00); //-not offset
	 ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	 ssd1306_WriteCommand(0xF0); //--set divide ratio
	 ssd1306_WriteCommand(0xD9); //--set pre-charge period
	 ssd1306_WriteCommand(0x22); //
	 ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
	 ssd1306_WriteCommand(0x12);
	 ssd1306_WriteCommand(0xDB); //--set vcomh
	 ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
	 ssd1306_WriteCommand(0x8D); //--set DC-DC enable
	 ssd1306_WriteCommand(0x14); //
	 ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel
	
	/* Clear screen */
	 ssd1306_Fill(Black);
	
	/* Update screen */
	ssd1306_UpdateScreen();
	
	/* Set default values */
	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	
	/* Initialized OK */
	SSD1306.Initialized = 1;
	
	/* Return OK */
	return 1;
}


void ssd1306_Fill(SSD1306_COLOR color) 
{
	/* Set memory */
        uint32_t i;
	for (i=0; i<sizeof(SSD1306_Buffer);i++)
        {
          SSD1306_Buffer[i]=(color==Black)?0x00:0xFF;
        }
}


void ssd1306_UpdateScreen(void) 
{
        uint8_t i;
        for (i=0; i<8; i++)
        {
          ssd1306_WriteCommand(0xB0+i);
          ssd1306_WriteCommand(0x00);
          ssd1306_WriteCommand(0x00);
          
          HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x40,1,&SSD1306_Buffer[SSD1306_WIDTH *i],SSD1306_WIDTH,10);
        }
}



void ssd1306_DrawPixel(uint8_t x, uint8_t y,SSD1306_COLOR color) {
	if (x >= SSD1306_WIDTH ||y >= SSD1306_HEIGHT) 
        {
		/* Error */
		return;
	}
	
	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (SSD1306_COLOR)!color;
	}
	
	/* Set color */
	if (color == White) 
        {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}



///////////***********************////////////////////

char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;
	
	// Check remaining space on current line
	if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}
	
	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	
	// The current space is now taken
	SSD1306.CurrentX += Font.FontWidth;
	
	// Return written char for validation
	return ch;
}

//------------------------startScreen-------------------
void startScreen() {
  
    uint8_t j;
    uint8_t i;
	for (i = 0; i < 128; i++)
	{       
                uint8_t i2=i/4;
		for (j = 0; j < 8; j++)
		{
			if ((dota[i])&(0x01<<(7-j)))
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j+(i%4)*8, (SSD1306.CurrentY + i2),White);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j+(i%4)*8, (SSD1306.CurrentY + i2), Black);
			}
                        
		}
	}
  ssd1306_UpdateScreen();
  HAL_Delay(200);
}
//
//  Write full string to screenbuffer
//
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
	// Write until null-byte
	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
		{
			// Char could not be written
			return *str;
		}
		
		// Next char
		str++;
	}
	
	// Everything ok
	return *str;
}

//
//	Position the cursor
//
void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

