
/*
*
* by Lewis Loflin www.bristolwatch.com lewis@bvu.net
* http://www.bristolwatch.com/rpi/i2clcd.htm
* Using wiringPi by Gordon Henderson
*
*
* Port over lcd_i2c.py to C and added improvements.
* Supports 16x2 and 20x4 screens.
* This was to learn now the I2C lcd displays operate.
* There is no warrenty of any kind use at your own risk.
*
*/

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <stdio.h>

// Define some device parameters
// #define I2C_ADDR   0x27 // I2C device address

// Define some device constants
// #define LCD_CHR  1 // Mode - Sending data
// #define LCD_CMD  0 // Mode - Sending command

// #define LINE1  0x80 // 1st line
// #define LINE2  0xC0 // 2nd line

// #define LCD_BACKLIGHT 0x08  // On
// #define LCD_BACKLIGHT 0x00  // Off

// #define ENABLE  0b00000100 // Enable bit

void lcd_init(int fd);
void lcd_byte(int fd, int bits, int mode);
void lcd_toggle_enable(int fd, int bits);

// added by Lewis
void typeInt(int fd, int i);
void typeFloat(int fd, float myFloat);
void lcdLoc(int fd, int line); //move cursor
void ClrLcd(int fd); // clr LCD return home
void typeln(int fd, const char *s);
void typeChar(int fd, char val);

//added by Maker
void switchBacklight(int fd, int bool);

// float to string
void typeFloat(int fd, float myFloat)   {
  char buffer[20];
  sprintf(buffer, "%4.2f",  myFloat);
  typeln(fd, buffer);
}

// int to string
void typeInt(int fd, int i)   {
  char array1[20];
  sprintf(array1, "%d",  i);
  typeln(fd, array1);
}

// clr lcd go home loc 0x80
void ClrLcd(int fd)   {
  lcd_byte(fd, 0x01, 0);
  lcd_byte(fd, 0x02, 0);
}

// go to location on LCD
void lcdLoc(int fd, int line)   {
  lcd_byte(fd, line, 0);
}

// out char to LCD at current position
void typeChar(int fd, char val)   {
  lcd_byte(fd, val, 1);
}


// this allows use of any size string
void typeln(int fd, const char *s)   {

  while ( *s ) lcd_byte(fd, *(s++), 1);

}

void lcd_byte(int fd, int bits, int mode)   {
  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = mode | (bits & 0xF0);
  bits_low = mode | ((bits << 4) & 0xF0);

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(fd, bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(fd, bits_low);
}

void switchBacklight(int fd, int power){
  int backlight;
  if(power == 1){
    backlight = 0x08;
  }else{
    backlight = 0x00;
  }
  //Send byte to data pins
  // bits = the data
  // mode = 1 for data, 0 for command
  int bits_high;
  int bits_low;
  // uses the two half byte writes to LCD
  bits_high = 0 | backlight ;
  bits_low = 0 | backlight ;

  // High bits
  wiringPiI2CReadReg8(fd, bits_high);
  lcd_toggle_enable(fd, bits_high);

  // Low bits
  wiringPiI2CReadReg8(fd, bits_low);
  lcd_toggle_enable(fd, bits_low);
}

void lcd_toggle_enable(int fd, int bits)   {
  // Toggle enable pin on LCD display
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits | 0b00000100));
  delayMicroseconds(500);
  wiringPiI2CReadReg8(fd, (bits & ~0b00000100));
  delayMicroseconds(500);
}


void lcd_init(int fd)   {
  // Initialise display
  lcd_byte(fd, 0x33, 0); // Initialise
  lcd_byte(fd, 0x32, 0); // Initialise
  lcd_byte(fd, 0x06, 0); // Cursor move direction
  lcd_byte(fd, 0x0C, 0); // 0x0F On, Blink Off
  lcd_byte(fd, 0x28, 0); // Data length, number of lines, font size
  lcd_byte(fd, 0x01, 0); // Clear display
  delayMicroseconds(500);
}
