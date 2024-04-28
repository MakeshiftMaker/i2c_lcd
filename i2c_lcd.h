#ifndef I2C_LCD_H
#define I2C_LCD_H

extern int backlight_state;

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
void switchBacklight(int fd, int power);


// Add declarations for your other functions here

#endif // I2C_LCD_H