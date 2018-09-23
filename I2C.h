#ifndef I2C_H
#define	I2C_H

#ifndef I2C_BAUDRATE
#define I2C_BAUDRATE (unsigned long)400000 //default baud rate 400kHz
#endif

#ifndef I2C_BAUDRATE_CODEC
#define I2C_BAUDRATE_CODEC (unsigned long)100000 //default baud rate 100kHz
#endif

#define I2C_ERROR -1
#define I2C_OK 1

void I2C_Init(void);
int setBaudRate(int codec);
int I2C_WriteReg(char dev_addr, char reg_addr, char value);
int I2C_Write_Reg(char dev_addr, char reg_addr, char value);
int I2C_ReadReg(char dev_addr, char reg_addr, char *value);
int I2C_ReadPIC(char data_addr, char *value);

#endif	/* I2C_H */

