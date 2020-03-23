/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#pragma config WDTE = OFF /* WDT operating mode->WDT Disabled */
#pragma config LVP = ON   /* Low-voltage programming enabled, RE3 pin is MCLR */

#define _XTAL_FREQ                  64000000UL
#include <pic18.h>
#include <xc.h>
#include <stdint.h>

#define PPS_CONFIG_RB1_I2C_SCL_IN       0x09
#define PPS_CONFIG_RB2_I2C_SDA_IN       0x0A
#define PPS_CONFIG_RB1_I2C_SCL_OUT      0x0F
#define PPS_CONFIG_RB1_I2C_SDA_OUT      0x10
#define BAUD_RATE_DIVIDER               0x9F
#define I2C_SLAVE_ADDR                  0x20
#define MCP23008_REG_ADDR_IODIR         0x00
#define MCP23008_REG_ADDR_GPIO          0x09
#define I2C_RW_BIT                      0x01
#define PINS_DIGITAL_OUTPUT             0x00
#define PINS_DIGITAL_LOW                0x00
#define PINS_DIGITAL_HIGH               0xFF

static void CLK_init(void);
static void PPS_init(void);
static void PORT_init(void);

static void I2C1_init(void);
static void I2C1_open(void);
static void I2C1_close(void);
static void I2C1_start(void);
static void I2C1_stop(void);
static void I2C1_sendData(uint8_t data);
static void I2C1_interruptFlagPolling(void);
static uint8_t I2C1_getAckstatBit(void);
static void I2C1_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t data);

static void CLK_init(void)
{
    /* Set Oscilator Source: HFINTOSC and Set Clock Divider: 1 */
    OSCCON1 = _OSCCON1_NOSC1_MASK | _OSCCON1_NOSC2_MASK;

    /* Set Nominal Freq: 64 MHz */
    OSCFRQ = _OSCFRQ_FRQ3_MASK;
}

static void PPS_init(void)
{
    /* PPS setting for using RB1 as SCL */
    SSP1CLKPPS = PPS_CONFIG_RB1_I2C_SCL_IN;
    RB1PPS = PPS_CONFIG_RB1_I2C_SCL_OUT;

    /* PPS setting for using RB2 as SDA */
    SSP1DATPPS = PPS_CONFIG_RB2_I2C_SDA_IN;
    RB2PPS = PPS_CONFIG_RB1_I2C_SDA_OUT;
}

static void PORT_init(void)
{
    /* Set pins RB1 and RB2 as Digital */
    ANSELB &= ~(_ANSELB_ANSELB1_MASK | _ANSELB_ANSELB2_MASK);
    
    /* Set pull-up resistorsfor RB1 and RB2 */
    WPUB |= _WPUB_WPUB1_MASK | _WPUB_WPUB2_MASK;
}

static void I2C1_init(void)
{
    /* I2C Master Mode: Clock = F_OSC / (4 * (SSP1ADD + 1)) */
    SSP1CON1 |= _SSP1CON1_SSPM3_MASK;
    
    /* Set the boud rate devider */
    SSP1ADD  = BAUD_RATE_DIVIDER;
}

static void I2C1_interruptFlagPolling(void)
{
    /* Polling Interrupt Flag */
    while (!(PIR3 & _PIR3_SSP1IF_MASK))
    {
        ;
    }

    /* Clear Interrupt Flag */
    PIR3 &= ~_PIR3_SSP1IF_MASK;
}

static void I2C1_open(void)
{
    /* Clear IRQ */
    PIR3 &= ~_PIR3_SSP1IF_MASK;

    /* I2C Master Open */
    SSP1CON1 |= _SSP1CON1_SSPEN_MASK;
}

static void I2C1_close(void)
{
    /* Disable I2C1 */
    SSP1CON1 &= ~_SSP1CON1_SSPEN_MASK;
}

static void I2C1_start(void)
{
    /* I2C Master Start */
    SSP1CON2 |= _SSP1CON2_SEN_MASK;
    
    I2C1_interruptFlagPolling();
}

static void I2C1_stop(void)
{
    /* STOP Condition */
    SSP1CON2 |= _SSP1CON2_PEN_MASK;
    
    I2C1_interruptFlagPolling();
}

static void I2C1_sendData(uint8_t byte)
{
    SSP1BUF  = byte;
    I2C1_interruptFlagPolling();
}

static uint8_t I2C1_getAckstatBit(void)
{
    /* Return ACKSTAT bit */
    if ((SSP1CON2 & _SSP1CON2_ACKSTAT_MASK))
    {
        /* Acknowledge not received from slave */
        return 1;
    }
    
    return 0;
}

static void I2C1_write1ByteRegister(uint8_t address, uint8_t reg, uint8_t data)
{
    /* Shift the 7 bit address and add a 0 bit to indicate write operation */
    uint8_t writeAddress = (address << 1) & ~I2C_RW_BIT;
    
    I2C1_open();
    I2C1_start();
    
    I2C1_sendData(writeAddress);
    if (I2C1_getAckstatBit())
    {
        return ;
    }
    
    I2C1_sendData(reg);
    if (I2C1_getAckstatBit())
    {
        return ;
    }
    
    I2C1_sendData(data);
    if (I2C1_getAckstatBit())
    {
        return ;
    }
    
    I2C1_stop();
    I2C1_close();
}

void main(void)
{
    CLK_init();
    PPS_init();
    PORT_init();
    I2C1_init();
    
    /* Set the extended pins as digital output */
    I2C1_write1ByteRegister(I2C_SLAVE_ADDR, MCP23008_REG_ADDR_IODIR, PINS_DIGITAL_OUTPUT);
    
    while (1)
    {
        /* Set the extended pins to digital low */
        I2C1_write1ByteRegister(I2C_SLAVE_ADDR, MCP23008_REG_ADDR_GPIO, PINS_DIGITAL_LOW);
        __delay_ms(500);
        /* Set the extended pins to digital high */
        I2C1_write1ByteRegister(I2C_SLAVE_ADDR, MCP23008_REG_ADDR_GPIO, PINS_DIGITAL_HIGH);
        __delay_ms(500);
	}
}
