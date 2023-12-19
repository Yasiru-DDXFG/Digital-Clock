#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../include/i2cmaster.h"

#define HC595_PORT PORTD
#define HC595_DDR DDRD
#define HC595_DS_POS PD0
#define HC595_SH_CP_POS PD1
#define HC595_ST_CP_POS PD5
#define HC595DataHigh() (HC595_PORT |= (1 << HC595_DS_POS))
#define HC595DataLow() (HC595_PORT &= (~(1 << HC595_DS_POS)))

#define LEDpin PD7
#define LEDport PORTD
#define LED_DDR DDRD
#define SW1_PIN PINC
#define SW2_PIN PINC
#define SW1_NUM 0
#define SW2_NUM 1

#define idle1 0
#define settingsmode 1
#define timeout 2

unsigned char sec10 = 5;
unsigned char sec1 = 3;
unsigned char min10 = 5;
unsigned char min1 = 2;
unsigned char hrs10 = 1;
unsigned char hrs1 = 0;
unsigned char minutes = 0;
unsigned char seconds = 0;
unsigned char hours = 0;

volatile uint8_t currentstate = idle1;
volatile uint8_t segState = 0;
volatile uint8_t cnter = 0;
volatile uint8_t segNUM = 2;
volatile uint16_t timeoutctr = 0;
uint8_t but1pressed = 0;
uint8_t but2pressed = 0;

volatile uint8_t charLUT[11] = {0b00111111,
                                0b00000110,
                                0b01011011,
                                0b01001111,
                                0b01100110,
                                0b01101101,
                                0b01111101,
                                0b00000111,
                                0b01111111,
                                0b01101111,
                                0b00000000}; // last one for blank char

void checkButtons()
{
    static uint8_t but1last = 0;
    static uint8_t but2last = 0;
    uint8_t but1state = SW1_PIN & (1 << SW1_NUM);
    uint8_t but2state = SW2_PIN & (1 << SW2_NUM);
    if (but1state > 0 && but1last == 0)
    {
        but1pressed = 1;
    }
    if (but2state > 0 && but2last == 0)
    {
        but2pressed = 1;
    }
    but1last = but1state;
    but2last = but2state;
}

void HC595Init()
{
    HC595_DDR |= ((1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS));
}

void HC595Pulse()
{
    HC595_PORT |= (1 << HC595_SH_CP_POS);

    HC595_PORT &= (~(1 << HC595_SH_CP_POS));
}

void HC595Latch()
{
    HC595_PORT |= (1 << HC595_ST_CP_POS);
    _delay_loop_1(1);

    HC595_PORT &= (~(1 << HC595_ST_CP_POS));
    _delay_loop_1(1);
}

void HC595Write(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data1 & 0b10000000)
        {
            HC595DataHigh();
        }
        else
        {
            HC595DataLow();
        }

        HC595Pulse();
        data1 = data1 << 1;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data2 & 0b10000000)
        {
            HC595DataHigh();
        }
        else
        {
            HC595DataLow();
        }

        HC595Pulse();
        data2 = data2 << 1;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data3 & 0b10000000)
        {
            HC595DataHigh();
        }
        else
        {
            HC595DataLow();
        }

        HC595Pulse();
        data3 = data3 << 1;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data4 & 0b10000000)
        {
            HC595DataHigh();
        }
        else
        {
            HC595DataLow();
        }

        HC595Pulse();
        data4 = data4 << 1;
    }

    HC595Latch();
}

void writeTIME()
{
    uint8_t i2cbuff[4];
    i2c_read_bytes(i2cbuff, sizeof(i2cbuff), DS3231_addr);
    i2cbuff[0] = sec1 | (sec10 << 4);
    i2cbuff[1] = min1 | (min10 << 4);
    i2cbuff[2] = hrs1 | (hrs10 << 4) | (1 << 6);
    i2c_write_bytes(i2cbuff, sizeof(i2cbuff), DS3231_addr);
}

void readTime()
{
    uint8_t i2cbuff[4];
    i2c_read_bytes(i2cbuff, sizeof(i2cbuff), DS3231_addr);
    sec1 = i2cbuff[0] & 0x0F;
    sec10 = (i2cbuff[0] >> 4) & 0x0F;
    min1 = i2cbuff[1] & 0x0F;
    min10 = (i2cbuff[1] >> 4) & 0x0F;
    hrs1 = i2cbuff[2] & 0x0F;
    hrs10 = (i2cbuff[2] >> 4) & 0x01;
}

void enable_sqwave()
{
    uint8_t i2cbuff[18];
    i2c_read_bytes(i2cbuff, sizeof(i2cbuff), DS3231_addr);
    i2cbuff[0x0E] = 0x0;
    i2c_write_bytes(i2cbuff, sizeof(i2cbuff), DS3231_addr);
}

void Initialize_Clock()
{
    i2c_init();
    cli();
    LED_DDR |= (1 << LEDpin);

    /// Initialize shift register port
    HC595Init();

    /// Setup external interrupt 0. DS3231 interrupts uC every 1s to update time.
    EICRA |= 0x0F;
    EIMSK |= 0x01;
    enable_sqwave();

    /// set tim0 for 256ms interrupt to drive LED seven segment digits.
    TCCR0B |= 1 << CS00 | 1 << CS02;
    TIMSK0 |= 1 << TOIE0;
    readTime();
    sei(); /// Enable global interrupt
}

void calDigits()
{
    min1 = minutes % 10;
    min10 = minutes / 10;
    hrs1 = hours % 10;
    hrs10 = hours / 10;
}

void update()
{
    while (1)
    {
        checkButtons();

        if (currentstate == idle1)
        {
            
            if (but1pressed)
            {
                currentstate = settingsmode;
                but1pressed = 0;
                continue;
            }
        }
        if (currentstate == settingsmode)
        {
            if (but1pressed)
            {
                timeoutctr = 0;
                segNUM++;
                if (segNUM > 1){
                    segNUM = 0;
                }
                but1pressed = 0;
                continue;
            }
            if (but2pressed)
            {
                timeoutctr = 0;
                if (segNUM == 0)
                {
                    hours++;
                    if (hours > 12)
                        hours = 0;
                    calDigits();
                }
                if (segNUM == 1)
                {
                    minutes++;
                    if (minutes > 59)
                        minutes = 0;
                    calDigits();
                }
                but2pressed = 0;
                continue;
            }
        }
        if (currentstate == timeout)
        {
            writeTIME();
            currentstate = idle1;
        }
    }
}

ISR(INT0_vect)
{
    if (currentstate == idle1)
    {
        readTime();
        HC595Write(charLUT[min1], charLUT[min10], charLUT[hrs1], charLUT[hrs10]);
        LEDport ^= (1 << LEDpin);
    }
}

ISR(TIMER0_OVF_vect)
{
    if (currentstate == settingsmode)
    {
        timeoutctr++;
        if (cnter == 0)
        {
            HC595Write(charLUT[min1], charLUT[min10], charLUT[hrs1], charLUT[hrs10]);
        }
        if (cnter == 1)
        {
            if (segNUM == 1)
                HC595Write(charLUT[10], charLUT[10], charLUT[hrs1], charLUT[hrs10]);
            if (segNUM == 0)
                HC595Write(charLUT[min1], charLUT[min10], charLUT[10], charLUT[10]);
        }
        cnter++;
        if (cnter == 2)
        {
            cnter = 0;
        }
        if (timeoutctr > 200)
        {
            currentstate = timeout;
            timeoutctr = 0;
        }
    }
}
