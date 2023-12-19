#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void writeTIME();
void readTime();
void checkButtons();
void update();
void enable_sqwave();
void Initialize_Clock();
void HC595Write(uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4);
void HC595Latch();
void HC595Pulse();
void HC595Init();

int main(void)
{

    Initialize_Clock();

    /* Main loop
       Check if button 1 is pressed. If so enter settings mode.
       In settings mode, check for button presses to select or increment digits. 
       Digits are selected incrementally and rolls back to first digit.
       Value of digits are incremented from 0 to 9 and rolls back to 0
       Enter idle mode if no buttons are pressed within timeout.*/

    while(1){
        update();
    }
    return 0;
}
