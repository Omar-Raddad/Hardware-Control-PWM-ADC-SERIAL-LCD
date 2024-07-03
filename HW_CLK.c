/*
 * File:   pwm_asl.c
 * PWM + ADC + SERIAL + LCD
 * LCD is set to work on the simulator, must be fixed to work with real
 */


#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>

#include <string.h>
//#include <pic18f4620.h>
#include "my_s.h"
#include "my_adc.h"
#include "lcd_x8.h"

#define STARTVALUE  3036

#define SELSIZE  3
typedef enum{
    INC_SEC,    
  //  INC_10SEC,
    INC_MIN,
 //   INC_10MIN,
    INC_HR,
    INC_NONE,
}INCMode_T;


typedef enum{
    IDLE,    
    SET_CLOCK,
    s_NONE,
}State_T;
typedef enum{
    DISP_AI01,
    DISP_NAME,
            
    DISP_NONE,
}State_L4;

typedef union
{
    unsigned char byt;//one byte
    struct    //bits
    {       
        unsigned b0 :1; //bit debounce RA5
        unsigned b1 :1;
        unsigned b2 :1;
        unsigned b3 :1;
        
        unsigned b4 :1;
        unsigned b5 :1;
        unsigned b6 :1;
        unsigned b7 :1;// bit 7 , msb
    };
}Flag_T;

State_T state = IDLE;    
INCMode_T IncMode = INC_SEC;

State_L4  state_L4 = DISP_AI01;

float SetPoint = 40 ;
Flag_T Flags;
  char ch;
  int max=7;
  char array[16];
  int index=0;
  char ready=0;
  
    
  
int H = 3;
unsigned char HeatON = 0 ;
char *IncStr[SELSIZE]= {"Sec", "Min","HR"};

char *StateStr[2]= {"Normal","SetClok"};
int   IncVal[SELSIZE]= {1,60,3600};

unsigned char EnableHeat = 0;

int MaxHour = 10;
int CookingTime = 0;//maximum 10hours = 10* 3600 = 36000

int MaxCookTime = 5*3600;
long Clock = 0;//

char StopCooking = 0;
int CountBeep =0;


void setupPorts(void) {
    
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}


void initTimers01(void) {
    
    INTCON =0;
    
    INTCON2 =0;
    INTCON3 =0;
    T0CON = 0;
  
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    PIE1=0;
    PIE1bits.RCIE =1;
    
    
    RCON = 0;
    RCONbits.IPEN =0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.GIEL = 1;
    INTCONbits.T0IE = 1;
    INTCONbits.INT0IE= 1;
    INTCON2bits.INTEDG0 = 1;
    INTCON2bits.INTEDG1 = 1;
    INTCON2bits.INTEDG2 = 1;
    INTCON3 = 0;
    INTCON3bits.INT1IE =1;
    INTCON3bits.INT2IE = 1;
    
    PIE2 = 0;
    
    PIR2 = 0;
    T3CON =0;
  
    T0CONbits.TMR0ON = 1;
}

void RX_ISR(void)
{
    // PIR1bits.RCIF=0;
    ch=RCREG;
    if (ready)return;
    
    if(ch == '<')
        
        
    {
        index=0;
        array[index++]=ch;
    }

    else if(index < max)
    {
        if(array[0]!='<') return;
        array[index++]=ch;

         if(ch =='>')
        {
            ready =1;
            array[index]=0;               

        }
    }                
    
    
    
    
}
void int1_isr(void )
{
    if(!Flags.b1){
        state++;
        if(state >= s_NONE)  state = IDLE;
        Flags.b1 =1;
    }
    INTCON3bits.INT1IF =0;
}
void int0_isr()
{
    if(!Flags.b0){
        state_L4++;
        
        
        if(state_L4 >= DISP_NONE)  state_L4 = DISP_AI01;
        Flags.b0 =1;
    }
    INTCONbits.INT0IF =0;
}
void int2_isr(void)
{
    INTCON3bits.INT2IF =0;
    if(!Flags.b2){
        IncMode++;
        if(IncMode >= INC_NONE )  IncMode = INC_SEC;
        
        Flags.b2 =1;
    }
}
void timer_isr(void)
{
   if(state != SET_CLOCK){
            Clock++;
            if (Clock >= 24*3600) Clock = 0;
        }    
        TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
        TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
      
        INTCONbits.T0IF = 0;
        
}

void __interrupt(high_priority) highIsr(void)//new syntax
{    
    if(INTCON3bits.INT1IF)
    {  // Increment Selection Mode
        int1_isr();        
    }
    
   else if(INTCON3bits.INT2F){      
        int2_isr();
    }

    
    
    else if(INTCONbits.TMR0IF)
    {//Timer 0 Interrupt
        timer_isr();      
    }
    else if(PIR1bits.RCIF)
    {     
       RX_ISR();
    }
    else if(INTCONbits.INT0IF)
        
    {  // Toggle display mode
        int0_isr();      
    }
}
 
void main(void) {
    int m;
    char Buffer[32]; // for sprintf
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float voltage;
    setupPorts();
    setupSerial();
    lcd_init();
    init_adc_no_lib();
    int readh =PORTCbits.RC5;
    
    
     int readc =PORTCbits.RC2;
     int readd0 =0;
     int readd1=0;
     int readd2=0 ;
     int readd3=0 ;
     int readd4 =0;
     int readd5 =0;
     int readd6 =0;
     int readd7 =0;
     
     
     
     
      
    lcd_putc('\f'); //clears the display
    initTimers01();   // These will be used to measure the speed    
    EnableHeat = 0;
    Flags.byt =0;
    float T =  read_adc_voltage(2)*100;
    int hr, min, sec, t;     
    char displayClock = 0; //
    
  while (1) {
    CLRWDT(); // 
    
    
    
    if (ready) {
        lcd_gotoxy(1, 4);       
        
        lcd_puts(array);
        if ((array[1] == 'R') && (array[2] == 't') && (array[3] == '>')) {
            sprintf(Buffer, "Time:%02d:%02d:%02d\n ", hr, min, sec);
            
            
            
            send_string_no_lib(Buffer);
        }
        else if ((array[1] == 'R') && (array[2] == 'A') && (array[4] == '>')) {
            switch (array[3]) {
                case '0':   
                    
                    sprintf(Buffer, "an0=%f\n", AN[0]);
                    send_string_no_lib(Buffer);
                    break;
                case '1':
                    sprintf(Buffer, "an1=%f\n", AN[1]);
                    send_string_no_lib(Buffer);
                    break;
                case '2':
                    sprintf(Buffer, "an2=%f\n", AN[2]);
                    send_string_no_lib(Buffer);
                    break;
            }
        }
        else if ((array[1] == 'R' && (array[3] == '>'))) { ///<WHV
            switch (array[2]) {
                case 'H':
                    readh = PORTCbits.RC5;
                    if (readh == 1)
                        send_string_no_lib("ON\n");
                    else
                        send_string_no_lib("OFF\n");
                    break;
                case 'C':
                    readc = PORTCbits.RC2;
                    if (readc == 1)
                        send_string_no_lib("ON\n");
                    else
                        send_string_no_lib("OFF\n");
                    break;
                case 'D':
                    readd0 = PORTDbits.RD0;
                    if (readd0 == 1)
                        send_string_no_lib("D0:ON\n");
                    else
                        send_string_no_lib("D0:OFF\n");

                    readd1 = PORTDbits.RD1;
                    if (readd1 == 1)
                        send_string_no_lib("D1:ON\n");
                    else
                        send_string_no_lib("D1:OFF\n");

                    readd2 = PORTDbits.RD2;
                    if (readd2 == 1)
                        send_string_no_lib("D2:ON\n");
                    else
                        send_string_no_lib("D2:OFF\n");

                    readd3 = PORTCbits.RC3;
                    if (readd3 == 1)
                        send_string_no_lib("D3:ON\n");
                    else
                        send_string_no_lib("D3:OFF\n");

                    readd4 = PORTDbits.RD4;
                    if (readd4 == 1)
                        send_string_no_lib("D4:ON\n");
                    else
                        send_string_no_lib("D4:OFF\n");

                    readd5 = PORTDbits.RD5;
                    if (readd5 == 1)
                        send_string_no_lib("D5:ON\n");
                    else
                        send_string_no_lib("D5:OFF\n");
                    readd6 = PORTDbits.RD6;
                    if (readd6 == 1)
                        send_string_no_lib("D6:ON\n");
                    else
                        send_string_no_lib("D4:OFF\n");
                    readd7 = PORTDbits.RD7;
                    if (readd7 == 1)
                        send_string_no_lib("D7:ON\n");
                    else
                        send_string_no_lib("D7:OFF\n");
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == 'H') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTCbits.RC5 = 0;
                    break;
                case '1':
                    PORTCbits.RC5 = 1;
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == 'C') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTCbits.RC2 = 0;
                    break;
                case '1':
                    PORTCbits.RC2 = 1;
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == 'H') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTCbits.RC5 = 0;
                    break;
                case '1':
                    PORTCbits.RC5 = 1;
                    break;
            }
        }             
        else if ((array[1] == 'W') && (array[2] == '4') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTDbits.RD4 = 0;
                    break;
                case '1':
                    PORTDbits.RD4 = 1;
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == '3') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTDbits.RD3 = 0;
                    break;
                case '1':
                    PORTDbits.RD3 = 1;
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == '2') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTDbits.RD2 = 0;
                    break;
                case '1':
                    PORTDbits.RD2 = 1;
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == '1') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTDbits.RD1 = 0;
                    break;
                case '1':
                    PORTDbits.RD1 = 1;
                    break;
            }
        }
        else if ((array[1] == 'W') && (array[2] == '0') && (array[3] == 'V') && (array[5] == '>')) {
            switch (array[4]) {
                case '0':
                    PORTDbits.RD0 = 0;
                    break;
                case '1':
                    PORTDbits.RD0 = 1;
                    break;
            }
        }
        array[0] = 0;
        ready = 0;          
    }
}

    
        // Read the Analog Values
       for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
           voltage = read_adc_voltage((unsigned char) channel);
           AN[channel] = voltage; // store in array AN0--AN2
       }
        
        T = AN[2] *100;
        
        if(!PORTBbits.RB3){ // Increment By IncTime
          
           if (state == SET_CLOCK){
                Clock += IncVal[IncMode];
                if(Clock >= (long)86400) Clock = 0;   //cast           
            }           
        }
        if(!PORTBbits.RB4){ // Decrement By IncTime
            
             if (state == SET_CLOCK){
                Clock -= IncVal[IncMode];
                if(Clock < 0) Clock = 0; ;               
            }
            
        }
        displayClock = 1;
        lcd_gotoxy(1, 1);
        if(displayClock){
            hr = Clock/3600;
            t =  Clock%3600;         
        }
        
        min = t/60;
        sec = t%60;
        sprintf(Buffer, "%02d:%02d:%02d %4.2f",  hr,min,sec, T);
        lcd_puts(Buffer);
        
        lcd_gotoxy(1, 2);
        sprintf(Buffer,"H:%3s    C:%3s", PORTCbits.RC5 ? "ON":"OFF" ,PORTCbits.RC2 ? "ON":"OFF");
         lcd_puts(Buffer);
         
        lcd_gotoxy(1, 3);
        if(state == IDLE )
        sprintf(Buffer, "%-5s           ", StateStr[state]);
        else 
            sprintf(Buffer, "%-5s  MD:%-7s", StateStr[state] , IncStr[IncMode]);
         lcd_puts(Buffer);
        lcd_gotoxy(1, 4);
        if(state_L4 == DISP_AI01)
            sprintf(Buffer, "AI0:%3.1f  AI1:%3.1f ", AN[0],AN[1]); // Display Speed   
        else 
           sprintf(Buffer,"%-16s", "omar raddad");
        lcd_puts(Buffer);
        
        delay_ms(100);
        if(Flags.b2) { delay_ms(200); Flags.b2 =0;}
        if(Flags.b1) { delay_ms(200); Flags.b1 =0;}
        if(Flags.b0) { delay_ms(200); Flags.b0 =0;}
    }





































































































///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd///////////////312328tyuhdkdshjgsvdhdsjgfhjdhkdsjhdsjashjhgdsdsjhdsfdhjfhdsfhdsfhdshfhdjfdjsjfjdkjdhihuhewfdbkskdfkds
///////////////////////sdfkjdsjfdhksjfdjshfjksdjhfdsfjhghdsjfjfskjfddhsfhdkjsfhjhkdjsfjdsfkkkkkkkk
//ds/sd
//ds/s
//dsdsdsds//
//dssd