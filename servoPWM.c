/******************************************************************************
 *                                                                            *
 *   Program to command position of three Servo motors using PWM and UART     *
 *                                                                            *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * Author:                    Henry Ugochukwu Odoemelem                       *
 * Created: 4-May-2020                                                        *
 *                                                                            *
 *                                                                            *
 ******************************************************************************
 *                                                                            *
 * Hardware Setup                                                             *
 *                                                                            *
 *                               MSP430FR5969                                 *
 *                            -----------------                               *
 *                           |             P1.5|--> (arm 3)                   *
 *                           |             P1.4|--> (arm 2)                   *
 *                           |             P1.3|--> (arm 1,link1)             *
 *                           |                 |                              *
 *                            -----------------                               *
 *                                                                            *
 ******************************************************************************/


#include <msp430fr5969.h>
#include <stdint.h>

uint8_t selectArm = 0;
#pragma PERSISTENT(savedState1)
  uint8_t savedState1 = 0;
#pragma PERSISTENT(savedState2)
  uint8_t savedState2 = 0;
#pragma PERSISTENT(savedState3)
  uint8_t savedState3 = 0;

//lookup table 0(900 counts) to 180(2700 counts) degrees in terms of number of counts, increments of 10 counts
const uint16_t Angle[181] =
{
 900,910,920,930,940,950,960,970,980,990,
 1000,1010,1020,1030,1040,1050,1060,1070,
 1080,1090,1100,1110,1120,1130,1140,1150,
 1160,1170,1180,1190,1200,1210,1220,1230,
 1240,1250,1260,1270,1280,1290,1300,1310,
 1320,1330,1340,1350,1360,1370,1380,1390,
 1400,1410,1420,1430,1440,1450,1460,1470,
 1480,1490,1500,1510,1520,1530,1540,1550,
 1560,1570,1580,1590,1600,1610,1620,1630,
 1640,1650,1660,1670,1680,1690,1700,1710,
 1720,1730,1740,1750,1760,1770,1780,1790,
 1800,1810,1820,1830,1840,1850,1860,1870,
 1880,1890,1900,1910,1920,1930,1940,1950,
 1960,1970,1980,1990,2000,2010,2020,2030,
 2040,2050,2060,2070,2080,2090,2100,2110,
 2120,2130,2140,2150,2160,2170,2180,2190,
 2200,2210,2220,2230,2240,2250,2260,2270,
 2280,2290,2300,2310,2320,2330,2340,2350,
 2360,2370,2380,2390,2400,2410,2420,2430,
 2440,2450,2460,2470,2480,2490,2500,2510,
 2520,2530,2540,2550,2560,2570,2580,2590,
 2600,2610,2620,2630,2640,2650,2660,2670,
 2680,2690,2700

};

//void uartTx(uint8_t data);




/* MAIN PROGRAM */
void main(void)
{
    // Stop watchdog timer.
    WDTCTL = WDTPW | WDTHOLD;


    // Initialize the clock system to generate 16 MHz DCO clock.
    CSCTL0_H    = CSKEY_H;              // Unlock CS registers.

    CSCTL1      = DCOFSEL_0;            // for 1MHz
    CSCTL2      = SELA__LFXTCLK |        // Set ACLK = LFXTCLK = 32 kHz.
                  SELS__DCOCLK |        //   Set SMCLK = DCOCLK.
                  SELM__DCOCLK;         //   Set MCLK = DCOCLK.
                                        // SMCLK = MCLK = DCOCLK = 1 MHz.
    CSCTL3      = DIVA__1 |             //   Set ACLK divider to 1.
                  DIVS__1 |             //   Set SMCLK divider to 1.
                  DIVM__1;              //   Set MCLK divider to 1.
                                        // Set all dividers to 1.
    CSCTL0_H    = 0;                    // Lock CS registers.


    // Initialize unused GPIOs to minimize energy-consumption.
    // Port 1:
    P1DIR = 0xFF;
    P1OUT = 0x00;
    // Port 2:
    P2DIR = 0xFF;
    P2OUT = 0x00;
    // Port 3:
    P3DIR = 0xFF;
    P3OUT = 0x00;
    // Port 4:
    P4DIR = 0xFF;
    P4OUT = 0x00;
    // Port J:
    PJDIR = 0xFFFF;
    PJOUT = 0x0000;

    // Initialize port 1:
    P1DIR |= BIT3;                      // P1.3 - output
    P1OUT &= ~BIT3;

    P1DIR |= BIT4;                      // P1.4 - output
    P1OUT &= ~BIT4;

    P1DIR |= BIT5;                      // P1.5 - output
    P1OUT &= ~BIT5;




      // Initialize port 2:
      // Select Tx and Rx functionality of eUSCI0 for hardware UART.
      // P2.0 - UART Tx (UCA0TXD).
      // P2.1 - UART Rx (UCA0RXD).
      P2SEL0 =  ~(BIT1 | BIT0);
      P2SEL1 = BIT1 | BIT0;


    // Disable the GPIO power-on default high-impedance mode to activate the
    // previously configured port settings.
    PM5CTL0 &= ~LOCKLPM5;



     /* Initialization of the serial UART interface */
    UCA0CTLW0 |= UCSSEL__SMCLK |        // Select clock source SMCLK = 1 MHz.
                 UCSWRST;               // Enable software reset.
    // Set Baud rate of 9600 Bd.
    // Recommended settings available in table 30-5, p. 765 of the User's Guide.
    UCA0BRW = 6;                        // Clock prescaler of the
                                        //   Baud rate generator.
    UCA0MCTLW = UCBRF_8 |               // First modulations stage.
                UCBRS5 |                // Second modulation stage.
                UCOS16;                 // Enable oversampling mode.
    UCA0CTLW0 &= ~UCSWRST;              // Disable software reset and start
                                        //   eUSCI state machine.

    UCA0IE |= UCRXIE;             // enable interrupt, flag is clear initially by default


    /* TODO HARDWARE PWM INIT */
     //P1.3, TA1.2
     P1SEL0   |= BIT3;                // enable primary function for P1.3, TA1.2
     TA1CCTL2 |= OUTMOD_7;           // select output mode  Reset/Set
     TA1CCR0 = 20000;                //PWM Period (20ms, 1MHz/50)
     TA1CCR2 = Angle[savedState1];

     //P1.4, TB0.1
     P1SEL0   |= BIT4;                // enable primary function for P1.4, TB0.1
     TB0CCTL1 |= OUTMOD_7;           // select output mode  Reset/Set
     TB0CCR0 = 20000;                   //PWM Period (20ms, 1MHz/50)
     TB0CCR1 = Angle[savedState2];

     //P1.5, TB0.2
     P1SEL0   |= BIT5;                // enable primary function for P1.5, TB0.2
     TB0CCTL2 |= OUTMOD_7;           // select output mode  Reset/Set
     TB0CCR2 = Angle[savedState3];





     TA1CTL = TACLR;                 // clear and stop timer A1
     TA1CTL = TASSEL__SMCLK| MC__UP;  //select clock source SMCLK, 1MHz, up mode

     TB0CTL = TBCLR;                 // clear and stop timer B0
     TB0CTL = TBSSEL__SMCLK| MC__UP;  //select clock source SMCLK, 1MHz, up mode


    // Enable interrupts globally.
    __bis_SR_register(GIE);
    __no_operation();



}

//ISR
#pragma vector  = USCI_A0_VECTOR
__interrupt void  USCI_A0_ISR(void)
{


    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG)) {

        case USCI_UART_UCRXIFG:
           UCA0IE &=~ UCRXIE; // disable interrupt while in process


           // 251, 252,253 to represent arm1,arm2,arm3
           // angle data ranges from 0 to 180 ie B4
           // first 251,252 or 253 has to be received to select the
           // right arm before angle data is received
           if((uint8_t)UCA0RXBUF == 0xFB){//251
               selectArm =  0xFB;

           }
           else if((uint8_t)UCA0RXBUF == 0xFC){//252
               selectArm =  0xFC;
           }
           else if((uint8_t)UCA0RXBUF == 0xFD){//253
               selectArm =  0xFD;
           }
           else if ((uint8_t)UCA0RXBUF <= 0xB4){//Do if in range of angle
               if(selectArm ==  0xFB){// arm 1, P1.3
                   FRCTL0  = FRCTLPW;           //FRAM write enable
                   savedState1 = (uint8_t)UCA0RXBUF;  // save the data state
                   FRCTL0_H  = 0;                    // FRAM write disable
                   TA1CCR2 = Angle[savedState1 ];

                 }
               else if(selectArm ==  0xFC){//arm2 P1.4

                   FRCTL0  = FRCTLPW;           //FRAM write enable
                   savedState2 = (uint8_t)UCA0RXBUF;  // save the data state
                   FRCTL0_H  = 0;                    // FRAM write disable
                  TB0CCR1 = Angle[savedState2];

                }
                else if(selectArm ==  0xFD){//arm3 P1.5
                  FRCTL0  = FRCTLPW;           //FRAM write enable
                  savedState3 = (uint8_t)UCA0RXBUF;  // save the data state
                  FRCTL0_H  = 0;                    // FRAM write disable
                  TB0CCR2 = Angle[savedState3];

                }

           }


           UCA0IE |= UCRXIE;//enable interrupt after processing
           break;
        default: break;
    }
}



//
//void uartTx(uint8_t data)
//{
//        while((UCA0STATW & UCBUSY));    // Wait while module is busy with data.
//
//        UCA0TXBUF = data;         // Transmit element i of data array.
//
//}

