//        MSP430FG6626��Դ���ƣ�����LED�������У����п��� LED ǿ�ȺͶ�ȡ����ģת�����ĸ��ӹ��ܡ�
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |            XT2IN|-
//            |                 | HF XTAL (20MHz)     ԭ��ͼ��18MHz
//            |           XT2OUT|-
//            |                 |

//            |     (1)P6.4/AD0+|<----------------- NV1
//            |     (2)P6.5/AD0-|<----------------- Gnd
//            |     (3)P6.6/AD1+|<----------------- NV2
//            |     (4)P6.7/AD1-|<----------------- Gnd
//            |     (5)P7.4/AD2+|<----------------- NV3
//            |     (6)P7.5/AD2-|<----------------- Gnd
//            |     (7)P7.6/AD3+|<----------------- NV4
//            |     (8)P7.7/AD3-|<----------------- Gnd

//            | (10)   P5.1/DAC0|-----------------> LED 1
//            | (11)   P5.6/DAC1|-----------------> LED 2

//            |      (23)   P2.5|<----------------- Timer input
//            |      (50)   P4.0|<----------------- 
//            |      (51)   P4.1|<----------------- Input   ԭ��ͼû����
//            |      (60)   P8.2|<----------------- Input LED color
//            |      (61)   P8.3|<----------------- Input Calibration input

//            |      (39)   P1.5|
//            |      (40)   P1.6|

//            |(18) P2.0/UCA0TXD|-----------------> 19200 - 8N1 (transmit)
//            |                 | 
//            |(19) P2.1/UCA0RXD|<----------------- 19200 - 8N1   ����������

//            |      (21)   P2.3|-----------------> Vreg1
//            |      (22)   P2.4|-----------------> Vreg2

//             -----------------
//*****************************************************************************

//******************************************************************************
//  MSP430FG662x Demo - CTSD16, Single Conversion of 3 Input Channels
//
//  Description: This program uses the CTSD16 module to perform a single
//  conversion on 3 input channels (0, 1, and 2). A CTSD16 interrupt occurs when
//  the conversions have completed.
//

#include <msp430.h>
#include <intrinsics.h>

void Port_Mapping(void);            //�˿�ӳ�亯��
void delay(void);                    //��ʱ����

#define Num_of_Channels    4
#define R_max    40000                 // R:740nm���
#define R_min    3000
#define IR_max   40000                 // IR:850nm�����
#define IR_min   3000

unsigned int Rval = 0;                //���ֵ
unsigned int IRval = 0;               //�����ֵ
unsigned int Chresults[Num_of_Channels] = {0};     //4��ͨ�����
unsigned int Chresults2[100] = {0};
unsigned int iarray[100] = {0};
unsigned int index = 0;        //����������
unsigned int dum1 = 0;
unsigned int res[80] = {0};           //�ֱ���
long res0 = 0;
unsigned int rcount = 0;
unsigned int ttest[80] = {0};
unsigned int NV0 = 0;                           // New value
long DV0 = 0;                                   // Acc value
unsigned int NV1 = 0;                           // New value
long DV1 = 0;                                   // Acc value
unsigned int NV2 = 0;                           // New value
long DV2 = 0;                                   // Acc value
unsigned int NV3 = 0;                           // New value
long DV3 = 0;                                   // Acc value
unsigned int NV4 = 0;                           // New value
long DV4 = 0;                                   // Acc value
long D1R = 0;
long D2R = 0;
long D3R = 0;
long D4R = 0;
long D1IR = 0;
long D2IR = 0;
long D3IR = 0;
long D4IR = 0;
unsigned int dummy = 0;
unsigned int red = 0;

unsigned int acount = 0;
unsigned int bcount = 0;
unsigned int Pacount = 0;
unsigned int Pbcount = 0;

long DV0a = 0;
long DV0b = 0;
long D01 = 0;
long D02 = 0;
long PV02 = 0;
long PV01 = 0;
long PV01b = 0;

unsigned int auto1 = 0;

unsigned int ADCcount = 0;

unsigned int RInt = 0xfff;       // ~1.5V x 2 was FFF  1111 1111 1111
unsigned int Rstep = 0x0A;       //  0000 1010
unsigned int IRInt = 0xFFF;
unsigned int IRstep = 0x0A;


int __low_level_init(void)      //��.h�ļ����棿
  {
     WDTCTL = WDTPW + WDTHOLD;   // Stop WDT (Watch Dog Timer)
  }

void main(void) 
  {
    WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog
    
    CTSD16CCTL0 |= CTSD16SC;                  // Workaround for CTSD16OFFG errata  CTSD16ͨ��0���ƼĴ���  Start conversion
    do
    {
      CTSD16CTL &= ~CTSD16OFFG;               //CTSD16���ƼĴ���   CTSD16ʱ�ӹ��ϱ�־
    }
    while (CTSD16CTL&CTSD16OFFG);            // End of CTSD16OFFG workaround

    while(BAKCTL & LOCKBAK)                   // Unlock XT1 pins for operation ��غ󱸿��ƼĴ���  0b = Backup subsystem not locked
       BAKCTL &= ~(LOCKBAK);
    
    P7SEL |= BIT2;                            // Port select XT2  p7.2   18Mhz
    UCSCTL6 &= ~XT2OFF;                       // Set XT2 On    Enable XT2    0b = XT2     ͳһʱ��ϵͳ���ƼĴ���6
    
    UCSCTL3 |= SELREF_2;                      // FLLref = REFO
                                              // Since LFXT1 is not used,
                                              // sourcing FLL with LFXT1 can cause
                                              // XT1OFFG flag to set
    UCSCTL4 |= SELA_2;                        // ACLK=REFO,SMCLK=DCO,MCLK=DCO
    
    //UCSCTL6 &= ~(XT1OFF);                     // XT1 On    32.768kHz
    //UCSCTL6 |= XCAP_3;                        // Internal load cap     Oscillator capacitor selection.����������ѡ��
    // Loop until XT1 fault flag is cleared
    
    do
    {
      UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);
                                              // Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= ~OFIFG;                      // Clear fault flags      �жϱ�־�Ĵ���
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
    
    //UCSCTL4 |= SELS_5 | SELM_5;             // SMCLK=MCLK=XT2
    
    //UCSCTL6 &= ~XT2DRIVE0;                  // Decrease XT2 Drive according to
                                              // expected frequency
    //UCSCTL4 |= SELS_5 | SELM_5;             // SMCLK=MCLK=XT2

    P4DIR &= ~0x03;                           // P4.0 and 4.1 inputs   0000 0011
    P8DIR &= ~0x0C;                           // P8.2 and 8.3 niputs   0000 1100
    
    Port_Mapping();
    
    P2SEL |= 0x03;                            // Assign P2.0 to UCA0TXD and...  �˿�2ѡ��Ĵ���  0b=ѡ����I/O����  1b=ѡ��������ģ�鹦��
    P2DIR |= 0x03;                            // P2.1 to UCA0RXD        �˿�2����Ĵ���         
    
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 3;                              // 1MHz 19200 (see User's Guide)
    UCA0BR1 = 0;                              // 1MHz 19200
    UCA0MCTL = UCBRS_0 | UCBRF_6 | UCOS16;    // Modln UCBRSx=0, UCBRFx=6,
                                              // over sampling
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    //UCA0IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
    UCA0IE &= ~UCRXIE;                        // Disable USCI_A0 RX interrupt

    P6SEL |= BIT4 | BIT5 | BIT6 | BIT7;       // Select AD0+ to AD3-...
    P7SEL |= BIT4 | BIT5 | BIT6 | BIT7;       // as analog input

    CTSD16CTL = CTSD16REFS;                   // Internal ref   CTSD16 reference select.
    CTSD16CCTL0  |= CTSD16SNGL | CTSD16DF;    // 2s complement, single convers   Single conversion mode select   CTSD16 data format
    //CTSD16CCTL0  |= CTSD16DF;               // 2s complement   
    //CTSD16INCTL0 |= CTSD16INCH_9;           // Select AD0+ / AD0- for CH0  CTSD16ͨ��0������ƼĴ���  01001b = in+ = AD0+, in- = AD0-
    CTSD16INCTL0 |= CTSD16RRI | CTSD16INCH_9; // Select AD0+ / AD0- for CH0  ��Թ�����ʹ��
    //CTSD16IFG &= ~CTSD16IFG0;               // Clear CH0 result interrupt flag
    CTSD16IE |= CTSD16IE0;                    // Enable CH0 result interrupts
    CTSD16IFG &= ~(CTSD16IFG0 | CTSD16OVIFG0);// Clear CH0 interrrupt flags    CTSD16ת����0����жϱ�־ 
    //CTSD16IE |= CTSD16IE0 | CTSD16OVIE0;    // Enable CH0 overflow, result interrupts   CTSD16ת����0����ж�ʹ��
    
    while(!(CTSD16CTL&CTSD16RRIRDY));         // Wait for rail-to-rail input ready  ��Թ�����ʹ��
    
    P2IES = 0x00;                             // Edge int., low to high   �˿�2�жϱ���ѡ��Ĵ���
    P2IE = 0x20;                              // Enable P2.5 int.   0010 0000
    P2IFG &= ~0x020;

    __delay_cycles(2000);                     // Delay ~120us for 1.2V ref to settle
    
    P2IE |= 0x20;                             // Enable P2.5 int.   0010 0000
    
    RInt = 0x0fff;                            //was fff   0000 1111 1111 1111
    IRInt = 0x0fff;
    Rstep = 0x0A;                             //0000 1010
    IRstep = 0x0A;
    
    //DAC12_0CTL0 = DAC12IR | DAC12SREF_0 | DAC12AMP_5 | DAC12ENC | DAC12CALON;   //DAC12���ƼĴ���0   ref 1.5V, 0x7FF 0.75V
    //DAC12_0DAT = 0x7FF;                       // ~0.75V
    DAC12_0CTL0 = DAC12IR | DAC12SREF_1 | DAC12AMP_5 | DAC12ENC | DAC12CALON;     // ref Vcc, 0x7FF 1.5V
    DAC12_0DAT = RInt;                         // ~1.5V
    DAC12_1CTL0 = DAC12IR | DAC12SREF_1 | DAC12AMP_5 | DAC12ENC | DAC12CALON;     // ref Vcc, 0x7FF 1.5V
    DAC12_1DAT = IRInt;                        // ~1.5V

    
 //   UCA0TXBUF = 0x62;               0110  0010
    
 //       CTSD16CCTL0 |= CTSD16SC;                // Set bit to start conversion
 //       __bis_SR_register(LPM0_bits | GIE);     // Enter LPM0 w/ interrupts
    
 //�����Ǿ���ʱ�ӣ�DAC����
    
    
    D1R = 0;
    
    while(1) 
    {
        CTSD16CCTL0 |= CTSD16SC;            // Set bit to start conversion
        __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
        __no_operation();                   // For debugger
        if (index >= Num_of_Channels) 
        {
          index = 0;                        // SET BREAKPOINT HERE
          CTSD16INCTL0 = CTSD16INCH_9;      // CTSD16 ͨ��0������ƼĴ���  9=01001b = in+ = AD0+, in- = AD0-  Reset input to CH0
        } 
        else 
        {
          CTSD16INCTL0 += 2;                // Increment CH count by 1 һ������һ��ͨ��
        }
    }
    
}
//������ͨ����������

// Port2 ISR�жϷ������
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR (void)
{
  _BIC_SR(LPM0_bits);                       // Clear CPUOFF bit from 0(SR) Exit LPM0 �˳��͹���ģʽ
  CTSD16IE &= ~CTSD16IE0;                   // Disable CH0 result interrupts
  CTSD16CCTL0 &= ~CTSD16SC;                 // Set bit to stop conversion
  //CTSD16IE |= CTSD16IE0 | CTSD16OVIE0;    // Enable CH0 overflow, result interrupts
  //CTSD16CCTL0 |= CTSD16SC;                // Set bit to start conversion
  P2IFG &= ~0x020;   //  �˿�2�жϱ�־�Ĵ��� p2.5
  P2IES ^= 0x20;     //0010 0000   �˿�2�жϱ���ѡ��Ĵ���  1b = P2IFG flag is set with a high-to-low transition. ^ ��λ���
  //P2IE &= ~0x020;  //0010 0000   �˿�2�ж�ʹ�����üĴ���
 
  
//����������У׼
  if (P8IN & 0x08) 
  {                        // P8.3 high  p8.3 Input Calibration����У׼     0000 1000
    auto1 = 1;
  } 
  else 
  {
    auto1 = 0;
  }
  
  //���У׼
  if (auto1 == 1) 
  {
    if (Rval > R_max) 
    {
      if (RInt > Rstep) 
      {
        RInt = RInt - Rstep;
      }
      //DAC12_0DAT = RInt;                       // R on
      //DAC12_1DAT = 0x00;                       // IR off
    } 
    
    else 
    {
      if (Rval < R_min) 
      {
        if (RInt < (0xFFF - Rstep)) 
        {
          RInt = RInt + Rstep;
        }
        //DAC12_0DAT = RInt;                      // R on
        //DAC12_1DAT = 0x00;                      // IR off
      }
    }
    
    //�����У׼
    if (IRval > IR_max) 
    {
      if (IRInt > IRstep) 
      {
        IRInt = IRInt - IRstep;
      }
    } 
    
    else 
    {
      if (IRval < IR_min) 
      {
        if (IRInt < (0xFFF - IRstep)) 
        {
          IRInt = IRInt + IRstep;
        }
      }
    }
  }
  
//���¿�ʼ���⣺ϵͳʱ����ƽ��淢�����ͺ����
  //if (P8IN & 0x04) {                 // P8.2 high  p8.2��Input LED color��R or IR    0000 0100

  if (P2IN & 0x20)                     //0010 0000   p2.5  p2.5��Timer input
  {                    
    DAC12_0DAT = 0x00;                 // R off
    DAC12_1DAT = IRInt;                // IR on
  } 
  else 
    {
      DAC12_0DAT = RInt;                // R on
      DAC12_1DAT = 0x00;                // IR off
    }
  
  //res[rcount++] = ADCcount;
  ADCcount = 0;
  if (rcount > 80) 
  {
    rcount =0;
  }


  if ((P2IES && 0x20) == 0x00)              //0010 0000  p2.5 Timer input
  {
  
    P2SEL |= 0x03;                          // 0000 0011  Assign P2.0 to UCA0TXD and...
    P2DIR |= 0x03;                          // P2.1 to UCA0RXD    ������������MSP430FG6626
  
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D1R;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1R >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1R >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1R >> 24;
    UCA0TXBUF = dummy;
    
    
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D2R;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D2R >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D2R >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D2R >> 24;
    UCA0TXBUF = dummy;

  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D3R;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D3R >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D3R >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D3R >> 24;
    UCA0TXBUF = dummy;
    

  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D1IR;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1IR >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1IR >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1IR >> 24;
    UCA0TXBUF = dummy;
    
  while (!(UCA0IFG&UCTXIFG));                // TX buffer ready?
    UCA0TXBUF = D2IR;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D2IR >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D2IR >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D2IR >> 24;
    UCA0TXBUF = dummy;
    
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D3IR;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D3IR >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D3IR >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D3IR >> 24;
    UCA0TXBUF = dummy;
    
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D4R;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D4R >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D4R >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D4R >> 24;
    UCA0TXBUF = dummy;

  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D4IR;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D4IR >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D4IR >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D4IR >> 24;
    UCA0TXBUF = dummy;
    
    //�ֱ�������
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready? Resolution 18
    UCA0TXBUF = res0;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = res0 >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = res0 >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = res0 >> 24;
    UCA0TXBUF = dummy;

  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = 13;                        // TXBUF0   0000 1101
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = 17;                        // TXBUF0   0001 0001
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = 19;                        // TXBUF0   0001 0011
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = 0;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = 0;                        // TXBUF0        Ϊʲô=0д���飿  UCAxTXBUF Register = eUSCI_Ax Transmit Buffer Register = eUSCI_Ax���ͻ���Ĵ��� 
   
    P2SEL &= ~0x03;                            // ֹͣ���ͺͽ�������  Assign P2.0 to UCA0TXD and...
    P2DIR &= ~0x03;                            // P2.1 to UCA0RXD off
  }
  
  __no_operation();                   // For debugger
  index = 0;
  res0 = 0;
  DV0=0;
  DV1=0;
  DV2=0;
  CTSD16IFG &= ~CTSD16IFG0;               // Clear CH0 result interrupt flag
    CTSD16CCTL0 |= CTSD16SC;            // Set bit to start conversion
  CTSD16IE |= CTSD16IE0 | CTSD16OVIE0;    // Enable CH0 overflow, result interrupts
//  CTSD16CCTL0 |= CTSD16SC;            // Set bit to start conversion
//  CTSD16INCTL0 |= CTSD16INTDLY;           // delay after 1st conversion
//  __delay_cycles(2000);                   // Delay ~120us for 1.2V ref to settle
//  CTSD16IE |= CTSD16IE0;    // Enable CH0 result interrupt
//  __bis_SR_register(LPM0_bits | GIE); // Enter LPM0 w/ interrupts
  //CTSD16IE &= ~CTSD16IE0;    // Disable CH0 result interrupts
  //CTSD16CCTL0 &= ~CTSD16SC;            // Set bit to stop conversion
  //P2IE |= 0x20;                              // Enable P2.5 int.
}

//�궨��
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=CTSD16_VECTOR         // #pragma vector�ṩ�жϺ�����ڵ�ַ,��ʹ�� _interrupt void _Name_Function�������жϺ�������
__interrupt void CTSD16_ISR(void)   //�ùؼ���__interrupt������һ���жϺ�������#pragma vector������ʾһ���ж�����
#elif defined(__GNUC__)
void __attribute__ ((interrupt(CTSD16_VECTOR))) CTSD16_ISR (void)
#else
#error Compiler not supported!     //������ʾ��Ϣ
#endif
{
    switch (__even_in_range(CTSD16IV,CTSD16IV_CTSD16MEM0)) //CTSD16�ж������Ĵ���  CTSD16 �ж�����ֵ��д��üĴ�����������й�����жϱ�־��
    {
        case CTSD16IV_NONE: break;
        case CTSD16IV_CTSD16OVIFG: 
          CTSD16IFG &= ~CTSD16OVIFG0;        // OPTIONAL BREAKPOINT HERE
          break;
        case CTSD16IV_CTSD16MEM0:
          //CTSD16IFG &= ~(CTSD16IFG0 | CTSD16OVIFG0); // Clear CH0 interrrupt flags
          ADCcount++;
          //__delay_cycles(2000);                   // Delay ~120us for 1.2V ref to settle
                   Chresults[index++] = CTSD16MEM0; // CTSD16 ͨ��0ת���洢�Ĵ���  Save CH0 result (clears IFG)
                   Chresults2[index] = CTSD16MEM0; 
                   DV0=Chresults[0]+DV0;
                   DV1=Chresults[1]+DV1;
                   DV2=Chresults[2]+DV2;
                   DV3=Chresults[3]+DV3;
                   if (P8IN & 0x08) 
                   {
                     //DV0=Chresults[0]+DV0;         // P8.3 high p8.3 Input Calibration
                     D1IR=DV0;
                     D2IR=DV1;
                     D3IR=DV2;
                     D4IR=DV3;
                     Rval = CTSD16MEM0;
                     res0++;
                   }
                   //�������
                   if (P8IN & 0x04) 
                   {              // P8.2 high  p8.2 Input LED color��R or IR
                     D1R=DV0;
                     D2R=DV1;
                     D3R=DV2;
                     D4R=DV3;
                     IRval = CTSD16MEM0;
                   } 
                   else //���������
                   {
                     DV0=Chresults[0]+DV0;         // P8.3 high
                     D1IR=DV0;
                     D2IR=DV1;
                     D3IR=DV2;
                     D4IR=DV3;
                   }
                   //if (index == 3) {DV2=Chresults[2]+DV2;}
                   /*
                   if (index >= Num_of_Channels) {
                      index = 0;                        // SET BREAKPOINT HERE
                      res0++;
                      CTSD16INCTL0 = CTSD16INCH_9;      // Reset input to CH0
                    } else {
                      CTSD16INCTL0 += 2;                // Increment CH count by 1
                    }
                   */
                   __bic_SR_register_on_exit(LPM0_bits); // Wake up  Exit LPM0  �˳��͹���ģʽ
                   break;
        default: break;
    }
}

// Echo back RXed character, confirm TX buffer is ready first
//����RX����
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR __interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!       //������ʾ��Ϣ
#endif
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    
    if ( UCA0RXBUF == 0x13) //0001 0011
    {
      auto1 = 1;
    } 
    else 
    {
      if (UCA0RXBUF == 0x17) //0001 0111
      {
        auto1 = 0;
      }
    }
    while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
    UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
    __no_operation();                   // For debugger
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

void Port_Mapping(void)
{
  // Disable Interrupts before altering Port Mapping registers
  __disable_interrupt();     
  // Enable Write-access to modify port mapping registers
  PMAPPWD = 0x02D52;     // 0010 1101 0101 0010   PMAPKEY�˿�д������Կ��ʼ�ն�ȡ 096A5h������д�� 02D52h ���ܶԶ˿�ӳ��Ĵ�������д���ʡ�             
  
  #ifdef PORT_MAP_RECFG                    
  // Allow reconfiguration during runtime
  PMAPCTL = PMAPRECFG;           //�˿�ӳ����ƼĴ���    �˿�ӳ�������ÿ���λ 0b = ֻ��������һ�� 1b = ����˿�ӳ��������        
  #endif  
  
  P2MAP0 = PM_UCA0TXD;  //USCI_A0 UART TXD���� USCI ���Ƶķ��� - �����
  P2MAP1 = PM_UCA0RXD;  //USCI_A0 UART RXD���� USCI ���Ƶķ��� - ���룩

  // Disable Write-Access to modify port mapping registers
  PMAPPWD = 0;                      //д0������                       
  #ifdef PORT_MAP_EINT
  __enable_interrupt();                     // Re-enable all interrupts
  #endif  
}

void delay(void)      //��ʱ����  156us��
{
    volatile unsigned int i = 260;           // Was 50, not enough
    do{
        i--;
    }while (i != 0);
}
