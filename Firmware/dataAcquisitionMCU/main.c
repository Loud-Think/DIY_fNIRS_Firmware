//
//        MSP430F5438A数据采集：与 MSP430FG6626 微控制器和蓝牙片上系统进行通信，负责光电二极管信号的模数转换，在定时器信号的引导下与LED脉冲同步。
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |            XT2IN|-
//            |                 | HF XTAL (18MHz)
//            |           XT2OUT|-
//            |                 |

//            |      (97)P6.0/A0|<----------------- NV0  ADC的模拟输入 A0
//            |      (98)P6.1/A1|<----------------- NV1  ADC的模拟输入 A1
//            |      (99)P6.2/A2|<----------------- NV2  ADC的模拟输入 A2
//            |     (100)P6.3/A3|<----------------- NV3  ADC的模拟输入 A3
//            |       (1)P6.4/A4|<----------------- Batt

//            |      (17)   P1.0|-----------------> LED 1 RED (not used)
//            |      (18)   P1.1|-----------------> LED 2 IR (used as a variable)  infrared  红外线

//            |      (21)   P1.4|-----------------> Timer


//            |      (25)   P2.0|-----------------> Output (not used)
//            |      (26)   P2.1|-----------------> Output (not used)

//            |      (30)   P2.5|<----------------- Timer input

//            |(39) P3.4/UCA0TXD|-----------------> 19200 - 8N1
//            |(40) P3.5/UCA0RXD|<----------------- 19200 - 8N1


//            |      (50)   P4.7|-----------------> Output ((LED color control to Intensity board to 60, 8.2)  对led光强度板的颜色控制
//            |      (51)   P5.4|-----------------> Output (calibration control to 61, 8.3)   校准控制
//            |      (75)   P9.7|-----------------> Output
//            |      (76)  P10.0|-----------------> Output
//             -----------------
//*****************************************************************************

#include <msp430x54x.h>

unsigned int NV0IR = 0;                           // New value新值
long DV0IR = 0;                                   // Acc value累计值
unsigned int NV1IR = 0;                           // New value
long DV1IR = 0;                                   // Acc value
unsigned int NV2IR = 0;                           // New value
long DV2IR = 0;                                   // Acc value
unsigned int NV3IR = 0;                           // New value
long DV3IR = 0;                                   // Acc value

unsigned int NV0 = 0;                           // New value新值    (pin97)P6.0/A0
long DV0 = 0;                                   // Acc value累计值
unsigned int NV1 = 0;                           // New value    (pin98)P6.1/A1
long DV1 = 0;                                   // Acc value
unsigned int NV2 = 0;                           // New value    (pin99)P6.2/A2
long DV2 = 0;                                   // Acc value
unsigned int NV3 = 0;                           // New value    (pin100)P6.3/A3
long DV3 = 0;                                   // Acc value
unsigned int NV4 = 0;                           // New value
long DV4 = 0;                                   // Acc value
unsigned int NV5 = 0;                           // New value
long DV5 = 0;                                   // Acc value
long res = 0;                                   // Resolution
//long resa[80] = {0};                          // Resolution
unsigned int dummy = 0;
//unsigned int tcount = 0;
long p_array[12] = {0};        //？？？
long q_array[12] = {0};        //？？？
unsigned int acount = 0;
unsigned int bcount = 0;
unsigned int Pacount = 0;
unsigned int Pbcount = 0;
long peak0 = 10000000;
long peak0b = 10000000;
long peak1 = 0;
long peak1b = 0;
long IRdiff = 0;
long Diffa = 0;
long Diffb = 0;

//long Patest[30] = {0};
//long Pbtest[30] = {0};

long D1R = 0;     //红光检测值
long D2R = 0;
long D3R = 0;
long D4R = 0;
long D1IR = 0;    //红外检测值
long D2IR = 0;
long D3IR = 0;
long D4IR = 0;
long DV0a = 0;   //？？？
long DV0b = 0;   //？？？
long D01 = 0;    
long D02 = 0;     
long PV02 = 0;    //？？？
long PV01 = 0;    //？？？
long PV01b = 0;   //？？？
unsigned int ucounta = 0;
unsigned int dcounta = 0;
unsigned int ucountb = 0;
unsigned int dcountb = 0;
unsigned int readya = 0;
unsigned int readyb = 0;
unsigned int dummy2 = 0;
unsigned int start = 0;
unsigned int calcount = 0;     //calibration count校准次数

long margin = 1000;
long bias = 0;
long bias_q = 0;

void delay(void);                            //延时函数
void SetVcoreUp (unsigned int level);       //设置核心电压函数

/*
UCS模块最多包括五个时钟源：
XT1CLK：低频或高频振荡器
VLOCLK：内置超低功耗低频振荡器，典型频率为10 kHz
REFOCLK：内部修剪过的低频振荡器  具有32768赫兹的典型频率，可用作进入锁相环的时钟参考
DCOCLK：可由FLL稳定的   内部数字控制振荡器(DCO)
XT2CLK：可选高频振荡器  XT2CLK可用作FLL的时钟基准
*/

int main( void )
{

  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  SetVcoreUp(PMMCOREV_1);                     // 设置核心电压  00b = V(CORE) level0   01b = V(CORE) level1    10b = V(CORE) level2    11b = V(CORE) level3 
  SetVcoreUp(PMMCOREV_2);                     // Set VCore to 1.8MHz for 20MHz   10b = V(CORE) level 2
  volatile unsigned int i;                  //定义无符号变量i
 
  P6SEL |= 0x01F;                           // Enable A/D channel input ADC的模拟输入  0001 1111 P6.0-P6.4   0b = I/O function is selected选择I/O功能   1b = Peripheral module function is selected选择外设模块功能
  P5SEL |= 0x0C;                            // Port select XT2     0000 1100  P5.2-P5.3  晶振端口选择

  P1DIR = 0x03;                             // P1.0, P1.1 to output direction    0000 0011   端口方向寄存器0b = Port configured as input  1b = Port configured as output    端口在原理图没用上？
  P1OUT = 0x01;                             // P1.0 RED on, 1.1 off IR, 1.0 is red    端口输出寄存器   端口在原理图没用上？
 
 
  UCSCTL6 |= XT2DRIVE0_H + XT2DRIVE1_H;     //00b=最低电流消耗，XT2振荡器的工作范围是4 MHz到8 MHz。 01B=驱动强度增加的XT2振荡器，XT2振荡器的工作范围是8 MHz到16 MHz。 
  UCSCTL6 &= ~XT2OFF;                       // Enable XT2       0b = XT2         统一时钟系统控制寄存器6
  UCSCTL3 |= SELREF_2;                      // FLLref = REFO    010b = REFOCLK   FLL参考选择，这些位选择FLL参考时钟源。 统一时钟系统控制寄存器3
  UCSCTL4 |= SELA_2;                        // ACLK=REFO        010b = REFOCLK   Selects the ACLK source   统一时钟系统控制寄存器4

  // Loop until XT2 & DCO stabilize
 

  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);  // XT2振荡器故障标志 + XT1振荡器故障标志(低频模式) + DCO故障标志
                                                 // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                           // Clear fault flags   中断标志寄存器1    振荡器故障中断标志
    for(i=0;i<0xFFFF;i++);                       // Delay for Osc to stabilize   i<65535
  }
  while (SFRIFG1&OFIFG);                        // Test oscillator fault flag是否为1    振荡器故障中断标志


  UCSCTL4 |= SELS_5 + SELM_5;                // SMCLK=XT2,MCLK=XT2   101b=XT2CLK(如果可用)，否则为DCOCLKDIV
  //UCSCTL4 |= SELS_3 + SELM_3;              // SMCLK=DCO,MCLK=DCO
 
  P2DIR |= 0x03;                             // P2.0 & P2.1 outputs设为输出   0000 0011
  P2OUT = 0x00;                              // P2.0 & P2.1 low    0b=输出低电平  1b=输出高电平   
  
 
  P2IES = 0x00;                              // Edge int., low to high  端口2中断边沿选择寄存器  0b = P2IFG flag is set with a low-to-high transition.1b = P2IFG flag is set with a high-to-low transition.
//  P2IE = 0x20;                             // Enable P2.5 int.    0010 0000
  P2IFG = 0x00;                              // 端口P2中断标志  0b = No interrupt is pending
 
////以上是时钟 中断 初始化配置
  
  
  ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_9;  // Turn on ADC12, extend sampling time，ADC12_A控制寄存器0，9d=1001b = 384 ADC12CLK cycles
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_3;        // Use sampling timer, repeated sequence
                                             //ADC12_A采样保持脉冲模式选择1b = SAMPCON signal is sourced from the sampling timer  SAMPCON信号来自采样定时器。
                                             //ADC12_A转换序列模式选择11b = Repeat-sequence-of-channels重复通道序列
  ADC12CTL1 |= (ADC12SSEL1);                 //ADC12_A时钟源选择，01b = ACLK    10b = MCLK   11b = SMCLK
  ADC12CTL1 |= (ADC12SSEL0);                 //ADC12_A时钟源选择，00b = ADC12OSC (MODCLK)

  ADC12MCTL0 = ADC12INCH_0;                 // ref+=Vref+, channel = A0，ADC12_A转换存储器控制寄存器  输入通道选择  0000b = A0
  ADC12MCTL1 = ADC12INCH_1;                 // ref+=Vref+, channel = A1    0001b = A1  0010b = A2   0011b = A3   0100b = A4
  ADC12MCTL2 = ADC12INCH_2;                 // ref+=Vref+, channel = A2    1000b = VeREF+    1001b = VREF-/VeREF-
  ADC12MCTL3 = ADC12INCH_3;                 // ref+=Vref+, channel = A3
  ADC12MCTL4 = ADC12INCH_4+ADC12EOS;        // ref+=Vref+, channel = A4，   ADC12EOS：1b = End of sequence
 
  ADC12IE = 0x10;                           // Enable ADC12IFG.4  This bit enables or disables the interrupt request for the ADC12IFG4 bit.
  ADC12CTL0 |= ADC12ENC;                    // Enable  ADC12_A enable conversion
  ADC12CTL0 |= ADC12SC;                     // Start conversions
  
////以上是ADC设置
  
// configure UART 19200  蓝牙配置
// 19200 from 18MHz

  P3SEL = 0x30;                             // P3.4,P3.5 = USCI_A0 TXD/RXD    1b = Peripheral module function is selected   0011 0000
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**，1b = Enabled. USCI logic held in reset state.111 101 011 000
 
  UCA0CTL1 |= UCSSEL_3;                     // SMCLK   11b = SMCLK   USCI时钟源选择
  UCA0BR0 = 0xA9;                           // 18MHz 19200   USCI_AX波特率控制寄存器0  波特率发生器时钟预分频器设置的低字节。 1010 1001  参考手册搜索UCBRx有波特率表
  UCA0BR1 = 0x03;                           // 18MHz 19200   USCI_AX波特率控制寄存器1  波特率发生器时钟预分频器设置的高字节。 0000 0011
  UCA0MCTL |= UCBRF_9 + UCBRS_4;            // Modulation    UCBRSx=4:第二调制级选择。 UCBRFx=9:第一个调制级选择。(18MHz)  USCI_Ax调制控制寄存器  9h=1001b  4h=0100b
 
  /*
  UCA0CTL1 |= UCSSEL_3;                     // SMCLK   11b = SMCLK   USCI时钟源选择
  UCA0BR0 = 117;                            // 1MHz 9600 (see User's Guide)  0111 0101
  UCA0BR1 = 0;                              // 1MHz 9600
  UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,  
   */                                       // over sampling
 
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**  110 101 010 000
  UCA0IE &= ~UCRXIE;                        // Disble USCI_A0 RX interrupt
 
  P4DIR |= 0x080;                           // P4.7 output    1000 0000     TB0 时钟输入 SMCLK 输出
  P4OUT = 0x00;                             // P4.7 low       0000 0000
  P5DIR |= 0x010;                           // P5.4 output    0001 0000     从机输出，主机输入 – USCI_B1 SPI 模式    I2C 时钟 – USCI_B1 I2C 模式
  P5OUT = 0x00;                             // P5.4 low (no auto intensity)
  P9DIR |= 0x080;                           // P9.7 output    1000 0000    在原理图没用上？
  P9OUT = 0x00;                             // P9.7 low       
  P10DIR |= 0x01;                           // P10.0 output   0000 0001    在原理图没用上？
  P10OUT = 0x00;                            // P10.0 low      0000 0000
 
  P2IE = 0x20;                              // Enable P2.5 int. 0010 0000
 
  P4OUT |= 0x080;                           // P4.7 hi        1000 0000
 
    __bis_SR_register(LPM0_bits + GIE);     // LPM0 + Enable global inters   进入低功耗模式   Enter LPM0 with interrupts enabled.    __bis_SR_register应该是个.h文件

}

// Port2 ISR中断服务程序
#pragma vector=PORT2_VECTOR                //用关键字__interrupt来定义一个中断函数。用#pragma vector来表示一个中断向量
__interrupt void PORT2_ISR (void)
{
  _BIC_SR(LPM0_bits);                       // Clear CPUOFF bit from 0(SR)  Exit LPM0 退出低功耗模式
  P2IFG = 0x00;                             //  端口2中断标志寄存器
  P2IE &= ~0x20;                           //0010 0000   端口2中断使能启用寄存器
  P2IES ^= 0x20; // 323 to 324              //0010 0000   端口2中断边沿选择寄存器  1b = P2IFG flag is set with a high-to-low transition. ^ 按位异或
  //P2IES = 0x20; // 773 to 774             
 
  //tcount++;
 
  if (P1OUT == 0x01)        //P1.0 high    但是原理图都没用到p1.0，怎么控制？
  {
    P4OUT = 0x80;                          // P4.7 high use for LED R color controls  1000 0000
  } 
  else 
    {
      P4OUT = 0x00;                           // P4.7 low for LED IR
    }
 
  calcount++;
 
  if (calcount == 200000) 
  {
    //P4OUT &= ~0x80;                       // P4.7 low    1000 0000
    P5OUT &= ~0x10;                         // P5.4 low no auto intensity不自动校准  p5.4：Output (calibration control to 61, p8.3)   输出校准控制   0001 0000
  } 
  else 
    {
      if (calcount > 65000) 
      {
        calcount = 201;
      }
    }
 
 
  if (P1OUT == 0x01)       //P1.0 high    但是原理图都没用到p1.0，怎么控制？
  {
   
    D1R = DV0;    //D1R是什么变量？ D1R是红光检测值？    DV0是Acc value累计值
    D2R = DV1;    
    D3R = DV2;
    D4R = DV3;
   
    if (acount < 12) 
    {
      p_array[acount] = DV0;
      if (readya > 0) 
      {
        readya = 2;
        if (acount == 11) 
        {
          bias = bias + DV0 - p_array[0];
        } 
        else 
        {
          bias = bias + DV0 - p_array[acount + 1];
        }
      }
    } 
    else 
    {
      acount = 0;
      if (readya == 0) 
      {
        for (dummy2=2;dummy2<12;dummy2++) 
        {
        bias = p_array[dummy2] + bias;
        }
        readya = 1;
        PV01 = DV0 - bias/10;
      }
      bias = bias + DV0 - p_array[1];
      p_array[acount] = DV0;
    }
   
    //bias = 0;
   
    if (readya > 1) 
    {
      DV0a = DV0 - bias/10;
        if (dcounta > 0) 
        {
          if (DV0a > PV01 + margin) 
          {
            ucounta++;
            if (ucounta >0) 
            {
              peak1 = DV0a;
            }
          }
        }
        if (DV0a < PV01 - margin) 
        {
          dcounta++;
          if (ucounta > 0) 
          {

            if (ucounta > 0) 
            {
              Diffa = peak1 - peak0;
              //Patest[Pacount] = Diffa;
              Pacount++;
              _NOP();       //空操作
            }
            dcounta = 1;
            ucounta = 0;
            _NOP();
          }
          peak0 = DV0a;
        } 
          else 
          {
            // for now, nothing to be safe
          }
        PV01 = DV0a;
       
    }
    acount++;
  }
  
 //重复上述过程
  if (P1OUT == 0x02)         //0000 0010  p1.1   原理图没用上.//P1.1 high    但是原理图都没用到p1.1，怎么控制？
  {
   
    D1IR = DV0IR;      //D1R是红光检测值？    DV0是Acc value累计值
    D2IR = DV1IR;
    D3IR = DV2IR;
    D4IR = DV3IR;
   
    if (bcount < 12) 
    {
      q_array[bcount] = DV0;
      if (readyb > 0) 
      {
        readyb = 2;
        if (bcount == 11) 
        {
          bias_q = bias_q + DV0 - q_array[0];
        } 
        else 
        {
          bias_q = bias_q + DV0 - q_array[bcount + 1];
        }
      }
    } 
    else 
    {
      bcount = 0;
      if (readyb == 0) 
      {
        for (dummy2=2;dummy2<12;dummy2++) 
        {
        bias_q = q_array[dummy2] + bias_q;
        }
        readyb = 1;
        PV01b = DV0 - bias_q/10;
      }
      bias_q = bias_q + DV0 - q_array[1];
      q_array[bcount] = DV0;
    }
   
    //bias_q = 0;
   
    if (readyb > 1) 
    {
      DV0b = DV0 - bias_q/10;
        if (dcountb > 0) 
        {
          if (DV0b > PV01b + margin) 
          {
            ucountb++;
            if (ucountb >0) 
            {
              peak1b = DV0b;
            }
          }
        }
        if (DV0b < PV01b - margin) 
        {
          dcountb++;
          if (ucountb > 0) 
          {
            if (ucountb > 0)         //写了两遍？
            {
              Diffb = peak1b - peak0b;    //峰值的不同 = 两峰值之差
              //Pbtest[Pbcount] = Diffb;
              Pbcount++;
              _NOP();
            }
            dcountb = 1;
            ucountb = 0;
            _NOP();
          }
          peak0b = DV0b;
        } 
        else 
        {
          // for now, nothing to be safe
        }
        PV01b = DV0b;
    }
    bcount++;
   
  }
 
/*

  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?   0b = No interrupt pending
    UCA0TXBUF = DV0;                        // TXBUF0    // Acc value累计值
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = DV0 >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = DV0 >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = DV0 >> 24;
    UCA0TXBUF = dummy;
   
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = DV2;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = DV2 >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = DV2 >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = DV2 >> 24;
    UCA0TXBUF = dummy;
*/
 
    
  if (P2IES == 0x20)       //端口 2 中断边沿选择寄存器   0010 0000  p2.5 high     p2.5：Timer input来自msp2001
  {
   
    P3SEL = 0x30;                            // 开始发送和接收数据 P3.4,p3.5 = USCI_A0 TXD/RXD 蓝牙  0011 0000
 
    //红光数据
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = D1R;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = D1R >> 8;                        //dummy是中间值？
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
   
    //红外数据 
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
   
   //分辨率数据
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready? Resolution 18
    UCA0TXBUF = res;                        // TXBUF0
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = res >> 8;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = res >> 16;
    UCA0TXBUF = dummy;
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    dummy = res >> 24;
    UCA0TXBUF = dummy;

    //13 17 19 00啥意思？？？
  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?   
    UCA0TXBUF = 13;                         // TXBUF0      0000 1101
  while (!(UCA0IFG&UCTXIFG));              // TX buffer ready?
    UCA0TXBUF = 17;                        // TXBUF0       0001 0001
  while (!(UCA0IFG&UCTXIFG));              // TX buffer ready?
    UCA0TXBUF = 19;                        // TXBUF0       0001 0011

  while (!(UCA0IFG&UCTXIFG));               // TX buffer ready?
    UCA0TXBUF = 00;                         // TXBUF0
  while (!(UCA0IFG&UCTXIFG));              // TX buffer ready?
    UCA0TXBUF = 19;                        // TXBUF0      0001 0011
   
  P3SEL &= ~0x30;                         // 停止发送和接收数据  P3.4,p3.5 = USCI_A0 TXD/RXD    0011 0000
   
  }

  DV0=0;
  DV1=0;
  DV2=0;
  DV3=0;
  DV4=0;
  DV5=0;
  DV0IR=0;
  DV1IR=0;
  DV2IR=0;
  DV3IR=0;
  /*
  resa[tcount]=res;
  if (tcount > 80) 
  {
  tcount = 0;
  }
  */
  res=0;
 
  if (P1OUT == 0x01)       //p1.0    原理图没用到？
  {
    P1OUT = 0x02;          //p1.1原理图没用到？
    P2OUT = 0x02;          //p2.1原理图没用到？
    //P4OUT = 0x080;                       // P4.7 high use for LED R color controls   1000 0000
  } 
  else 
  {
    P1OUT = 0x01;          //p1.0
    P2OUT = 0x01;          //p2.0
    //P4OUT = 0x00;                        // P4.7 low for LED IR
  }
 
  P2IE = 0x20;                             // Enable P2.5 int.  p2.5：Timer input   0010 0000

  ADC12IE = 0x10;                         // Enable ADC12IFG.4是ADC12MEM4中断标志。当 ADC12MEM4 加载转换结果时，该位置位。如果访问 ADC12MEM4，则该位复位，或者可以用软件复位。   0001 0000
  ADC12CTL0 |= ADC12ENC;                  // Enable conversions使能转换
  ADC12CTL0 |= ADC12SC;                   // Start conversions开始转换
   
  _NOP();                                 //空操作
}

#pragma vector=ADC12_VECTOR               //#pragma vector提供中断函数入口地址  
__interrupt void ADC12ISR (void)         //使用 _interrupt void _Name_Function来定义中断函数名称
{                                         //用关键字__interrupt来定义一个中断函数，用#pragma vector用来表示一个中断向量。
  switch(__even_in_range(ADC12IV,34))   //ADC12IV是ADC12_A 中断向量寄存器
  {
/*
00h = No interrupt pending
02h = Interrupt Source: ADC12MEMx overflow; Interrupt Flag: –; Interrupt
Priority: Highest
04h = Interrupt Source: Conversion time overflow; Interrupt Flag: –
06h = Interrupt Source: ADC12MEM0 interrupt flag; Interrupt Flag: ADC12IFG0
08h = Interrupt Source: ADC12MEM1 interrupt flag; Interrupt Flag: ADC12IFG1
0Ah = Interrupt Source: ADC12MEM2 interrupt flag; Interrupt Flag: ADC12IFG2
0Ch = Interrupt Source: ADC12MEM3 interrupt flag; Interrupt Flag: ADC12IFG3
*/
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6: break;                           // Vector  6:  ADC12IFG0
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14:
    if (P1OUT == 0x01) //p1.0 high
    {                                       // Red    0000 0001
    NV0 = ADC12MEM0;                        //ADC12_A转换存储寄存器0
    NV1 = ADC12MEM1;
    NV2 = ADC12MEM2;
    NV3 = ADC12MEM3;
    DV0 = NV0 + DV0;                       //DV0是Acc value累计值
    DV1 = NV1 + DV1;
    DV2 = NV2 + DV2;
    DV3 = NV3 + DV3;
    }
    NV4 = ADC12MEM4;
    NV5 = ADC12MEM5;
    if (P1OUT == 0x02) {                    // IR   0000 0010
    NV0IR = ADC12MEM0;
    NV1IR = ADC12MEM1;
    NV2IR = ADC12MEM2;
    NV3IR = ADC12MEM3;
    DV0IR = NV0IR + DV0IR;
    DV1IR = NV1IR + DV1IR;
    DV2IR = NV2IR + DV2IR;
    DV3IR = NV3IR + DV3IR;
    }
    res++;
   
    DV4 = NV4 + DV4;
    DV5 = NV5 + DV5;
   /*
0Eh = Interrupt Source: ADC12MEM4 interrupt flag; Interrupt Flag: ADC12IFG4
10h = Interrupt Source: ADC12MEM5 interrupt flag; Interrupt Flag: ADC12IFG5
12h = Interrupt Source: ADC12MEM6 interrupt flag; Interrupt Flag: ADC12IFG6
14h = Interrupt Source: ADC12MEM7 interrupt flag; Interrupt Flag: ADC12IFG7
16h = Interrupt Source: ADC12MEM8 interrupt flag; Interrupt Flag: ADC12IFG8
18h = Interrupt Source: ADC12MEM9 interrupt flag; Interrupt Flag: ADC12IFG9
1Ah = Interrupt Source: ADC12MEM10 interrupt flag; Interrupt Flag:
ADC12IFG10
1Ch = Interrupt Source: ADC12MEM11 interrupt flag; Interrupt Flag:
ADC12IFG11
1Eh = Interrupt Source: ADC12MEM12 interrupt flag; Interrupt Flag:
ADC12IFG12
20h = Interrupt Source: ADC12MEM13 interrupt flag; Interrupt Flag:
ADC12IFG13
22h = Interrupt Source: ADC12MEM14 interrupt flag; Interrupt Flag:
ADC12IFG14
24h = Interrupt Source: ADC12MEM15 interrupt flag; Interrupt Flag:
ADC12IFG15; Interrupt Priority: Lowest
*/
    break;                                  // Vector 14:  ADC12IFG4   0Eh
  case 16: break;                           // Vector 16:  ADC12IFG5   10h
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14    22h
  default: break;
  }  
}

void delay(void)      //延时33us
{
    volatile unsigned int i = 50;             // Was 40 and tested with 1 module
    do{
        i--;
    }while (i != 0);
}

void SetVcoreUp (unsigned int level)          //设置核心电压函数
{
  // Open PMM registers for write
  PMMCTL0_H = PMMPW_H;                         //MOV.B #PMMPW_H, &PMMCTL0_H ; Open PMM registers for write
  // Set SVS/SVM high side new level
  SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;  //电源电压监控器和监视器高端控制寄存器
  // Set SVM low side to new level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;     //电源电压监控和监控低端控制寄存器
  // Wait till SVM is settled
  while ((PMMIFG & SVSMLDLYIFG) == 0);         //电源管理模块中断标志寄存器
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);           //电源管理模块中断标志寄存器
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;               //核心电压*level
  // Wait till new level reached
  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);
  // Set SVS/SVM low side to new level
  SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;                             //电源管理模块控制寄存器0
}

//提高核心电压的C代码示例。注：一次更改一个电平的核心电压。
/*
参考手册示例代码
void SetVCoreUp (unsigned int level)
{
// Open PMM registers for write access
PMMCTL0_H = 0xA5;          //1010 0101
// Make sure no flags are set for iterative sequences
while ((PMMIFG & SVSMHDLYIFG) == 0);
while ((PMMIFG & SVSMLDLYIFG) == 0);
// Set SVS/SVM high side new level
SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
// Set SVM low side to new level
SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
// Wait till SVM is settled
while ((PMMIFG & SVSMLDLYIFG) == 0);
// Clear already set flags
PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
// Set VCore to new level
PMMCTL0_L = PMMCOREV0 * level;
// Wait till new level reached
if ((PMMIFG & SVMLIFG))
while ((PMMIFG & SVMLVLRIFG) == 0);
// Set SVS/SVM low side to new level
SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
// Lock PMM registers for write access
PMMCTL0_H = 0x00;
}
*/


