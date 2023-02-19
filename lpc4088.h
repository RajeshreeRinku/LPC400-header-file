#include<stdint.h>
#define __I volatile
#define __O volatile
#define __IO volatile
	
/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
/******  Cortex-M4 Processor Exceptions Numbers ***************************************************/
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,      /*!< 2 Non Maskable Interrupt                         */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt          */
  BusFault_IRQn                 = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                  */
  UsageFault_IRQn               = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                */
  SVCall_IRQn                   = -5,       /*!< 11 Cortex-M3 SV Call Interrupt                   */
  DebugMonitor_IRQn             = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt             */
  PendSV_IRQn                   = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt                   */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M3 System Tick Interrupt               */

/******  LPC177x_8x Specific Interrupt Numbers *******************************************************/
  WDT_IRQn                      = 0,        /*!< Watchdog Timer Interrupt                         */
  TIMER0_IRQn                   = 1,        /*!< Timer0 Interrupt                                 */
  TIMER1_IRQn                   = 2,        /*!< Timer1 Interrupt                                 */
  TIMER2_IRQn                   = 3,        /*!< Timer2 Interrupt                                 */
  TIMER3_IRQn                   = 4,        /*!< Timer3 Interrupt                                 */
  UART0_IRQn                    = 5,        /*!< UART0 Interrupt                                  */
  UART1_IRQn                    = 6,        /*!< UART1 Interrupt                                  */
  UART2_IRQn                    = 7,        /*!< UART2 Interrupt                                  */
  UART3_IRQn                    = 8,        /*!< UART3 Interrupt                                  */
  PWM1_IRQn                     = 9,        /*!< PWM1 Interrupt                                   */
  I2C0_IRQn                     = 10,       /*!< I2C0 Interrupt                                   */
  I2C1_IRQn                     = 11,       /*!< I2C1 Interrupt                                   */
  I2C2_IRQn                     = 12,       /*!< I2C2 Interrupt                                   */
  Reserved0_IRQn                = 13,       /*!< Reserved                                         */
  SSP0_IRQn                     = 14,       /*!< SSP0 Interrupt                                   */
  SSP1_IRQn                     = 15,       /*!< SSP1 Interrupt                                   */
  PLL0_IRQn                     = 16,       /*!< PLL0 Lock (Main PLL) Interrupt                   */
  RTC_IRQn                      = 17,       /*!< Real Time Clock Interrupt                        */
  EINT0_IRQn                    = 18,       /*!< External Interrupt 0 Interrupt                   */
  EINT1_IRQn                    = 19,       /*!< External Interrupt 1 Interrupt                   */
  EINT2_IRQn                    = 20,       /*!< External Interrupt 2 Interrupt                   */
  EINT3_IRQn                    = 21,       /*!< External Interrupt 3 Interrupt                   */
  ADC_IRQn                      = 22,       /*!< A/D Converter Interrupt                          */
  BOD_IRQn                      = 23,       /*!< Brown-Out Detect Interrupt                       */
  USB_IRQn                      = 24,       /*!< USB Interrupt                                    */
  CAN_IRQn                      = 25,       /*!< CAN Interrupt                                    */
  DMA_IRQn                      = 26,       /*!< General Purpose DMA Interrupt                    */
  I2S_IRQn                      = 27,       /*!< I2S Interrupt                                    */
  ENET_IRQn                     = 28,       /*!< Ethernet Interrupt                               */
  MCI_IRQn                      = 29,       /*!< SD/MMC card I/F Interrupt                        */
  MCPWM_IRQn                    = 30,       /*!< Motor Control PWM Interrupt                      */
  QEI_IRQn                      = 31,       /*!< Quadrature Encoder Interface Interrupt           */
  PLL1_IRQn                     = 32,       /*!< PLL1 Lock (USB PLL) Interrupt                    */
  USBActivity_IRQn              = 33,       /*!< USB Activity interrupt                           */
  CANActivity_IRQn              = 34,       /*!< CAN Activity interrupt                           */
  UART4_IRQn                    = 35,       /*!< UART4 Interrupt                                  */
  SSP2_IRQn                     = 36,       /*!< SSP2 Interrupt                                   */
  LCD_IRQn                      = 37,       /*!< LCD Interrupt                                    */
  GPIO_IRQn                     = 38,       /*!< GPIO Interrupt                                   */
  PWM0_IRQn                     =  39,              /*!<  39  PWM0                                                             */
  EEPROM_IRQn                   =  40,              /*!<  40  EEPROM                                                           */
  CMP0_IRQn                     =  41,              /*!<  41  CMP0                                                             */
  CMP1_IRQn                     =  42               /*!<  42  CMP1                                                             */
} IRQn_Type;

/************************************************ SYSTEM AND CLOCK CONTROL *************************************************************/

typedef struct
{
 __IO uint32_t PLLCON0;
 __IO uint32_t PLLCFG0;
 __I uint32_t PLLSTAT0;
 __O uint32_t PLLFEED0;
 __IO uint8_t RESRVED0[0x13];
	
 __IO uint32_t PLLCON1;
 __IO uint32_t PLLCFG1;
 __I uint32_t PLLSTAT1;
 __O uint32_t PLLFEED1;
 __IO uint8_t RESRVED1[0x13];
	
	__IO uint32_t PCON;
	__IO uint32_t PCONP;
	__IO uint32_t PCONP1;
	__IO uint8_t RESRVED2[0x37];
	__IO uint32_t EMCCLKSEL;                 /*External Memory Controller Clock Selection register*/
	__IO uint32_t CCLKSEL;                   /*CPU Clock Selection register*/
	__IO uint32_t USBCLKSEL;                 /*USB Clock Selection register*/
	__IO uint32_t CLKSRCSEL;                 /*Clock Source Select Register*/
	__IO uint32_t CANSLEEPCLR;               /*Allows clearing the current CAN channel sleep state as well as reading back that state*/
  __IO uint32_t CANWAKEFLAGS;              /*Indicates the wake-up state of the CAN channels.*/
	__IO uint8_t RESERVED5[0x28];
	__IO uint32_t EXTINT;                    /*External Interrupt Flag Register*/
	__IO uint32_t RESERVED6;
	__IO uint32_t EXTMODE;                   /*External Interrupt Mode register*/
	__IO uint32_t EXTPOLAR;                  /*External Interrupt Polarity Register*/
	__IO uint8_t RESERVED7[0x30];
	__IO uint32_t RSID;                      /*Reset Source Identification Register*/
	__IO uint8_t RESERVED8[0x1C];
	__IO uint32_t SCS;                       /*System Control and Status*/
	__IO uint8_t RESREVED9[4];
	__IO uint32_t PCLKSEL;                   /*Peripheral Clock Selection register*/
	__IO uint8_t RESERVED10[4];
	__IO uint32_t PBOOST;                    /*Power boost register*/
	__IO uint32_t SPIFICLKSEL;               /*SPIFI Clock Selection register*/
	__IO uint32_t LCD_CFG;                   /*LCD Clock configuration register*/
	__IO uint8_t RESERVED11[4];
	__IO uint32_t USBINTST;                  /*USB Interrupt Status*/
	__IO uint32_t DMACREQSEL;                /*Selects between alternative requests on DMA channels 0 through 7 and 10 through 15.*/
  __IO uint32_t CLKOUTCFG;                 /*Clock Output Configuration register*/
	__IO uint32_t RESERVED12;
	__IO uint32_t RSTCON0;                   /*Individual peripheral reset control bits*/
	__IO uint32_t RESERVED13;
	__IO uint32_t RSTCON1;                   /*Individual peripheral reset control bits*/
	__IO uint8_t RESERVED14[0x8];
	__IO uint32_t EMCDLYCTL;                 /*Values for the 4 programmable delays associated with SDRAM operation.*/
  __IO uint32_t RESRVED15;
	__IO uint32_t EMCCAL;                    /*Controls the calibration counter for programmable delays and returns the result value.*/

}SYSTEM_CONTROL_TYPEDEF;

typedef struct
{
__IO uint32_t FLASHCFG;
}FLASH_CONTROL_TYPEDEF;

typedef struct
{
__IO uint32_t ISER0;                      /*Interrupt Set-Enable Registers. These registers allow enabling*/
__IO uint32_t ISER1;                      /*interrupts and reading back the interrupt enables for specific peripheral functions*/

}INTERRUPT_SET_ENABLE_TYPEDEF;

typedef struct
{
	__IO uint32_t ICER0;                    /*Interrupt Clear-Enable Registers. These registers allow Interrupt Clear-Enable Registers. These registers allow
                                            disabling interrupts and reading back the interrupt enables for specific peripheral functions.*/
	__IO uint32_t ICER1;
}INTERRUPT_CLEAR_ENABLE_TYPEDEF;

typedef struct
{
 __IO uint32_t ISPR0;                    /*Interrupt Set-Pending Registers. These registers allow changing the interrupt state to pending and reading back the
                                           interrupt pending state for specific peripheral functions.*/
 __IO uint32_t ISPR1;
}INTERRUPT_SET_PENDING_TYPEDEF;

typedef struct
{
 __IO uint32_t ICPR0;                   /*Interrupt Clear-Pending Registers. These registers allow changing the interrupt state to not pending and reading back
                                          the interrupt pending state for specific peripheral functions.*/
 __IO uint32_t ICPR1;
}INTERRUPT_CLEAR_PENDING_TYPEDEF;

typedef struct
{
 __I uint32_t IABR0;                   /*Interrupt Active Bit Registers. These registers allow reading the current interrupt active state for specific peripheral functions.*/
 __I uint32_t IABR1;
}INTERRUPT_ACTIVE_BIT_TYPEDEF;

typedef struct
{
  __IO uint32_t IPR0;
	__IO uint32_t IPR1;
	__IO uint32_t IPR2;
	__IO uint32_t IPR3;
	__IO uint32_t IPR4;
	__IO uint32_t IPR5;
	__IO uint32_t IPR6;
	__IO uint32_t IPR7;
	__IO uint32_t IPR8;
	__IO uint32_t IPR9;
	__IO uint32_t IPR10;
}INTERRUPT_POLARITY_TYPEDEF;

typedef struct
{
 __O uint32_t STIR;                   /*Software Trigger Interrupt Register. This register allows software to generate an interrupt.*/
}SOFTWARE_TRIGGER_INTERRUPT_TYPEDEF;

typedef struct
{
  __IO uint32_t IOCON_P0_00;
  __IO uint32_t IOCON_P0_01;
	__IO uint32_t IOCON_P0_02;
	__IO uint32_t IOCON_P0_03;
	__IO uint32_t IOCON_P0_04;
	__IO uint32_t IOCON_P0_05;
	__IO uint32_t IOCON_P0_06;
	__IO uint32_t IOCON_P0_07;
	__IO uint32_t IOCON_P0_08;
	__IO uint32_t IOCON_P0_09;
	__IO uint32_t IOCON_P0_10;
	__IO uint32_t IOCON_P0_11;
	__IO uint32_t IOCON_P0_12;
	__IO uint32_t IOCON_P0_13;
	__IO uint32_t IOCON_P0_14;
	__IO uint32_t IOCON_P0_15;
	__IO uint32_t IOCON_P0_16;
	__IO uint32_t IOCON_P0_17;
	__IO uint32_t IOCON_P0_18;
	__IO uint32_t IOCON_P0_19;
	__IO uint32_t IOCON_P0_20;
	__IO uint32_t IOCON_P0_21;
	__IO uint32_t IOCON_P0_22;
	__IO uint32_t IOCON_P0_23;
	__IO uint32_t IOCON_P0_24;
	__IO uint32_t IOCON_P0_25;
	__IO uint32_t IOCON_P0_26;
	__IO uint32_t IOCON_P0_27;
	__IO uint32_t IOCON_P0_28;
	__IO uint32_t IOCON_P0_29;
	__IO uint32_t IOCON_P0_30;
	__IO uint32_t IOCON_P0_31;
	
	__IO uint32_t IOCON_P1_00;
  __IO uint32_t IOCON_P1_01;
	__IO uint32_t IOCON_P1_02;
	__IO uint32_t IOCON_P1_03;
	__IO uint32_t IOCON_P1_04;
	__IO uint32_t IOCON_P1_05;
	__IO uint32_t IOCON_P1_06;
	__IO uint32_t IOCON_P1_07;
	__IO uint32_t IOCON_P1_08;
	__IO uint32_t IOCON_P1_09;
	__IO uint32_t IOCON_P1_10;
	__IO uint32_t IOCON_P1_11;
	__IO uint32_t IOCON_P1_12;
	__IO uint32_t IOCON_P1_13;
	__IO uint32_t IOCON_P1_14;
	__IO uint32_t IOCON_P1_15;
	__IO uint32_t IOCON_P1_16;
	__IO uint32_t IOCON_P1_17;
	__IO uint32_t IOCON_P1_18;
	__IO uint32_t IOCON_P1_19;
	__IO uint32_t IOCON_P1_20;
	__IO uint32_t IOCON_P1_21;
	__IO uint32_t IOCON_P1_22;
	__IO uint32_t IOCON_P1_23;
	__IO uint32_t IOCON_P1_24;
	__IO uint32_t IOCON_P1_25;
	__IO uint32_t IOCON_P1_26;
	__IO uint32_t IOCON_P1_27;
	__IO uint32_t IOCON_P1_28;
	__IO uint32_t IOCON_P1_29;
	__IO uint32_t IOCON_P1_30;
	__IO uint32_t IOCON_P1_31;
	
	__IO uint32_t IOCON_P2_00;
  __IO uint32_t IOCON_P2_01;
	__IO uint32_t IOCON_P2_02;
	__IO uint32_t IOCON_P2_03;
	__IO uint32_t IOCON_P2_04;
	__IO uint32_t IOCON_P2_05;
	__IO uint32_t IOCON_P2_06;
	__IO uint32_t IOCON_P2_07;
	__IO uint32_t IOCON_P2_08;
	__IO uint32_t IOCON_P2_09;
	__IO uint32_t IOCON_P2_10;
	__IO uint32_t IOCON_P2_11;
	__IO uint32_t IOCON_P2_12;
	__IO uint32_t IOCON_P2_13;
	__IO uint32_t IOCON_P2_14;
	__IO uint32_t IOCON_P2_15;
	__IO uint32_t IOCON_P2_16;
	__IO uint32_t IOCON_P2_17;
	__IO uint32_t IOCON_P2_18;
	__IO uint32_t IOCON_P2_19;
	__IO uint32_t IOCON_P2_20;
	__IO uint32_t IOCON_P2_21;
	__IO uint32_t IOCON_P2_22;
	__IO uint32_t IOCON_P2_23;
	__IO uint32_t IOCON_P2_24;
	__IO uint32_t IOCON_P2_25;
	__IO uint32_t IOCON_P2_26;
	__IO uint32_t IOCON_P2_27;
	__IO uint32_t IOCON_P2_28;
	__IO uint32_t IOCON_P2_29;
	__IO uint32_t IOCON_P2_30;
	__IO uint32_t IOCON_P2_31;
	
	__IO uint32_t IOCON_P3_00;
  __IO uint32_t IOCON_P3_01;
	__IO uint32_t IOCON_P3_02;
	__IO uint32_t IOCON_P3_03;
	__IO uint32_t IOCON_P3_04;
	__IO uint32_t IOCON_P3_05;
	__IO uint32_t IOCON_P3_06;
	__IO uint32_t IOCON_P3_07;
	__IO uint32_t IOCON_P3_08;
	__IO uint32_t IOCON_P3_09;
	__IO uint32_t IOCON_P3_10;
	__IO uint32_t IOCON_P3_11;
	__IO uint32_t IOCON_P3_12;
	__IO uint32_t IOCON_P3_13;
	__IO uint32_t IOCON_P3_14;
	__IO uint32_t IOCON_P3_15;
	__IO uint32_t IOCON_P3_16;
	__IO uint32_t IOCON_P3_17;
	__IO uint32_t IOCON_P3_18;
	__IO uint32_t IOCON_P3_19;
	__IO uint32_t IOCON_P3_20;
	__IO uint32_t IOCON_P3_21;
	__IO uint32_t IOCON_P3_22;
	__IO uint32_t IOCON_P3_23;
	__IO uint32_t IOCON_P3_24;
	__IO uint32_t IOCON_P3_25;
	__IO uint32_t IOCON_P3_26;
	__IO uint32_t IOCON_P3_27;
	__IO uint32_t IOCON_P3_28;
	__IO uint32_t IOCON_P3_29;
	__IO uint32_t IOCON_P3_30;
	__IO uint32_t IOCON_P3_31;
	
	__IO uint32_t IOCON_P4_00;
  __IO uint32_t IOCON_P4_01;
	__IO uint32_t IOCON_P4_02;
	__IO uint32_t IOCON_P4_03;
	__IO uint32_t IOCON_P4_04;
	__IO uint32_t IOCON_P4_05;
	__IO uint32_t IOCON_P4_06;
	__IO uint32_t IOCON_P4_07;
	__IO uint32_t IOCON_P4_08;
	__IO uint32_t IOCON_P4_09;
	__IO uint32_t IOCON_P4_10;
	__IO uint32_t IOCON_P4_11;
	__IO uint32_t IOCON_P4_12;
	__IO uint32_t IOCON_P4_13;
	__IO uint32_t IOCON_P4_14;
	__IO uint32_t IOCON_P4_15;
	__IO uint32_t IOCON_P4_16;
	__IO uint32_t IOCON_P4_17;
	__IO uint32_t IOCON_P4_18;
	__IO uint32_t IOCON_P4_19;
	__IO uint32_t IOCON_P4_20;
	__IO uint32_t IOCON_P4_21;
	__IO uint32_t IOCON_P4_22;
	__IO uint32_t IOCON_P4_23;
	__IO uint32_t IOCON_P4_24;
	__IO uint32_t IOCON_P4_25;
	__IO uint32_t IOCON_P4_26;
	__IO uint32_t IOCON_P4_27;
	__IO uint32_t IOCON_P4_28;
	__IO uint32_t IOCON_P4_29;
	__IO uint32_t IOCON_P4_30;
	__IO uint32_t IOCON_P4_31;
	
	__IO uint32_t IOCON_P5_00;
  __IO uint32_t IOCON_P5_01;
	__IO uint32_t IOCON_P5_02;
	__IO uint32_t IOCON_P5_03;
	__IO uint32_t IOCON_P5_04;
}IOCON_TYPEDEF;

typedef struct
{
 __IO uint32_t DIR0;                /*GPIO Port0 Direction control register.*/
 __IO uint8_t RESERVED0[0xC];
 __IO uint32_t MASK0;               /*Mask register for Port0.*/
 __IO uint32_t PIN0;                /*Port0 Pin value register using FIOMASK.*/
 __IO uint32_t SET0;                /*Port0 Output Set register using FIOMASK.*/
 __O uint32_t CLR0;                 /*Port0 Output Clear register using FIOMASK.*/
	
 __IO uint32_t DIR1;                /*GPIO Port1 Direction control register.*/
 __IO uint8_t RESERVED1[0xC];
 __IO uint32_t MASK1;               /*Mask register for Port1.*/
 __IO uint32_t PIN1;                /*Port1 Pin value register using FIOMASK.*/
 __IO uint32_t SET1;                /*Port1 Output Set register using FIOMASK.*/
 __O uint32_t CLR1;                 /*Port1 Output Clear register using FIOMASK.*/
	
 __IO uint32_t DIR2;                /*GPIO Port2 Direction control register.*/
 __IO uint8_t RESERVED2[0xC];
 __IO uint32_t MASK2;               /*Mask register for Port2.*/
 __IO uint32_t PIN2;                /*Port2 Pin value register using FIOMASK.*/
 __IO uint32_t SET2;                /*Port2 Output Set register using FIOMASK.*/
 __O uint32_t CLR2;                 /*Port2 Output Clear register using FIOMASK.*/
	
 __IO uint32_t DIR3;                /*GPIO Port3 Direction control register.*/
 __IO uint8_t RESERVED3[0xC];
 __IO uint32_t MASK3;               /*Mask register for Port3.*/
 __IO uint32_t PIN3;                /*Port3 Pin value register using FIOMASK.*/
 __IO uint32_t SET3;                /*Port3 Output Set register using FIOMASK.*/
 __O uint32_t CLR3;                 /*Port3 Output Clear register using FIOMASK.*/
 
 __IO uint32_t DIR4;                /*GPIO Port4 Direction control register.*/
 __IO uint8_t RESERVED4[0xC];
 __IO uint32_t MASK4;               /*Mask register for Port4.*/
 __IO uint32_t PIN4;                /*Port4 Pin value register using FIOMASK.*/
 __IO uint32_t SET4;                /*Port4 Output Set register using FIOMASK.*/
 __O uint32_t CLR4;                 /*Port4 Output Clear register using FIOMASK.*/
 
 __IO uint32_t DIR5;                /*GPIO Port5 Direction control register.*/
 __IO uint8_t RESERVED5[0xC];
 __IO uint32_t MASK5;               /*Mask register for Port5.*/
 __IO uint32_t PIN5;                /*Port5 Pin value register using FIOMASK.*/
 __IO uint32_t SET5;                /*Port5 Output Set register using FIOMASK.*/
 __O uint32_t CLR5;                 /*Port5 Output Clear register using FIOMASK.*/
 
}GPIO_TYPEDEF;

typedef struct
{
 __I uint32_t STATUS;               /*GPIO overall Interrupt Status*/
 __I uint32_t STATR0;               /*GPIO Interrupt Status for Rising edge for Port 0.*/
 __I uint32_t STATF0;               /*GPIO Interrupt Status for Falling edge for Port 0.*/
 __O uint32_t CLR0;                 /*GPIO Interrupt Clear*/
 __IO uint32_t ENR0;                /*GPIO Interrupt Enable for Rising edge for Port 0.*/
 __IO uint32_t ENF0;                /*GPIO Interrupt Enable for Falling edge for Port 0.*/
 __IO uint8_t RESERVED0[0xC];
 __I uint32_t STATR2;               /*GPIO Interrupt Status for Rising edge for Port 0.*/
 __I uint32_t STATF2;               /*GPIO Interrupt Status for Falling edge for Port 0.*/
 __O uint32_t CLR2;
 __IO uint32_t ENR2;
 __IO uint32_t ENF2;

}GPIO_INTERRUPT_TYPEDEF;

typedef struct
{
 __IO uint32_t CONTROL;                     /*Controls operation of the memory controller.*/
 __I uint32_t STATUS;                       /*Provides EMC status information*/
 __IO uint32_t CONFIG;                      /*Configures operation of the memory controller*/
 __IO uint8_t RESERVED0[0x14]; 
 __IO uint32_t DYNAMICCONTROL;              /*Controls dynamic memory operation.*/
 __IO uint32_t DYNAMICREFRESH;              /*Configures dynamic memory refresh*/
 __IO uint32_t DYNAMICREADCONFIG;           /*Configures dynamic memory read strategy.*/
 __IO uint32_t RESERVED1;
 __IO uint32_t DYNAMICRP;                   /*Precharge command period.*/
 __IO uint32_t DYNAMICRAS;                  /*Active to precharge command period.*/
 __IO uint32_t DYNAMICSREX;                 /*Self-refresh exit time.*/
 __IO uint32_t DYNAMICAPR;                  /*Last-data-out to active command time.*/
 __IO uint32_t DYNAMICDAL;                  /*Data-in to active command time.*/
 __IO uint32_t DYNAMICWR;                   /*Write recovery time.*/
 __IO uint32_t DYNAMICRC;                   /*Selects the active to active command period.*/
 __IO uint32_t DYNAMICRFC;                  /*Selects the auto-refresh period.*/
 __IO uint32_t DYNAMICXSR;                  /*Time for exit self-refresh to active command*/
 __IO uint32_t DYNAMICRRD;                  /*Latency for active bank A to active bank B*/
 __IO uint32_t DYNAMICMRD;                  /*Time for load mode register to active command.*/
 __IO uint8_t RESERVED2[0x24];
 __IO uint32_t STATICEXTENDEDWAIT;          /*Time for long static memory read and write transfers.*/
 __IO uint8_t RESERVED3[0x7C];
 __IO uint32_t DYNAMICCONFIG0;              /*Configuration information for EMC_DYCS0.*/
 __IO uint32_t DYNAMICRASCAS0;              /*RAS and CAS latencies for EMC_DYCS0. -*/
 __IO uint8_t RESERVED4[0x18];
 __IO uint32_t DYNAMICCONFIG1;              /*Configuration information for EMC_DYCS1.*/
 __IO uint32_t DYNAMICRASCAS1;              /*RAS and CAS latencies for EMC_DYCS1.*/
 __IO uint8_t RESERVED5[0x18];
 __IO uint32_t DYNAMICCONFIG2;              /*Configuration information for EMC_DYCS2*/
 __IO uint32_t DYNAMICRASCAS2;              /*RAS and CAS latencies for EMC_DYCS2*/
 __IO uint8_t RESERVED6[0x18];
 __IO uint32_t DYNAMICCONFIG3;              /*Configuration information for EMC_DYCS3.*/
 __IO uint32_t DYNAMICRASCAS3;              /*RAS and CAS latencies for EMC_DYCS3.*/
 __IO uint8_t RESERVED7[0x98];
 __IO uint32_t STATICCONFIG0;               /*Configuration for EMC_CS0.*/
 __IO uint32_t STATICWAITWEN0;              /*Delay from EMC_CS0 to write enable*/
 __IO uint32_t STATICWAITOEN0;              /*Delay from EMC_CS0 or address change,whichever is later, to output enable*/
 __IO uint32_t STATICWAITRD0;               /*Delay from EMC_CS0 to a read access.*/
 __IO uint32_t STATICWAITPAGE0;             /*Delay for asynchronous page mode sequential accesses for EMC_CS0.*/
 __IO uint32_t STATICWAITWR0;               /*Delay from EMC_CS0 to a write access.*/
 __IO uint32_t STATICWAITTURN0;             /*Number of bus turnaround cycles EMC_CS0.*/
 __IO uint32_t RESERVED8;
 __IO uint32_t STATICCONFIG1;               /*Memory configuration for EMC_CS1.*/
 __IO uint32_t STATICWAITWEN1;              /*Delay from EMC_CS1 to write enable*/
 __IO uint32_t STATICWAITOEN1;              /*Delay from EMC_CS1 or address change,whichever is later, to output enable*/
 __IO uint32_t STATICWAITRD1;               /*Delay from EMC_CS1 to a read access.*/
 __IO uint32_t STATICWAITPAGE1;             /*Delay for asynchronous page mode sequential accesses for EMC_CS1.*/
 __IO uint32_t STATICWAITWR1;               /*Delay from EMC_CS1 to a write access.*/
 __IO uint32_t STATICWAITTURN1;             /*Bus turnaround cycles for EMC_CS1.*/
 __IO uint32_t RESERVED9;
 __IO uint32_t STATICCONFIG2;               /*Memory configuration for EMC_CS2.*/
 __IO uint32_t STATICWAITWEN2;              /*Delay from EMC_CS2 to write enable.*/
 __IO uint32_t STATICWAITOEN2;              /*Delay from EMC_CS2 or address change,whichever is later, to output enable.*/
 __IO uint32_t STATICWAITRD2;               /*Delay from EMC_CS2 to a read access.*/
 __IO uint32_t STATICWAITPAGE2;             /*Delay for asynchronous page mode sequential accesses for EMC_CS2.*/
 __IO uint32_t STATICWAITWR2;               /*Delay from EMC_CS2 to a write access.*/
 __IO uint32_t EMCStaticWaitTurn2;          /*Bus turnaround cycles for EMC_CS2.*/
 __IO uint32_t RESERVED10;
 __IO uint32_t STATICCONFIG3;               /*Memory configuration for EMC_CS3*/
 __IO uint32_t STATICWAITWEN3;              /*Delay from EMC_CS3 to write enable.*/
 __IO uint32_t STATICWAITOEN3;              /*Delay from EMC_CS3 or address change,whichever is later, to output enable*/
 __IO uint32_t STATICWAITRD3;               /*Delay from EMC_CS3 to a read access.*/
 __IO uint32_t STATICWAITPAGE3;             /*Delay for asynchronous page mode sequential accesses for EMC_CS3.*/
 __IO uint32_t STATICWAITWR3;               /*Delay from EMC_CS3 to a write access.*/
 __IO uint32_t STATICWAITTURN3;             /*Bus turnaround cycles for EMC_CS3.*/
}EMC_TYPEDEF;

/****************************************************** ETHERNET **********************************************************************/

typedef struct
{
 __IO uint32_t MAC1;                         /*MAC configuration register 1.*/
 __IO uint32_t MAC2;                         /*MAC configuration register 2.*/
 __IO uint32_t IPGT;                         /*Back-to-Back Inter-Packet-Gap register*/
 __IO uint32_t IPGR;                         /*Non Back-to-Back Inter-Packet-Gap register*/
 __IO uint32_t CLRT;                         /*Collision window / Retry register.*/
 __IO uint32_t MAXF;                         /*Maximum Frame register.*/
 __IO uint32_t SUPP;                         /*PHY Support register*/
 __IO uint32_t TEST;                         /*Test register*/
 __IO uint32_t MCFG;                         /*MII Mgmt Configuration register*/
 __IO uint32_t MCMD;                         /*MII Mgmt Command register*/
 __IO uint32_t MADR;                         /*MII Mgmt Address register*/
 __O uint32_t MWTD;                          /*MII Mgmt Write Data register.*/
 __I uint32_t MRDD;                          /*MII Mgmt Read Data register*/
 __I uint32_t MIND;                          /*MII Mgmt Indicators register.*/
 __IO uint8_t RESERVED0[8];	
 __IO uint32_t SA0;                          /*Station Address 0 register.*/
 __IO uint32_t SA1;                          /*Station Address 1 register.*/
 __IO uint32_t SA2;                          /*Station Address 2 register.*/
 __IO uint8_t RESERVED1[0xB4];
 __IO uint32_t COMMAND;                      /*Command register*/
 __I uint32_t STATUS;                        /*Status register*/
 __IO uint32_t RXDESCRIPTOR;                 /*Receive descriptor base address register.*/
 __IO uint32_t RXSTATUS;                     /*Receive status base address register.*/
 __IO uint32_t RXDESCRIPTORNUMBER;           /*Receive number of descriptors register*/
 __I uint32_t RXPRODUCEINDEX;                /*Receive produce index register*/
 __IO uint32_t RXCONSUMEINDEX;               /*Receive consume index register*/
 __IO uint32_t TXDESCRIPTOR;                 /*Transmit descriptor base address register*/
 __IO uint32_t TXSTATUS;                     /*Transmit status base address register*/
 __IO uint32_t TXDESCRIPTORNUMBER;           /*Transmit number of descriptors register.*/
 __IO uint32_t TXPRODUCEINDEX;               /*TXPRODUCEINDEX*/
 __I uint32_t TXCONSUMEINDEX;                /*Transmit consume index register*/
 __IO uint8_t RESERVED2[0x28];             
 __I uint32_t TSV0;                          /*Transmit status vector 0 register.*/
 __I uint32_t TSV1;                          /*Transmit status vector 1 register.*/
 __I uint32_t RSV;                           /*Receive status vector register*/
 __IO uint8_t RESERVED3[0xC];
 __IO uint32_t FLOWCONTROLCOUNTER;           /*Flow control counter register*/
 __I uint32_t FLOWCONTROLSTATUS;             /*Flow control status register*/ 
 __IO uint8_t RESERVED4[0x88];
 __IO uint32_t RXFILTERCTRL;                 /*Receive filter control register*/
 __I uint32_t RXFILTERWOLSTATUS;             /*Receive filter WoL status register*/
 __O uint32_t RXFILTERWOLCLEAR;              /*Receive filter WoL clear register.*/
 __IO uint32_t RESERVED5;
 __IO uint32_t HASHFILTERL;                  /*Hash filter table LSBs register.*/
 __IO uint32_t HASHFILTERH;                  /*Hash filter table MSBs register*/
 __IO uint8_t RESERVED6[0xDC8];
 __I uint32_t INTSTATUS;                     /*Interrupt status register*/
 __IO uint32_t INTENABLE;                    /*Interrupt enable register*/
 __O uint32_t INTCLEAR;                      /*Interrupt clear register*/
 __O uint32_t INTSET;                        /*Interrupt set register*/
 __IO uint32_t POWERDOWN;                    /*Power-down register.*/
}ETHERNET_TYPEDEF;

/****************************************** LCD CONTROLLER ****************************************************************************/

typedef struct
{
 __IO uint32_t TIMH;                          /*Horizontal Timing Control register*/
 __IO uint32_t TIMV;                          /*Vertical Timing Control register*/
 __IO uint32_t POL;                           /*Clock and Signal Polarity Control register*/
 __IO uint32_t LE;                            /*Line End Control register*/
 __IO uint32_t UPBASE;                        /*Upper Panel Frame Base Address register*/
 __IO uint32_t LPBASE;                        /*Lower Panel Frame Base Address register*/
 __IO uint32_t CTRL;                          /*LCD Control register*/
 __IO uint32_t INTMSK;                        /*Interrupt Mask register*/
 __I uint32_t INTRAW;                         /*Raw Interrupt Status register*/
 __I uint32_t INTSTAT;                        /*Masked Interrupt Status register*/
 __O uint32_t INTCLR;                         /*Interrupt Clear register*/
 __I uint32_t UPCURR;                         /*Upper Panel Current Address Value register*/
 __I uint32_t LPCURR;                         /*Lower Panel Current Address Value register*/
 __IO uint8_t RESERVED0[0x1CC];
	
 __IO uint32_t PAL0;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL1;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL2;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL3;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL4;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL5;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL6;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL7;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL8;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL9;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL10;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL11;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL12;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL13;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL14;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL15;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL16;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL17;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL18;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL19;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL20;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL21;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL22;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL23;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL24;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL25;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL26;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL27;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL28;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL29;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL30;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL31;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL32;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL33;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL34;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL35;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL36;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL37;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL38;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL39;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL40;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL41;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL42;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL43;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL44;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL45;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL46;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL47;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL48;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL49;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL50;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL51;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL52;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL53;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL54;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL55;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL56;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL57;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL58;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL59;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL60;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL61;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL62;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL63;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL64;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL65;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL66;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL67;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL68;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL69;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL70;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL71;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL72;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL73;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL74;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL75;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL76;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL77;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL78;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL79;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL80;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL81;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL82;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL83;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL84;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL85;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL86;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL87;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL88;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL89;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL90;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL91;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL92;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL93;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL94;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL95;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL96;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL97;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL98;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL99;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL100;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL101;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL102;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL103;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL104;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL105;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL106;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL107;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL108;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL109;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL110;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL111;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL112;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL113;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL114;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL115;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL116;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL117;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL118;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL119;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL120;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL121;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL122;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL123;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL124;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL125;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL126;                           /*256x16-bit Color Palette registers*/
 __IO uint32_t PAL127;                           /*256x16-bit Color Palette registers*/

 __IO uint8_t RESERVED1[0x400];

 __IO uint32_t CRSR_IMG0;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG1;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG2;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG3;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG4;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG5;                        /*Cursor Image registers*/ 
 __IO uint32_t CRSR_IMG6;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG7;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG8;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG9;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG10;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG11;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG12;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG13;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG14;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG15;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG16;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG17;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG18;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG19;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG20;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG21;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG22;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG23;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG24;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG25;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG26;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG27;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG28;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG29;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG30;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG31;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG32;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG33;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG34;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG35;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG36;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG37;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG38;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG39;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG40;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG41;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG42;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG43;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG44;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG45;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG46;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG47;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG48;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG49;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG50;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG51;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG52;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG53;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG54;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG55;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG56;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG57;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG58;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG59;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG60;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG61;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG62;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG63;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG64;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG65;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG66;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG67;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG68;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG69;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG70;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG71;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG72;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG73;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG74;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG75;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG76;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG77;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG78;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG79;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG80;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG81;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG82;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG83;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG84;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG85;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG86;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG87;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG88;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG89;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG90;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG91;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG92;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG93;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG94;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG95;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG96;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG97;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG98;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG99;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG100;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG101;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG102;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG103;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG104;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG105;                        /*Cursor Image registers*/ 
 __IO uint32_t CRSR_IMG106;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG107;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG108;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG109;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG110;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG111;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG112;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG113;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG114;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG115;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG116;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG117;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG118;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG119;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG120;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG121;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG122;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG123;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG124;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG125;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG126;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG127;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG128;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG129;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG130;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG131;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG132;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG133;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG134;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG135;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG136;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG137;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG138;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG139;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG140;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG141;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG142;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG143;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG144;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG145;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG146;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG147;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG148;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG149;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG150;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG151;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG152;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG153;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG154;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG155;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG156;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG157;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG158;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG159;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG160;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG161;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG162;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG163;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG164;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG165;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG166;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG167;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG168;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG169;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG170;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG171;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG172;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG173;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG174;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG175;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG176;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG177;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG178;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG179;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG180;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG181;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG182;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG183;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG184;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG185;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG186;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG187;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG188;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG189;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG190;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG191;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG192;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG193;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG194;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG195;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG196;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG197;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG198;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG199;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG200;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG201;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG202;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG203;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG204;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG205;                        /*Cursor Image registers*/ 
 __IO uint32_t CRSR_IMG206;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG207;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG208;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG209;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG210;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG211;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG212;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG213;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG214;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG215;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG216;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG217;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG218;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG219;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG220;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG221;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG222;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG223;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG224;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG225;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG226;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG227;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG228;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG229;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG230;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG231;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG232;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG233;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG234;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG235;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG236;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG237;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG238;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG239;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG240;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG241;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG242;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG243;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG244;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG245;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG246;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG247;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG248;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG249;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG250;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG251;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG252;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG253;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG254;                        /*Cursor Image registers*/
 __IO uint32_t CRSR_IMG255;                        /*Cursor Image registers*/
 
 __IO uint32_t CRSR_CTRL;                          /*Cursor Control register*/
 __IO uint32_t CRSR_CFG;                           /*Cursor Configuration register*/
 __IO uint32_t CRSR_PAL0;                          /*Cursor Palette register 0*/
 __IO uint32_t CRSR_PAL1;                          /*Cursor Palette register 1*/
 __IO uint32_t CRSR_XY;                            /*Cursor XY Position register*/
 __IO uint32_t CRSR_CLIP;                          /*Cursor Clip Position register*/
 __IO uint32_t CRSR_INTMSK;                        /*Cursor Interrupt Mask register*/
 __O uint32_t CRSR_INTCLR;                         /*Cursor Interrupt Clear register*/
 __I uint32_t CRSR_INTRAW;                         /*Cursor Raw Interrupt Status register*/
 __I uint32_t CRSR_INTSTAT;                        /*Cursor Masked Interrupt Status register*/
}LCD_CONTROLLER_TYPEDEF;

/***************************************** USB device controller **********************************************************************/

typedef struct
{
 __IO uint32_t PORTSEL;                           /*USB Port Select. This register is also used for OTG configuration. In device-only operations only bits 0 and 1 of
                                                    this register are*/
 __IO uint8_t RESERVED0[0xEC];
 __I uint32_t DEVINTST;                           /*USB Device Interrupt Status*/
 __IO uint32_t DEVINTEN;                          /*USB Device Interrupt Enable*/
 __O uint32_t DEVINTCLR;                          /*USB Device Interrupt Clear*/
 __O uint32_t DEVINTSET;                          /*USB Device Interrupt Set*/
 __O uint32_t CMDCODE;                            /*USB Command Code*/
 __I uint32_t CMDDATA;                            /*USB Command Data*/
 __I uint32_t RXDATA;                             /*USB Receive Data*/
 __O uint32_t TXDATA;                             /*USB Transmit Data*/
 __I uint32_t RXPLEN;                             /*USB Receive Packet Length*/
 __O uint32_t TXPLEN;                             /*USB Transmit Packet Length*/
 __IO uint32_t CTRL;                              /*USB Control*/
 __O uint32_t DEVINTPRI;                          /*USB Device Interrupt Priority*/
 __I uint32_t EPINTST;                            /*USB Endpoint Interrupt Status*/
 __IO uint32_t EPINTEN;                           /*USB Endpoint Interrupt Enable*/
 __O uint32_t EPINTCLR;                           /*USB Endpoint Interrupt Clear*/
 __O uint32_t EPINTSET;                           /*USB Endpoint Interrupt Set*/
 __O uint32_t EPINTPRI;                           /*USB Endpoint Priority*/
 __IO uint32_t REEP;                              /*USB Realize Endpoint*/
 __O uint32_t EPIN;                               /*USB Endpoint Index*/
 __IO uint32_t MAXPSIZE;                          /*USB MaxPacketSize*/
 __I uint32_t DMARST;                             /*USB DMA Request Status*/
 __O uint32_t DMARCLR;                            /*USB DMA Request Clear*/
 __O uint32_t DMARSET;                            /*USB DMA Request Set*/
 __IO uint8_t RESERVED1[0x24];
 __IO uint32_t UDCAH;                             /*USB UDCA Head*/
 __I uint32_t EPDMAST;                            /*USB Endpoint DMA Status*/
 __O uint32_t EPDMAEN;                            /*USB Endpoint DMA Enable*/
 __O uint32_t EPDMADIS;                           /*USB Endpoint DMA Disable*/
 __I uint32_t DMAINTST;                           /*USB DMA Interrupt Status*/
 __IO uint32_t DMAINTEN;                          /*USB DMA Interrupt Enable*/
 __IO uint8_t RSESEVED2[8];      
 __I uint32_t EOTINTST;                           /*USB End of Transfer Interrupt Status*/
 __O uint32_t EOTINTCLR;                          /*USB End of Transfer Interrupt Clear*/
 __O uint32_t EOTINTSET;                          /*USB End of Transfer Interrupt Set*/
 __I uint32_t NDDRINTST;                          /*USB New DD Request Interrupt Status*/
 __O uint32_t NDDRINTCLR;                         /*USB New DD Request Interrupt Clear*/
 __O uint32_t NDDRINTSET;                         /*USB New DD Request Interrupt Set*/
 __I uint32_t SYSERRINTST;                        /*USB System Error Interrupt Status*/
 __O uint32_t SYSERRINTCLR;                       /*USB System Error Interrupt Clear*/
 __O uint32_t SYSERRINTSET;                       /*USB System Error Interrupt Set*/
 __IO uint8_t RESERVED3[0xD30];
 __IO uint32_t CLKCTRL;                           /*USB Clock Control*/
 __I uint32_t CLKST;                              /*USB Clock Status*/ 
}USB_DEVICE_CONTROLLER_TYPEDEF;

/************************************************* USB HOST CONTROLLER *****************************************************************/

typedef struct
{
 __I uint32_t HcRevision;                          /*BCD representation of the version of the HCI specification that is implemented by the Host Controller*/
 __IO uint32_t HcControl;                          /*Defines the operating modes of the HC.*/
 __IO uint32_t HcCommandStatus;                    /*This register is used to receive the commands from the Host Controller Driver (HCD). It also indicates the status of the HC*/
 __IO uint32_t HcInterruptStatus;                  /*Indicates the status on various events that cause hardware interrupts by setting the appropriate bits.*/
 __IO uint32_t HcInterruptEnable;                  /*Controls the bits in the HcInterruptStatus register and indicates which events will generate a hardware interrupt.*/
 __IO uint32_t HcInterruptDisable;                 /*The bits in this register are used to disable corresponding bits in the HCInterruptStatus register and in turn disable that event leading to hardware interrupt.*/
 __IO uint32_t HcHCCA;                             /*Contains the physical address of the host controller communication area.*/
 __I uint32_t HcPeriodCurrentED;                   /*Contains the physical address of the current isochronous or interrupt endpoint descriptor*/
 __IO uint32_t HcControlHeadED;                    /*Contains the physical address of the first endpoint descriptor of the control list.*/
 __IO uint32_t HcControlCurrentED;                 /*Contains the physical address of the current endpoint descriptor of the control list*/
 __IO uint32_t HcBulkHeadED;                       /*Contains the physical address of the first endpoint descriptor of the bulk list.*/
 __IO uint32_t HcBulkCurrentED;                    /*Contains the physical address of the current endpoint descriptor of the bulk list.*/
 __I uint32_t HcDoneHead;                          /*Contains the physical address of the last transfer descriptor added to the ‘Done’ queue.*/
 __IO uint32_t HcFmInterval;                       /*Defines the bit time interval in a frame and the full speed maximum packet size which would not cause an overrun.*/
 __I uint32_t HcFmRemaining;                       /*A 14-bit counter showing the bit time remaining in the current frame*/
 __I uint32_t HcFmNumber;                          /*Contains a 16-bit counter and provides the timing reference among events happening in the HC and the HCD*/
 __IO uint32_t HcPeriodicStart;                    /*Contains a programmable 14-bit value which determines the earliest time HC should start processing a periodic list*/
 __IO uint32_t HcLSThreshold;                      /*Contains 11-bit value which is used by the HC to determine whether to commit to transfer a maximum of 8-byte LS packet before EOF*/
 __IO uint32_t HcRhDescriptorA;                    /*First of the two registers which describes the characteristics of the root hub.*/
 __IO uint32_t HcRhDescriptorB;                    /*Second of the two registers which describes the characteristics of the Root Hub.*/
 __IO uint32_t HcRhStatus;                         /*This register is divided into two parts. The lower D-word represents the hub status field and the upper word represents the hub status change field.*/
 __IO uint32_t HcRhPortStatus1;                    /*Controls and reports the port events on a per-port basis*/
 __IO uint32_t HcRhPortStatus2;                    /*Controls and reports the port events on a per port basis.*/
 __IO uint8_t RESERVED0[0xA0]; 
 __I uint32_t Module_ID_OR_Ver_Rev_ID;             /*IP number, where yy (0x00) is unique version number and zz (0x00) is a unique revision number*/
}USB_HOST_CONTROLLER_TYPEDEF;

/************************************************* USB OTG controller ****************************************************************/
typedef struct 
{
 /*$$$$$$$$$$$$$$$$ OTG Registers $$$$$$$$$$$$$$$$*/
	__I uint32_t INTST;                             /*OTG Interrupt Status*/
	__IO uint32_t INTEN;                            /*OTG Interrupt Enable*/
	__O uint32_t INTSET;                            /*OTG Interrupt Set*/
	__O uint32_t INTCLR;                            /*OTG Interrupt Clear*/
	__IO uint32_t PORTSEL;                          /*OTG Status and Control and USB port select*/
  __IO uint32_t TMR;                              /*OTG Timer*/
	
  __IO uint8_t RESERVED0[0x1E8];
	 union I2C_
	 {
		 __I uint32_t I2C_RX;                        /*I2C Receive*/
		 __O uint32_t I2C_TX;                        /*I2C Transmit*/
	 }USB_OTG_CONTROLLER_TYPEDEF_I2C;
	 __I uint32_t I2C_STS;                         /*I2C Status*/
	 __IO uint32_t I2C_CTL;                        /*I2C Control*/
	 __IO uint32_t I2C_CLKHI;                      /*I2C Clock High*/
	 __IO uint32_t I2C_CLKLO;                      /*I2C Clock Low*/
	 __IO uint8_t RESERVED1[0xCE0];
	 __IO uint32_t CLKCTRL;                        /*OTG clock controller*/
	 __I uint32_t CLKST;                /*OTG clock status*/
}USB_OTG_CONTROLLER_TYPEDEF;

/***************************************************** SD card interface **********************************************************************************/
typedef struct
{
 __IO uint32_t PWR;                             /*Power control register*/
 __IO uint32_t CLOCK;                           /*Clock control register.*/
 __IO uint32_t ARGUMENT;                        /*Argument register.*/
 __IO uint32_t COMMAND;                         /*Command register.*/
 __I uint32_t RESPCMD;                          /*Response command register*/
 __I uint32_t RESPONSE0;                        /*Response register.*/
 __I uint32_t RESPONSE1;                        /*Response register.*/
 __I uint32_t RESPONSE2;                        /*Response register.*/
 __I uint32_t RESPONSE3;                        /*Response register.*/
 __IO uint32_t DATATIMER;                       /*Data Timer*/
 __IO uint32_t DATALENGTH;                      /*Data length register.*/
 __IO uint32_t DATACTRL;                        /*Data control register*/
 __I uint32_t DATACNT;                          /*Data counter.*/
 __I uint32_t STATUS;                           /*Status register.*/
 __O uint32_t CLEAR;                            /*Clear register*/
 __IO uint32_t MASK0;                           /*Interrupt 0 mask register.*/
 __IO uint8_t RESERVED0[0x40];               
 __IO uint32_t FIFO[0xF];                       /*Data FIFO Register*/
 __IO uint8_t RESERVED1[0x3F88];
 __I uint32_t FIFOCNT;                          /*FIFO Counter.*/
}SD_CARD_INTERFACE_TYPEDEF;

/************************************************* UART1 *******************************************************************************/

typedef struct
{
	union{
 __I uint32_t RBR;                              /*DLAB =0. Receiver Buffer Register. Contains the next received character to be read.*/
 __O uint32_t THR;                              /*DLAB =0. Transmit Holding Register. The next character to be transmitted is written here.*/
 __IO uint32_t DLL;}UN1;                        /*DLAB =1. Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 union {
 __IO uint32_t DLM;                             /*DLAB =1. Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 __IO uint32_t IER;                             /*DLAB =0. Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART1 interrupts*/
 
 }UN2;
 union {
 __I uint32_t IIR;                              /*Interrupt ID Register. Identifies which interrupt(s) are pending.*/
 __O uint32_t FCR;                              /*FIFO Control Register. Controls UART1 FIFO usage and modes.*/
 }UN3;
 __IO uint32_t LCR;                             /*Line Control Register. Contains controls for frame formatting and break generation*/
 __IO uint32_t MCR;                             /*Modem Control Register. Contains controls for flow control handshaking and loopback mode*/
 __I uint32_t LSR;                              /*Line Status Register. Contains flags for transmit and receive status, including line errors*/
 __I uint32_t MSR;                              /*Modem Status Register. Contains handshake signal status flags.*/
 __IO uint32_t SCR;                             /*Scratch Pad Register. 8-bit temporary storage for software*/
 __IO uint32_t ACR;                             /*Auto-baud Control Register. Contains controls for the auto-baud feature*/
 __IO uint32_t RESERVED0;
 __IO uint32_t FDR;                             /*Fractional Divider Register. Generates a clock input for the baud rate divider*/
 __IO uint32_t RESERVED1;
 __IO uint32_t TER;                             /*Transmit Enable Register. Turns off UART transmitter for use with software flow control*/
 __IO uint8_t RESERVED[0x18];
 __IO uint32_t RS485CTRL;                       /*RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes*/
 __IO uint32_t RSADRMATCH;                      /*RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode*/
 __IO uint32_t RS485DLY;                        /*RS-485/EIA-485 direction control delay.*/
}UART1_TYPEDEF;

/*************************************************** UART0 ****************************************************************************/

typedef struct
{
union{
 __I uint32_t RBR;                              /*DLAB =0. Receiver Buffer Register. Contains the next received character to be read.*/
 __O uint32_t THR;                              /*DLAB =0. Transmit Holding Register. The next character to be transmitted is written here.*/
 __IO uint32_t DLL;}UN1;                        /*DLAB =1. Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 union {
 __IO uint32_t DLM;                             /*DLAB =1. Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 __IO uint32_t IER;                             /*DLAB =0. Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART1 interrupts*/
 
 }UN2;
 union {
 __I uint32_t IIR;                              /*Interrupt ID Register. Identifies which interrupt(s) are pending.*/
 __O uint32_t FCR;                              /*FIFO Control Register. Controls UART1 FIFO usage and modes.*/
 }UN3;
 __IO uint32_t LCR;                             /*Line Control Register. Contains controls for frame formatting and break generation*/
 __IO uint32_t RESERVED2;                             /*Modem Control Register. Contains controls for flow control handshaking and loopback mode*/
 __I uint32_t LSR;                              /*Line Status Register. Contains flags for transmit and receive status, including line errors*/
 __I uint32_t RESERVED3;                              /*Modem Status Register. Contains handshake signal status flags.*/
 __IO uint32_t SCR;                             /*Scratch Pad Register. 8-bit temporary storage for software*/
 __IO uint32_t ACR;                             /*Auto-baud Control Register. Contains controls for the auto-baud feature*/
 __IO uint32_t RESERVED0;
 __IO uint32_t FDR;                             /*Fractional Divider Register. Generates a clock input for the baud rate divider*/
 __IO uint32_t RESERVED1;
 __IO uint32_t TER;                             /*Transmit Enable Register. Turns off UART transmitter for use with software flow control*/
 __IO uint8_t RESERVED[0x18];
 __IO uint32_t RS485CTRL;                       /*RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes*/
 __IO uint32_t RSADRMATCH;                      /*RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode*/
 __IO uint32_t RS485DLY;                        /*RS-485/EIA-485 direction control delay.*/	
}UART0_TYPEDEF;

/*************************************************** UART2 ****************************************************************************/

typedef struct
{
union{
 __I uint32_t RBR;                              /*DLAB =0. Receiver Buffer Register. Contains the next received character to be read.*/
 __O uint32_t THR;                              /*DLAB =0. Transmit Holding Register. The next character to be transmitted is written here.*/
 __IO uint32_t DLL;}UN1;                        /*DLAB =1. Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 union {
 __IO uint32_t DLM;                             /*DLAB =1. Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 __IO uint32_t IER;                             /*DLAB =0. Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART1 interrupts*/
 
 }UN2;
 union {
 __I uint32_t IIR;                              /*Interrupt ID Register. Identifies which interrupt(s) are pending.*/
 __O uint32_t FCR;                              /*FIFO Control Register. Controls UART1 FIFO usage and modes.*/
 }UN3;
 __IO uint32_t LCR;                             /*Line Control Register. Contains controls for frame formatting and break generation*/
 __IO uint32_t RESERVED2;                             /*Modem Control Register. Contains controls for flow control handshaking and loopback mode*/
 __I uint32_t LSR;                              /*Line Status Register. Contains flags for transmit and receive status, including line errors*/
 __I uint32_t RESERVED3;                              /*Modem Status Register. Contains handshake signal status flags.*/
 __IO uint32_t SCR;                             /*Scratch Pad Register. 8-bit temporary storage for software*/
 __IO uint32_t ACR;                             /*Auto-baud Control Register. Contains controls for the auto-baud feature*/
 __IO uint32_t RESERVED0;
 __IO uint32_t FDR;                             /*Fractional Divider Register. Generates a clock input for the baud rate divider*/
 __IO uint32_t RESERVED1;
 __IO uint32_t TER;                             /*Transmit Enable Register. Turns off UART transmitter for use with software flow control*/
 __IO uint8_t RESERVED[0x18];
 __IO uint32_t RS485CTRL;                       /*RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes*/
 __IO uint32_t RSADRMATCH;                      /*RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode*/
 __IO uint32_t RS485DLY;                        /*RS-485/EIA-485 direction control delay.*/	
}UART2_TYPEDEF;

/*************************************************** UART3 ****************************************************************************/

typedef struct
{
union{
 __I uint32_t RBR;                              /*DLAB =0. Receiver Buffer Register. Contains the next received character to be read.*/
 __O uint32_t THR;                              /*DLAB =0. Transmit Holding Register. The next character to be transmitted is written here.*/
 __IO uint32_t DLL;}UN1;                        /*DLAB =1. Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 union {
 __IO uint32_t DLM;                             /*DLAB =1. Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 __IO uint32_t IER;                             /*DLAB =0. Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART1 interrupts*/
 
 }UN2;
 union {
 __I uint32_t IIR;                              /*Interrupt ID Register. Identifies which interrupt(s) are pending.*/
 __O uint32_t FCR;                              /*FIFO Control Register. Controls UART1 FIFO usage and modes.*/
 }UN3;
 __IO uint32_t LCR;                             /*Line Control Register. Contains controls for frame formatting and break generation*/
 __IO uint32_t RESERVED2;                             /*Modem Control Register. Contains controls for flow control handshaking and loopback mode*/
 __I uint32_t LSR;                              /*Line Status Register. Contains flags for transmit and receive status, including line errors*/
 __I uint32_t RESERVED3;                              /*Modem Status Register. Contains handshake signal status flags.*/
 __IO uint32_t SCR;                             /*Scratch Pad Register. 8-bit temporary storage for software*/
 __IO uint32_t ACR;                             /*Auto-baud Control Register. Contains controls for the auto-baud feature*/
 __IO uint32_t RESERVED0;
 __IO uint32_t FDR;                             /*Fractional Divider Register. Generates a clock input for the baud rate divider*/
 __IO uint32_t RESERVED1;
 __IO uint32_t TER;                             /*Transmit Enable Register. Turns off UART transmitter for use with software flow control*/
 __IO uint8_t RESERVED[0x18];
 __IO uint32_t RS485CTRL;                       /*RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes*/
 __IO uint32_t RSADRMATCH;                      /*RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode*/
 __IO uint32_t RS485DLY;                        /*RS-485/EIA-485 direction control delay.*/	
}UART3_TYPEDEF;


/*************************************************** UART4 ****************************************************************************/

typedef struct
{
union{
 __I uint32_t RBR;                              /*DLAB =0. Receiver Buffer Register. Contains the next received character to be read.*/
 __O uint32_t THR;                              /*DLAB =0. Transmit Holding Register. The next character to be transmitted is written here.*/
 __IO uint32_t DLL;}UN1;                        /*DLAB =1. Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 union {
 __IO uint32_t DLM;                             /*DLAB =1. Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud
                                                  rate from the fractional rate divider.*/
 __IO uint32_t IER;                             /*DLAB =0. Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART1 interrupts*/
 
 }UN2;
 union {
 __I uint32_t IIR;                              /*Interrupt ID Register. Identifies which interrupt(s) are pending.*/
 __O uint32_t FCR;                              /*FIFO Control Register. Controls UART1 FIFO usage and modes.*/
 }UN3;
 __IO uint32_t LCR;                             /*Line Control Register. Contains controls for frame formatting and break generation*/
 __IO uint32_t RESERVED2;                             /*Modem Control Register. Contains controls for flow control handshaking and loopback mode*/
 __I uint32_t LSR;                              /*Line Status Register. Contains flags for transmit and receive status, including line errors*/
 __I uint32_t RESERVED3;                              /*Modem Status Register. Contains handshake signal status flags.*/
 __IO uint32_t SCR;                             /*Scratch Pad Register. 8-bit temporary storage for software*/
 __IO uint32_t ACR;                             /*Auto-baud Control Register. Contains controls for the auto-baud feature*/
 __IO uint32_t ICR;                             /*IrDA Control Register. Enables and configures the IrDA mode.*/
 __IO uint32_t FDR;                             /*Fractional Divider Register. Generates a clock input for the baud rate divider*/
 __IO uint32_t OSR;                             /*Oversampling register. Controls the degree of oversampling during each bit time*/
 __IO uint8_t RESERVED[0x18];
 __IO uint32_t SCICTRL;                         /*Smart Card Interface control register. Enables and configures the smartcard Interface feature*/
 __IO uint32_t RS485CTRL;                       /*RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes*/
 __IO uint32_t ADRMATCH;                        /*RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode*/
 __IO uint32_t RS485DLY;                        /*RS-485/EIA-485 direction control delay.*/	
 __IO uint32_t SYNCCTRL;                        /*Synchronous mode control register*/
}UART4_TYPEDEF;

/******************************************** CAN ACCEPTANCE FILTER *******************************************************************/

typedef struct
{
 __IO uint32_t AFMR;                            /*Acceptance Filter Register*/
 __IO uint32_t SFF_SA;                          /*Standard Frame Individual Start Address Register*/
 __IO uint32_t SFF_GRP_SA;                      /*Standard Frame Group Start Address Register*/
 __IO uint32_t EFF_SA;                          /*Extended Frame Start Address Register*/
 __IO uint32_t EFF_GRP_SA;                      /*Extended Frame Group Start Address Register*/
 __IO uint32_t ENDOFTABLE;                      /*End of AF Tables register*/
 __I uint32_t LUTERRAD;                         /*LUT Error Address register*/
 __I uint32_t LUTERR;                           /*LUT Error Register*/
 __IO uint32_t FCANIE;                          /*FullCAN interrupt enable register*/
 __IO uint32_t FCANIC0;                         /*FullCAN interrupt and capture register0*/
 __IO uint32_t FCANIC1;                         /*FullCAN interrupt and capture register1*/
}CAN_ACCEPTANCE_FILTER_TYPEDEF;

/*********************************************** CENTRAL CAN **************************************************************************/

typedef struct
{
 __I uint32_t TXSR;                             /*CAN Central Transmit Status Register*/
 __I uint32_t RXSR;                             /*CAN Central Receive Status Register*/
 __I uint32_t MSR;                              /*CAN Central Miscellaneous Register*/
}CENTRAL_CAN_TYPEDEF;

/**************************************************** CAN1 ****************************************************************************/

typedef struct
{
 __IO uint32_t MOD;                             /*Controls the operating mode of the CAN Controller*/
 __O uint32_t CMR;                              /*Command bits that affect the state of the CAN Controller*/
 __I uint32_t GSR;                              /*Global Controller Status and Error Counters. The error counters can only be written when RM in CANMOD is 1.*/
 __I uint32_t ICR;                              /*Interrupt status, Arbitration Lost Capture, Error Code Capture*/
 __IO uint32_t IER;                             /*Interrupt Enable*/
 __IO uint32_t BTR;                             /*Bus Timing. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t EWL;                             /*Error Warning Limit. Can only be written when RM in CANMOD is 1*/
 __I uint32_t SR;                               /*Status Register*/
 __IO uint32_t RFS;                             /*Receive frame status. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t RID;                             /*Received Identifier. Can only be written when RM in CANMOD is 1*/
 __IO uint32_t RDA;                             /*Received data bytes 1-4. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t RDB;                             /*Received data bytes 5-8. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t TFI1;                            /*Transmit frame info (Tx Buffer 1)*/
 __IO uint32_t TID1;                            /*Transmit Identifier (Tx Buffer 1)*/
 __IO uint32_t TDA1;                            /*Transmit data bytes 1-4 (Tx Buffer 1)*/
 __IO uint32_t TDB1;                            /*Transmit data bytes 5-8 (Tx Buffer 1)*/
 __IO uint32_t TFI2;                            /*Transmit frame info (Tx Buffer 2)*/
 __IO uint32_t TID2;                            /*Transmit Identifier (Tx Buffer 2)*/
 __IO uint32_t TDA2;                            /*Transmit data bytes 1-4 (Tx Buffer 2)*/
 __IO uint32_t TDB2;                            /*Transmit data bytes 5-8 (Tx Buffer 2)*/
 __IO uint32_t TFI3;                            /*Transmit frame info (Tx Buffer 3)*/
 __IO uint32_t TID3;                            /*Transmit Identifier (Tx Buffer 3)*/
 __IO uint32_t TDA3;                            /*Transmit data bytes 1-4 (Tx Buffer 3)*/
 __IO uint32_t TDB3;                            /*Transmit data bytes 5-8 (Tx Buffer 3)*/
}CAN1_TYPEDEF;

/**************************************************** CAN2 ****************************************************************************/

typedef struct
{
 __IO uint32_t MOD;                             /*Controls the operating mode of the CAN Controller*/
 __O uint32_t CMR;                              /*Command bits that affect the state of the CAN Controller*/
 __I uint32_t GSR;                              /*Global Controller Status and Error Counters. The error counters can only be written when RM in CANMOD is 1.*/
 __I uint32_t ICR;                              /*Interrupt status, Arbitration Lost Capture, Error Code Capture*/
 __IO uint32_t IER;                             /*Interrupt Enable*/
 __IO uint32_t BTR;                             /*Bus Timing. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t EWL;                             /*Error Warning Limit. Can only be written when RM in CANMOD is 1*/
 __I uint32_t SR;                               /*Status Register*/
 __IO uint32_t RFS;                             /*Receive frame status. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t RID;                             /*Received Identifier. Can only be written when RM in CANMOD is 1*/
 __IO uint32_t RDA;                             /*Received data bytes 1-4. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t RDB;                             /*Received data bytes 5-8. Can only be written when RM in CANMOD is 1.*/
 __IO uint32_t TFI1;                            /*Transmit frame info (Tx Buffer 1)*/
 __IO uint32_t TID1;                            /*Transmit Identifier (Tx Buffer 1)*/
 __IO uint32_t TDA1;                            /*Transmit data bytes 1-4 (Tx Buffer 1)*/
 __IO uint32_t TDB1;                            /*Transmit data bytes 5-8 (Tx Buffer 1)*/
 __IO uint32_t TFI2;                            /*Transmit frame info (Tx Buffer 2)*/
 __IO uint32_t TID2;                            /*Transmit Identifier (Tx Buffer 2)*/
 __IO uint32_t TDA2;                            /*Transmit data bytes 1-4 (Tx Buffer 2)*/
 __IO uint32_t TDB2;                            /*Transmit data bytes 5-8 (Tx Buffer 2)*/
 __IO uint32_t TFI3;                            /*Transmit frame info (Tx Buffer 3)*/
 __IO uint32_t TID3;                            /*Transmit Identifier (Tx Buffer 3)*/
 __IO uint32_t TDA3;                            /*Transmit data bytes 1-4 (Tx Buffer 3)*/
 __IO uint32_t TDB3;                            /*Transmit data bytes 5-8 (Tx Buffer 3)*/
}CAN2_TYPEDEF;

/***************************************** CAN WAKE AND SLEEP *************************************************************************/

typedef struct
{
 __IO uint32_t CANSLEEPCLR;                    /*Allows clearing the current CAN channel sleep state as well as reading that state.*/
 __IO uint32_t CANWAKEFLAGS;                   /*Allows reading the wake-up state of the CAN channels*/
}CAN_WAKE_AND_SLEEP_TYPEDEF;

/****************************************************** SSP ***************************************************************************/

typedef struct
{
 __IO uint32_t CR0;                           /*Control Register 0. Selects the serial clock rate, bus type, and data size.*/
 __IO uint32_t CR1;                           /*Control Register 1. Selects master/slave and other modes*/
 __IO uint32_t DR;                            /*Data Register. Writes fill the transmit FIFO, and reads empty the receive FIFO*/
 __I uint32_t SR;                             /*Status Register*/
 __IO uint32_t CPSR;                          /*Clock Prescale Register*/
 __IO uint32_t IMSC;                          /*Interrupt Mask Set and Clear Register*/
 __IO uint32_t RIS;                           /*Raw Interrupt Status Register*/
 __IO uint32_t MIS;                           /*Masked Interrupt Status Register*/
 __IO uint32_t ICR;                           /*SSPICR Interrupt Clear Register*/
 __IO uint32_t DMACR;                         /*DMA Control Register*/
}SSP_TYPEDEF;

/************************************************* I2C BUS INTERFACE ******************************************************************/

typedef struct
{
 __IO uint32_t CONSET;                        /*I2C Control Set Register. When a one is written to a bit of this register,the corresponding bit in the I2C control register is set. Writing a zero has
                                                no effect on the corresponding bit in the I2C control register.*/
 __I uint32_t STAT;                           /*I2C Status Register. During I2C operation, this register provides detailed status codes that allow software to determine the next action
                                                needed.*/
 __IO uint32_t DAT;                           /*I2C Data Register. During master or slave transmit mode, data to be transmitted is written to this register. During master or slave receive
                                                mode, data that has been received may be read from this register.*/
 __IO uint32_t ADR0;                          /*I2C Slave Address Register 0. Contains the 7-bit slave address for operation of the I2C interface in slave mode, and is not used in master
                                                mode. The least significant bit determines whether a slave responds to the General Call address.*/
 __IO uint32_t SCLH;                          /*SCH Duty Cycle Register High Half Word. Determines the high time of the I2C clock.*/
 __IO uint32_t SCLL;                          /*SCL Duty Cycle Register Low Half Word. Determines the low time of
                                                the I2C clock. I2nSCLL and I2nSCLH together determine the clock
                                                frequency generated by an I2C master and certain times used in slave
                                                mode.*/
 __O uint32_t CONCLR;                         /*I2C Control Clear Register. When a one is written to a bit of this
                                                register, the corresponding bit in the I2C control register is cleared.
                                                Writing a zero has no effect on the corresponding bit in the I2C control
                                                register*/
 __IO uint32_t MMCTRL;                        /*Monitor mode control register.*/
 __IO uint32_t ADR1;                          /*I2C Slave Address Register 1. Contains the 7-bit slave address for
                                                operation of the I2C interface in slave mode, and is not used in master
                                                mode. The least significant bit determines whether a slave responds to
                                                the General Call address*/
 __IO uint32_t ADR2;                          /*I2C Slave Address Register 2. Contains the 7-bit slave address for
                                                operation of the I2C interface in slave mode, and is not used in master
                                                mode. The least significant bit determines whether a slave responds to
                                                the General Call address.*/
 __IO uint32_t ADR3;                          /*I2C Slave Address Register 3. Contains the 7-bit slave address for
                                                operation of the I2C interface in slave mode, and is not used in master
                                                mode. The least significant bit determines whether a slave responds to
                                                the General Call address.*/
 __I uint32_t DATA_BUFFER;                    /*Data buffer register. The contents of the 8 MSBs of the I2DAT shift
                                                register will be transferred to the I2DATA_BUFFER automatically after
                                                every 9 bits (8 bits of data plus ACK or NACK) has been received on the
                                                bus.*/
 __IO uint32_t MASK0;                         /*I2C Slave address mask register 0. This mask register is associated
                                                with I2ADR0 to determine an address match. The mask register has no
                                                effect when comparing to the General Call address (‘0000000’).*/
 __IO uint32_t MASK1;                         /*I2C Slave address mask register 1. This mask register is associated
                                                with I2ADR0 to determine an address match. The mask register has no
                                                effect when comparing to the General Call address (‘0000000’).*/
 __IO uint32_t MASK2;                         /*I2C Slave address mask register 2. This mask register is associated
                                                with I2ADR0 to determine an address match. The mask register has no
                                                effect when comparing to the General Call address (‘0000000’).*/
 __IO uint32_t MASK3;                         /*I2C Slave address mask register 3. This mask register is associated
                                                with I2ADR0 to determine an address match. The mask register has no
                                                effect when comparing to the General Call address (‘0000000’).*/
}I2C_BUS_INTERFACE_TYPEDEF;

/******************************************************** I2S *************************************************************************/

typedef struct
{
 __IO uint32_t DAO;                            /*Digital Audio Output Register. Contains control bits for the I2S
                                                 transmit channel*/
 __IO uint32_t DAI;                            /*Digital Audio Input Register. Contains control bits for the I2S receive
                                                 channel.*/
 __O uint32_t TXFIFO;                          /*Transmit FIFO. Access register for the 8 ? 32-bit transmitter FIFO*/
 __I uint32_t RXFIFO;                          /*Receive FIFO. Access register for the 8 ? 32-bit receiver FIFO*/
 __I uint32_t STATE;                           /*Status Feedback Register. Contains status information about the
                                                 I2S interface*/
 __IO uint32_t DMA1;                           /*DMA Configuration Register 1. Contains control information for
                                                 DMA request 1.*/
 __IO uint32_t DMA2;                           /*DMA Configuration Register 2. Contains control information for
                                                 DMA request 2.*/
 __IO uint32_t IRQ;                            /*Interrupt Request Control Register. Contains bits that control how
                                                 the I2S interrupt request is generated*/
 __IO uint32_t TXRATE;                         /*Transmit reference clock divider. This register determines the I2S
                                                 TX_REF rate by specifying the value to divide CCLK by in order to
                                                 produce TX_REF*/
 __IO uint32_t RXRATE;                         /*Receive reference clock divider. This register determines the I2S
                                                 RX_REF rate by specifying the value to divide CCLK by in order to
                                                 produce RX_REF.*/
 __IO uint32_t TXBITRATE;                      /*Transmit bit rate divider. This register determines the I2S transmit
                                                 bit rate by specifying the value to divide TX_REF by in order to
                                                 produce the transmit bit clock*/
 __IO uint32_t RXBITRATE;                      /*Receive bit rate divider. This register determines the I2S receive bit
                                                 rate by specifying the value to divide RX_REF by in order to
                                                 produce the receive bit clock.*/
 __IO uint32_t TXMODE;                         /*Transmit mode control*/
 __IO uint32_t RXMODE;                         /*Receive mode control*/
}I2S_TYPEDEF;

/*********************************************** TIMER ********************************************************************************/

typedef struct
{
 __IO uint32_t IR;                             /*Interrupt Register. The IR can be written to clear interrupts. The IR
                                                 can be read to identify which of eight possible interrupt sources are
                                                 pending.*/
 __IO uint32_t TCR;                            /*Timer Control Register. The TCR is used to control the Timer
                                                 Counter functions. The Timer Counter can be disabled or reset
                                                 through the TCR*/
 __IO uint32_t TC;                             /*Timer Counter. The 32 bit TC is incremented every PR+1 cycles of
                                                 PCLK. The TC is controlled through the TCR.*/
 __IO uint32_t PR;                             /*Prescale Register. When the Prescale Counter (PC) is equal to this
                                                 value, the next clock increments the TC and clears the PC.*/
 __IO uint32_t PC;                             /*Prescale Counter. The 32 bit PC is a counter which is incremented
                                                 to the value stored in PR. When the value in PR is reached, the TC
                                                 is incremented and the PC is cleared. The PC is observable and
                                                 controllable through the bus interface*/
 __IO uint32_t MCR;                            /*Match Control Register. The MCR is used to control if an interrupt
                                                 is generated and if the TC is reset when a Match occurs*/
 __IO uint32_t MR0;                            /*Match Register 0. MR0 can be enabled through the MCR to reset
                                                 the TC, stop both the TC and PC, and/or generate an interrupt
                                                 every time MR0 matches the TC.*/
 __IO uint32_t MR1;                            /*Match Register 1. MR1 can be enabled through the MCR to reset
                                                 the TC, stop both the TC and PC, and/or generate an interrupt
                                                 every time MR1 matches the TC.*/
 __IO uint32_t MR2;                            /*Match Register 2. MR0 can be enabled through the MCR to reset
                                                 the TC, stop both the TC and PC, and/or generate an interrupt
                                                 every time MR2 matches the TC.*/
 __IO uint32_t MR3;                            /*Match Register 3. MR3 can be enabled through the MCR to reset
                                                 the TC, stop both the TC and PC, and/or generate an interrupt
                                                 every time MR3 matches the TC.*/
 __IO uint32_t CCR;                            /*Capture Control Register. The CCR controls which edges of the
                                                 capture inputs are used to load the Capture Registers and whether
                                                 or not an interrupt is generated when a capture takes place.*/
 __I uint32_t CR0;                             /*Capture Register 0. CR0 is loaded with the value of TC when there
                                                 is an event on the CAPn.0 input.*/
 __I uint32_t CR1;                             /*Capture Register 1. CR1 is loaded with the value of TC when there
                                                 is an event on the CAPn.0 input.*/
 __IO uint32_t EMR;                            /*External Match Register. The EMR controls the external match
                                                 pins.*/
 __IO uint8_t RESERVED0[0x30];
 __IO uint32_t CTCR;                           /*Count Control Register. The CTCR selects between Timer and
                                                 Counter mode, and in Counter mode selects the signal and
                                                 edge(s) for counting.*/
}TIMER_TYPEDEF;

/******************************************* SYSTEM TICK TIMER **************************************************************************/

typedef struct
{
 __IO uint32_t STCTRL;                        /*System Timer Control and status register*/
 __IO uint32_t STRELOAD;                      /*System Timer Reload value register*/
 __IO uint32_t STCURR;                        /*System Timer Current value register*/
 __IO uint32_t STCALIB;                       /*System Timer Calibration value register*/
}SYSTEM_TICK_TIMER_TYPEDEF;

/********************************************************** PWM ***********************************************************************/

typedef struct
{
 __IO uint32_t IR;                            /*Interrupt Register. The IR can be written to clear interrupts, or read to
                                                identify which PWM interrupt sources are pending*/
 __IO uint32_t TCR;                           /*Timer Control Register. The TCR is used to control the Timer Counter
                                                functions.*/
 __IO uint32_t TC;                            /*Timer Counter. The 32 bit TC is incremented every PR+1 cycles of
                                                PCLK. The TC is controlled through the TCR.*/
 __IO uint32_t PR;                            /*Prescale Register. Determines how often the PWM counter is
                                                incremented.*/
 __IO uint32_t PC;                            /*Prescale Counter. Prescaler for the main PWM counter*/
 __IO uint32_t MCR;                           /*Match Control Register. The MCR is used to control whether an
                                                interrupt is generated and if the PWM counter is reset when a Match
                                                occurs.*/
 __IO uint32_t MR0;                           /*Match Register 0. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t MR1;                           /*Match Register 1. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t MR2;                           /*Match Register 2. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t MR3;                           /*Match Register 3. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t CCR;                           /*Capture Control Register. The CCR controls which edges of the
                                                capture inputs are used to load the Capture Registers and whether or
                                                not an interrupt is generated for a capture event.*/
 __I uint32_t CR0;                            /*Capture Register 0. CR0 of PWMn is loaded with the value of the TC
                                                when there is an event on the PWMn_CAP0 input*/
 __I uint32_t CR1;                            /*Capture Register 1. CR1 of PWMn is loaded with the value of the TC
                                                when there is an event on the PWMn_CAP0 input*/
 __IO uint8_t RESERVED0[0xC];
 __IO uint32_t MR4;                           /*Match Register 4. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t MR5;                           /*Match Register 5. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t MR6;                           /*Match Register 6. Match registers are continuously compared to the
                                                PWM counter in order to control PWM output edges*/
 __IO uint32_t PCR;                           /*PWM Control Register. Enables PWM outputs and selects either
                                                single edge or double edge controlled PWM outputs.*/
 __IO uint32_t LER;                           /*Load Enable Register. Enables use of updated PWM match values*/
 __IO uint8_t RESERVED1[0x1C];
 __IO uint32_t CTCR;                          /*Count Control Register. The CTCR selects between Timer and
                                                Counter mode, and in Counter mode selects the signal and edge(s)
                                                for counting.*/ 
}PWM_TYPEDEF;

/********************************************* MOTOR CONTROL PULSE WIDTH MODULATOR ****************************************************/

typedef struct
{
 __I uint32_t CON;                             /*PWM Control read address*/
 __O uint32_t CON_SET;                         /*PWM Control set address*/
 __O uint32_t CON_CLR;                         /*PWM Control clear address*/
 __I uint32_t CAPCON;                          /*Capture Control read address*/
 __O uint32_t CAPCON_SET;                      /*Capture Control set address*/
 __O uint32_t CAPCON_CLR;                      /*Event Control clear address*/
 __IO uint32_t TC0;                            /*Timer Counter register, channel 0*/
 __IO uint32_t TC1;                            /*Timer Counter register, channel 1*/
 __IO uint32_t TC2;                            /*Timer Counter register, channel 2*/
 __IO uint32_t LIM0;                           /*Limit register, channel 0*/
 __IO uint32_t LIM1;                           /*Limit register, channel 1*/
 __IO uint32_t LIM2;                           /*Limit register, channel 2*/
 __IO uint32_t MAT0;                           /*Match register, channel 0*/
 __IO uint32_t MAT1;                           /*Match register, channel 1*/
 __IO uint32_t MAT2;                           /*Match register, channel 2*/
 __IO uint32_t DT;                             /*Dead time register*/
 __IO uint32_t MCCP;                           /*Communication Pattern register*/
 __I uint32_t CAP0;                            /*Capture register, channel 0*/
 __I uint32_t CAP1;                            /*Capture register, channel 1*/
 __I uint32_t CAP2;                            /*Capture register, channel 2*/
 __I uint32_t INTEN;                           /*Interrupt Enable read address*/
 __O uint32_t INTEN_SET;                       /*Interrupt Enable set address*/
 __O uint32_t INTEN_CLR;                       /*Interrupt Enable clear address*/
 __I uint32_t CNTCON;                          /*Count Control read address*/
 __O uint32_t CNTCON_SET;                      /*Count Control set address*/
 __O uint32_t CNTCON_CLR;                      /*Count Control clear address*/
 __I uint32_t INTF;                            /*Interrupt flags read address*/
 __O uint32_t INTF_SET;                        /*Interrupt flags set address*/
 __O uint32_t INTF_CLR;                        /*Interrupt flags clear address*/
 __O uint32_t CAP_CLR;                         /*Capture clear address*/
}MOTOR_CONTROL_PULSE_WIDTH_MODULATOR_TYPEDEF;

/************************************** Quadrature Encoder Interface (QEI) ************************************************************/

typedef struct
{
 __O uint32_t CON;                             /*Control register*/
 __I uint32_t STAT;                            /*Status register*/
 __IO uint32_t CONF;                           /*Configuration register*/
 __I uint32_t POS;                             /*Position register*/
 __IO uint32_t MAXPOS;                         /*Maximum position register*/
 __IO uint32_t CMPOS0;                         /*Position compare register 0*/
 __IO uint32_t CMPOS1;                         /*Position compare register 1*/
 __IO uint32_t CMPOS2;                         /*Position compare register 2*/
 __I uint32_t INXCNT;                          /*Index count register 0*/
 __IO uint32_t INXCMP0;                        /*Index compare register 0*/
 __IO uint32_t LOAD;                           /*Velocity timer reload register*/
 __I uint32_t TIME;                            /*Velocity timer register*/
 __I uint32_t VEL;                             /*Velocity counter register*/
 __I uint32_t CAP;                             /*Velocity capture register*/
 __IO uint32_t VELCOMP;                        /*Velocity compare register*/
 __IO uint32_t FILTERPHA;                      /*Digital filter register on PHA*/
 __IO uint32_t FILTERPHB;                      /*Digital filter register on PHB*/
 __IO uint32_t FILTERINX;                      /*Digital filter register on IDX*/
 __IO uint32_t WINDOW;                         /*Index acceptance window register*/
 __IO uint32_t INXCMP1;                        /*Index compare register 1*/
 __IO uint32_t INXCMP2;                        /*Index compare register 2*/
 __IO uint8_t RESERVED0[0xF84];
 __O uint32_t IEC;                             /*Interrupt enable clear register*/
 __O uint32_t IES;                             /*Interrupt enable set register*/
 __I uint32_t INTSTAT;                         /*Interrupt status register*/
 __I uint32_t IE;                              /*Interrupt enable register*/
 __O uint32_t CLR;                             /*Interrupt status clear register*/
 __O uint32_t SET;                             /*Interrupt status set register*/
}QEI_TYPEDEF;

/************************************************ REAL TIME CLOCK *********************************************************************/

typedef struct
{
 __IO uint32_t ILR;                            /*Interrupt Location Register*/
 __IO uint32_t RESERVED0;
 __IO uint32_t CCR;                            /*Clock Control Register*/
 __IO uint32_t CIIR;                           /*Counter Increment Interrupt Register*/
 __IO uint32_t AMR;                            /*Alarm Mask Register*/
 __I uint32_t CTIME0;                          /*Consolidated Time Register 0*/
 __I uint32_t CTIME1;                          /*Consolidated Time Register 1*/
 __I uint32_t CTIME2;                          /*Consolidated Time Register 2*/
 __IO uint32_t SEC;                            /*Seconds Counter*/
 __IO uint32_t MIN;                            /*Minutes Register*/
 __IO uint32_t HRS;                            /*Hours Register*/
 __IO uint32_t DOM;                            /*Day of Month Register*/
 __IO uint32_t DOW;                            /*Day of Week Register*/
 __IO uint32_t DOY;                            /*Day of Year Register*/
 __IO uint32_t MONTH;                          /*Months Register*/
 __IO uint32_t YEAR;                           /*Years Register*/
 __IO uint32_t CALIBRATION;                    /*Calibration Value Register*/
 __IO uint32_t GPREG0;                         /*General Purpose Register 0*/
 __IO uint32_t GPREG1;                         /*General Purpose Register 1*/
 __IO uint32_t GPREG2;                         /*General Purpose Register 2*/
 __IO uint32_t GPREG3;                         /*General Purpose Register 3*/
 __IO uint32_t GPREG4;                         /*General Purpose Register 4*/
 __IO uint32_t RTC_AUXEN;                      /*RTC Auxiliary Enable register*/
 __IO uint32_t RTC_AUX;                        /*RTC Auxiliary control register*/
 __IO uint32_t ASEC;                           /*Alarm value for Seconds*/
 __IO uint32_t AMIN;                           /*Alarm value for Minutes*/
 __IO uint32_t AHRS;                           /*Alarm value for Hours*/
 __IO uint32_t ADOM;                           /*Alarm value for Day of Month*/
 __IO uint32_t ADOW;                           /*Alarm value for Day of Week*/
 __IO uint32_t ADOY;                           /*Alarm value for Day of Year*/
 __IO uint32_t AMON;                           /*Alarm value for Months*/
 __IO uint32_t AYRS;                           /*Alarm value for Year*/
}REAL_TIME_CLOCK_TYPEDEF;

/********************************************* EVENT MONITOR/RECORDER *****************************************************************/

typedef struct
{
 __IO uint32_t ERSTATUS;                        /*Event Monitor/Recorder Status register. Contains status flags for
                                                  event channels and other Event Monitor/Recorder conditions.*/
 __IO uint32_t ERCONTROL;                       /*Event Monitor/Recorder Control register. Contains bits that
                                                  control actions for the event channels as well as for Event
                                                  Monitor/Recorder setup.*/
 __I uint32_t ERCOUNTERS;                       /*Event Monitor/Recorder Counters register. Allows reading the
                                                  counters associated with the event channels*/
 __IO uint32_t RESERVED0;
 __I uint32_t ERFIRSTSTAMP0;                    /*Event Monitor/Recorder First Stamp register for channel 0.
                                                  Retains the time stamp for the first event on channel 0.*/
 __I uint32_t ERFIRSTSTAMP1;                    /*Event Monitor/Recorder First Stamp register for channel 1.
                                                  Retains the time stamp for the first event on channel 1.*/
 __I uint32_t ERFIRSTSTAMP2;                    /*Event Monitor/Recorder First Stamp register for channel 2.
                                                  Retains the time stamp for the first event on channel 2.*/
 __IO uint32_t RESERVED1;
 __I uint32_t ERLASTSTAMP0;                     /*Event Monitor/Recorder Last Stamp register for channel 0.
                                                  Retains the time stamp for the last (i.e. most recent) event on
                                                  channel 0*/
 __I uint32_t ERLASTSTAMP1;                     /*Event Monitor/Recorder Last Stamp register for channel 1.
                                                  Retains the time stamp for the last (i.e. most recent) event on
                                                  channel 1*/
 __I uint32_t ERLASTSTAMP2;                     /*Event Monitor/Recorder Last Stamp register for channel 2.
                                                  Retains the time stamp for the last (i.e. most recent) event on
                                                  channel 2*/																									
}EVENT_MONITOR_OR_RECORDER_TYPEDEF;

/*************************************************** WATCHDOG *************************************************************************/

typedef struct
{
 __IO uint32_t MOD;                             /*Watchdog mode register. This register determines the basic
                                                  mode and status of the Watchdog Timer.*/
 __IO uint32_t TC;                              /*Watchdog timer constant register. The value in this register
                                                  determines the time-out value.*/
 __O uint32_t FEED;                             /*Watchdog feed sequence register. Writing 0xAA followed by
                                                  0x55 to this register reloads the Watchdog timer with the
                                                  value contained in WDTC.*/
 __I uint32_t TV;                               /*Watchdog timer value register. This register reads out the
                                                  current value of the Watchdog timer.*/
 __IO uint32_t RESERVED0;
 __IO uint32_t WARNINT;                         /*Watchdog Warning Interrupt compare value*/ 
 __IO uint32_t WINDOW;                          /*Watchdog Window compare value.*/
}WATCHDOG_TYPEDEF;

/********************************************************** ADC ***********************************************************************/

typedef struct
{
 __IO uint32_t CR;                              /*A/D Control Register. The ADCR register must be written to select
                                                  the operating mode before A/D conversion can occur*/
 __IO uint32_t GDR;                             /*A/D Global Data Register. This register contains the ADC’s DONE
                                                  bit and the result of the most recent A/D conversion.*/
 __IO uint32_t RESERVED0;
 __IO uint32_t INTEN;                           /*A/D Interrupt Enable Register. This register contains enable bits
                                                  that allow the DONE flag of each A/D channel to be included or
                                                  excluded from contributing to the generation of an A/D interrupt.*/
 __I uint32_t DR0;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 0.*/
 __I uint32_t DR1;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 1.*/
 __I uint32_t DR2;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 2.*/
 __I uint32_t DR3;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 3.*/
 __I uint32_t DR4;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 4.*/
 __I uint32_t DR5;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 5.*/
 __I uint32_t DR6;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 6.*/
 __I uint32_t DR7;                              /*A/D Channel 0 Data Register. This register contains the result of
                                                  the most recent conversion completed on channel 7.*/
 __I uint32_t STAT;                             /*A/D Status Register. This register contains DONE and OVERRUN
                                                  flags for all of the A/D channels, as well as the A/D interrupt/DMA
                                                  flag.*/
 __IO uint32_t TRM;                             /*ADC trim register*/
}ADC_TYPEDEF;

/**************************************************** DAC *****************************************************************************/

typedef struct
{
 __IO uint32_t CR;                              /*D/A Converter Register. This register contains the digital value
                                                  to be converted to analog and a power control bit.*/
 __IO uint32_t CTRL;                            /*DAC Control register. This register controls DMA and timer
                                                  operation*/
 __IO uint32_t CNTVAL;                          /*DAC Counter Value register. This register contains the reload
                                                  value for the DAC DMA/Interrupt timer.*/
}DAC_TYPEDEF;

/************************************************* COMPARATOR *************************************************************************/

typedef struct 
{
 __IO uint32_t CMP_CTRL;                        /*Comparator block control register*/
 __IO uint32_t CMP_CTRL0;                       /*Comparator 0 control register*/
 __IO uint32_t CMP_CTRL1;                       /*Comparator 1 control register*/
}COMPARATOR_TYPEDEF;

/********************************************************* GPDMA **********************************************************************/

typedef struct
{
 __I uint32_t INTSTAT;                         /* DMA Interrupt Status Register*/
 __I uint32_t INTTCSTAT;                       /* DMA Interrupt Terminal Count Request Status Register*/
 __O uint32_t INTTCCLEAR;                      /*DMA Interrupt Terminal Count Request Clear Register*/
 __I uint32_t INTERRSTAT;                      /* DMA Interrupt Error Status Register*/
 __O uint32_t INTERRCLR;                       /* DMA Interrupt Error Clear Register*/
 __I uint32_t RAWINTTCSTAT;                    /* DMA Raw Interrupt Terminal Count Status Register*/
 __I uint32_t RAWINTERRSTAT;                   /* DMA Raw Error Interrupt Status Register*/
 __I uint32_t ENBLDCHNS;                       /* DMA Enabled Channel Register*/
 __IO uint32_t SOFTBREQ;                       /*DMA Software Burst Request Register*/
 __IO uint32_t SOFTSREQ;                       /* DMA Software Single Request Register*/
 __IO uint32_t SOFTLBREQ;                      /* DMA Software Last Burst Request Register*/
 __IO uint32_t SOFTLSREQ;                      /*DMA Software Last Single Request Register*/ 
 __IO uint32_t CONFIG;                         /* DMA Configuration Register*/
 __IO uint32_t SYNC;                           /*DMA Synchronization Register*/
 __IO uint8_t RESERVED0[0xC8];             
 __IO	uint32_t SRCADDR0;                       /*DMA Channel 0 Source Address Register*/
 __IO uint32_t DESTADDR0;                      /*DMA Channel 0 Destination Address Register*/
 __IO uint32_t LLI0;                           /*DMA Channel 0 Linked List Item Register*/
 __IO uint32_t CONTROL0;                       /*DMA Channel 0 Control Register*/
 __IO uint32_t CONFIG0;                        /*DMA Channel 0 Configuration Register*/
 __IO uint8_t RESERVED1[0xC];
 __IO	uint32_t SRCADDR1;                       /*DMA Channel 1 Source Address Register*/
 __IO uint32_t DESTADDR1;                      /*DMA Channel 1 Destination Address Register*/
 __IO uint32_t LLI1;                           /*DMA Channel 1 Linked List Item Register*/
 __IO uint32_t CONTROL1;                       /*DMA Channel 1 Control Register*/
 __IO uint32_t CONFIG1;                        /*DMA Channel 1 Configuration Register*/
 __IO uint8_t RESERVED2[0xC];
 __IO	uint32_t SRCADDR2;                       /*DMA Channel 2 Source Address Register*/
 __IO uint32_t DESTADDR2;                      /*DMA Channel 2 Destination Address Register*/
 __IO uint32_t LLI2;                           /*DMA Channel 2 Linked List Item Register*/
 __IO uint32_t CONTROL2;                       /*DMA Channel 2 Control Register*/
 __IO uint32_t CONFIG2;                        /*DMA Channel 2 Configuration Register*/
 __IO uint8_t RESERVED3[0xC];
 __IO	uint32_t SRCADDR3;                       /*DMA Channel 3 Source Address Register*/
 __IO uint32_t DESTADDR3;                      /*DMA Channel 3 Destination Address Register*/
 __IO uint32_t LLI3;                           /*DMA Channel 3 Linked List Item Register*/
 __IO uint32_t CONTROL3;                       /*DMA Channel 3 Control Register*/
 __IO uint32_t CONFIG3;                        /*DMA Channel 3 Configuration Register*/
 __IO uint8_t RESERVED4[0xC];
 __IO	uint32_t SRCADDR4;                       /*DMA Channel 4 Source Address Register*/
 __IO uint32_t DESTADDR4;                      /*DMA Channel 4 Destination Address Register*/
 __IO uint32_t LLI4;                           /*DMA Channel 4 Linked List Item Register*/
 __IO uint32_t CONTROL4;                       /*DMA Channel 4 Control Register*/
 __IO uint32_t CONFIG4;                        /*DMA Channel 4 Configuration Register*/
 __IO uint8_t RESERVED5[0xC];
 __IO	uint32_t SRCADDR5;                       /*DMA Channel 5 Source Address Register*/
 __IO uint32_t DESTADDR5;                      /*DMA Channel 5 Destination Address Register*/
 __IO uint32_t LLI5;                           /*DMA Channel 5 Linked List Item Register*/
 __IO uint32_t CONTROL5;                       /*DMA Channel 5 Control Register*/
 __IO uint32_t CONFIG5;                        /*DMA Channel 5 Configuration Register*/
 __IO uint8_t RESERVED6[0xC];
 __IO	uint32_t SRCADDR6;                       /*DMA Channel 6 Source Address Register*/
 __IO uint32_t DESTADDR6;                      /*DMA Channel 6 Destination Address Register*/
 __IO uint32_t LLI6;                           /*DMA Channel 6 Linked List Item Register*/
 __IO uint32_t CONTROL6;                       /*DMA Channel 6 Control Register*/
 __IO uint32_t CONFIG6;                        /*DMA Channel 6 Configuration Register*/
 __IO uint8_t RESERVED7[0xC];
 __IO	uint32_t SRCADDR7;                       /*DMA Channel 7 Source Address Register*/
 __IO uint32_t DESTADDR7;                      /*DMA Channel 7 Destination Address Register*/
 __IO uint32_t LLI7;                           /*DMA Channel 7 Linked List Item Register*/
 __IO uint32_t CONTROL7;                       /*DMA Channel 7 Control Register*/
 __IO uint32_t CONFIG7;                        /*DMA Channel 7 Configuration Register*/
}GPDMA_TYPEDEF;

/************************************************* CRC ENGINE *************************************************************************/

typedef struct
{
 __IO uint32_t MODE;                          /*CRC mode register*/
 __IO uint32_t SEED;                          /* CRC seed register*/
	union{
 __I uint32_t SUM;                            /*CRC checksum register*/
 __O uint32_t DATA;                           /*CRC data register*/
	}CRC_UNION;
}CRC_ENGINE_TYPEDEF;

/******************************************* EEPROM CONTROLLER ************************************************************************/

typedef struct
{
 __IO uint32_t CMD;                          /* EEPROM command register*/
 __IO uint32_t ADDR;                         /*EEPROM address register*/
 __O uint32_t WDATA;                         /*EEPROM write data register*/
 __I uint32_t RDATA;                         /*EEPROM read data register*/
 __IO uint32_t WSTATE;                       /*EEPROM wait state register*/
 __IO uint32_t CLKDIV;                       /*EEPROM clock divider register*/
 __IO uint32_t PWRDWN;                       /*EEPROM power-down register*/
 __IO uint8_t RESERVED[0xF3C];
 __O uint32_t INTENCLR;                      /*EEPROM interrupt enable clear*/
 __O uint32_t INTENSET;                      /*EEPROM interrupt enable set*/
 __I uint32_t INTSTAT;                       /*EEPROM interrupt status*/
 __I uint32_t INTEN;                         /*EEPROM interrupt enable*/
 __I uint32_t INTSTATCLR;                    /*EEPROM interrupt status clear*/
 __I uint32_t INTSTATSET;                    /*EEPROM interrupt status set*/
}EEPROM_CONTROLER_TYPEDEF;

/****************************************** FLASH CONTROLLER **************************************************************************/

typedef struct
{
 __IO uint32_t FMSSTART;                     /*Signature start address register*/
 __IO uint32_t FMSSTOP;                      /*Signature stop-address register*/
 __I uint32_t FMSW0;                         /* 128-bit signature Word 0*/
 __I uint32_t FMSW1;                         /*128-bit signature Word 1 */
 __I uint32_t FMSW2;                         /*128-bit signature Word 2*/
 __I uint32_t FMSW3;                         /*128-bit signature Word 3*/
 __IO uint8_t RESERVED0[0xFA4];
 __I uint32_t STAT;                          /*Signature generation status register*/
 __IO uint32_t RESERVED1;
 __O uint32_t STATCLR;                       /*Signature generation status clear register*/
}FLASH_CONTROLLER_TYPEDEF;

/************************************************************ BASE ADDRESS *************************************************************/

#define SYSTEM_CONTROL                                       (SYSTEM_CONTROL_TYPEDEF*)0x400FC080
#define FLASH_CONTROL                                        (FLASH_CONTROL_TYPEDEF*)0x400FC000
#define INTERRUPT_SET_ENABLE                                 (INTERRUPT_SET_ENABLE_TYPEDEF*)0xE000E100
#define INTERRUPT_CLEAR_ENABLE                               (INTERRUPT_CLEAR_ENABLE_TYPEDEF*)0xE000E180
#define INTERRUPT_SET_PENDING                                (INTERRUPT_SET_PENDING_TYPEDEF*)0xE000E200
#define INTERRUPT_CLEAR_PENDING                              (INTERRUPT_CLEAR_PENDING_TYPEDEF*)0xE000E280
#define INTERRUPT_ACTIVE_BIT                                 (INTERRUPT_ACTIVE_BIT_TYPEDEF*)0xE000E300
#define INTERRUPT_POLARITY                                   (INTERRUPT_POLARITY_TYPEDEF*)0xE000E400
#define SOFTWARE_TRIGGER_INTERRUPT                           (SOFTWARE_TRIGGER_INTERRUPT_TYPEDEF*)0xE000EF00
#define IOCON                                                (IOCON_TYPEDEF*)0x4002C000
#define GPIO                                                 (GPIO_TYPEDEF*)0x20098000
#define GPIO_INTERRUPT                                       (GPIO_INTERRUPT_TYPEDEF*)0x40028080
#define EMC                                                  (EMC_TYPEDEF*)0x2009C000
#define ETHERNET                                             (ETHERNET_TYPEDEF*)0x20084000
#define LCD_CONTROLLER                                       (LCD_CONTROLLER_TYPEDEF*)0x20088000
#define USB_DEVICE_CONTROLLER                                (USB_DEVICE_CONTROLLER_TYPEDEF*)0x2008C110
#define USB_HOST_CONTROLLER                                  (USB_HOST_CONTROLLER_TYPEDEF*)0x2008C000
#define USB_OTG_CONTROLLER                                   (USB_OTG_CONTROLLER_TYPEDEF*)0x2008C100
#define SD_CARD_INTERFACE                                    (SD_CARD_INTERFACE_TYPEDEF*)0x0x400C0000
#define UART1                                                (UART1_TYPEDEF*)0x40010000
#define UART0                                                (UART0_TYPEDEF*)0x4000C000
#define UART2                                                (UART2_TYPEDEF*)0x40088000
#define UART3                                                (UART3_TYPEDEF*)0x4009C000
#define UART4                                                (UART4_TYPEDEF*)0x400A4000
#define CAN_ACCEPTANCE_FILTER                                (CAN_ACCEPTANCE_FILTER_TYPEDEF*)0x4003C000
#define CENTRAL_CAN                                          (CENTRAL_CAN_TYPEDEF*)0x40040000
#define CAN1                                                 (CAN1_TYPEDEF*)0x40044000
#define CAN2                                                 (CAN2_TYPEDEF*)0x40048000
#define CAN_WAKE_AND_SLEEP                                   (CAN_WAKE_AND_SLEEP_TYPEDEF*)0x400FC000
#define SSP0                                                 (SSP_TYPEDEF*)0x40088000
#define SSP1                                                 (SSP_TYPEDEF*)0x40030000
#define SSP2                                                 (SSP_TYPEDEF*)0x400AC000
#define I2C0                                                 (I2C_BUS_INTERFACE_TYPEDEF*)0x4001C000
#define I2C1                                                 (I2C_BUS_INTERFACE_TYPEDEF*)0x4005C000
#define I2C2                                                 (I2C_BUS_INTERFACE_TYPEDEF*)0x400A0000
#define I2S                                                  (I2S_TYPEDEF*)0x400A8000
#define TIMER0                                               (TIMER_TYPEDEF*)0x40004000
#define TIMER1                                               (TIMER_TYPEDEF*)0x40008000
#define TIMER2                                               (TIMER_TYPEDEF*)0x40090000
#define TIMER3                                               (TIMER_TYPEDEF*)0x40094000
#define SYSTEM_TICK_TIMER                                    (SYSTEM_TICK_TIMER_TYPEDEF*)0xE000E010
#define PWM0                                                 (PWM_TYPEDEF)0x40014000
#define PWM1                                                 (PWM_TYPEDEF)0x40018000
#define MOTOR_CONTROL_PULSE_WIDTH_MODULATOR                  (MOTOR_CONTROL_PULSE_WIDTH_MODULATOR_TYPEDEF*)0x400B8000)
#define QEI                                                  (QEI_TYPEDEF*)0x400BC000
#define REAL_TIME_CLOCK                                      (REAL_TIME_CLOCK_TYPEDEF*)0x40024000
#define EVENT_MONITOR_OR_RECORDER                            (EVENT_MONITOR_OR_RECORDER_TYPEDEF*)0x40024000
#define WATCHDOG                                             (WATCHDOG_TYPEDEF*)0x40000000
#define ADC                                                  (ADC_TYPEDEF*)0x40034000
#define DAC                                                  (DAC_TYPEDEF*)0x4008C000
#define COMPARATOR                                           (COMPARATOR_TYPEDEF*)0x40020000
#define GPDMA                                                (GPDMA_TYPEDEF*)0x20080000                   
#define CRC_ENGINE                                           (CRC_ENGINE_TYPEDEF*)0x20090000
#define EEPROM_CONTROLLER                                    (EEPROM_CONTROLLER_TYPEDEF*)0x00200080
#define FLASH_CONTROLLER                                     (FLASH_CONTROLLER_TYPEDEF*)0x00200020