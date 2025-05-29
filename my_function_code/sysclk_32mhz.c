/*
 * sysclk_32MHz.c
 *
 *  Created on: 29.5.2025
 *      Author: thong
 */


void SetSysClk(void)
{
	uint32_t status = 0;

  /*1. Enable HSI High speed internal clock*/
  RCC->CR |= RCC_CR_HSION;

  /*2. Waits for the HSI to stabilize. */
 while(!(RCC->CR & RCC_CR_HSIRDY)){} //CR bit 1 HSIRDY sets when HSI oscillator is stable. p141
 status = (RCC->CR & RCC_CR_HSIRDY) ? 1 : 0; //if CR bit 1 HSIDRY high when oscillator is stable.

 if (status == 1) //If HSI ready
  {
    /*3.  PLL (Phase-Locked Loop)configuration to generate 32MHz*/
	  //PLLSCR bit 16, PLLMUL bits 18-21, PLLDIV bits 22,23.
	  //0000 0000 1111 1101 0000 0000 0000 0000 corresponding bits
	  //these bits PLLSRC, PLLMUL and PLLDIV cleared
	  RCC->CFGR &= ~(0x00FD0000);

	  //PLLMUL must be 0001 when multiplication=4 and PLLDIV must be 01 when division=2
	  //PLLCLK = (HSI * 4)/2 = 32 MHz p144/911
	  //PLLSCR bit 16 must be 1 --> HSE oscillator clock selected as PLL input clock (done in previous line)
	  //0000 0000 0100 0100 0000 0000 0000 0000
	  RCC->CFGR |= 0x00440000;
  }

  else
  {
    /* If HSI fails to start-up, the application will have wrong clock configuration. EX: toggle LED*/
  }

 /*4. Configures the Flash memory for optimal performance at 32 MHz by enabling 64-bit access, prefetch, and setting one wait state. */
   FLASH->ACR |= FLASH_ACR_ACC64; //ACC64 bit 2, 64-bit access. 64-bit access is used to improve the performance. p84
     /*Prefetch is enabled by setting the PRFTEN bit in the FLASH_ACR register.
     *This feature is useful if at least one wait state is needed to access the Flash memory.
 	*Figure 5 shows the execution of sequential 32-bit instructions*/
   FLASH->ACR |= FLASH_ACR_PRFTEN; //PRFTEN bit 1, prefetch enable. p84
   FLASH->ACR |= FLASH_ACR_LATENCY; //LATENCY one wait state bit 0. One wait state enabled. p84. p59

	/*5. Enables the power interface clock and configures the voltage regulator to supply 1.8V, waiting for it to stabilize.*/
	 RCC->APB1ENR |= RCC_APB1ENR_PWREN; //bit 28 PWREN: Power interface clock enable. p158. p101
	 PWR->CR = PWR_CR_VOS_0; //Bits 12:11 VOS[1:0]: Voltage scaling range selection, 01: 1.8 V (range 1). p121
	
	 /*A delay is required for the internal regulator to be ready after the voltage range is changed.
	The VOSF bit indicates that the regulator has reached the voltage level defined with bits VOS
	of PWR_CR register. p102 1.8V needed for 32 MHz clock and lower voltages to save power.
	
	0: Regulator is ready in the selected voltage range
	1: Regulator voltage output is changing to the required VOS level.
	*/
	 while((PWR->CSR & PWR_CSR_VOSF)){} //bit 4 VOSF: Voltage Scaling select flag. p125
	
	/*6. No-prescalers for the AHB, APB1, and APB2 buses, effectively setting them to run at the same frequency as the system clock.*/
	RCC->CFGR &= ~RCC_CFGR_HPRE_3; //Bits 7:4 HPRE[3:0]: AHB prescaler. 0xxx: SYSCLK not divided. p144
	RCC->CFGR &= ~RCC_CFGR_PPRE2_2; //Bits 13:11 PPRE2[2:0]: APB high-speed prescaler (APB2). p144
	RCC->CFGR &= ~RCC_CFGR_PPRE1_2; //Bits 10:8 PPRE1[2:0]: APB low-speed prescaler (APB1)p. 144
	
	/*7. Enables the PLL.*/
	RCC->CR |= RCC_CR_PLLON; //Bit 24 PLLON: PLL enable. p140
	
	/*8. Waits for the PLL to stabilize. */
	while(!(RCC->CR & RCC_CR_PLLRDY)){} //Bit 25 PLLRDY: PLL clock ready flag. p140
	
	/*9. Switches the system clock source to the configured PLL output (32 MHz).*/
	RCC->CFGR |= 0x3U; //Bits 1:0 SW[1:0]: System clock switch, 11: PLL used as system clock
	
	/*Bits 3:2 SWS[1:0]: System clock switch status, 11: PLL used as system clock.
	 * These bits are set and cleared by hardware to indicate which clock source is used as
	system clock. 11: PLL used as system clock. p145.
	 Waits for the system clock to confirm it's using the PLL. */
	while (!(RCC->CFGR & RCC_CFGR_SWS)){}
}
