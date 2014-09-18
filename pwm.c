#include "pwm.h"
#include <p30fxxxx.h>

/* PWM1-3 are defined in following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/***********************************************************************
* Function Name     : CloseMCPWM
* Description       : This function Clears the Interrupt enable ,flag 
*                     bits, PTCON, PWMCON1 and PWMCON2 registers. 
* Parameters        : void
* Return Value      : void 
**********************************************************************/
void CloseMCPWM(void)
{
    /* clear the Interrupt enables */
    IEC2bits.PWMIE = 0;	
    IEC2bits.FLTAIE = 0;	

#if defined(__dsPIC30F6010a2__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)
    IEC2bits.FLTBIE = 0;	
#endif

    /* clear the Interrupt flags */
    IFS2bits.PWMIF = 0;	
    IFS2bits.FLTAIF = 0;	

#if defined(__dsPIC30F6010a2__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)
    IFS2bits.FLTBIF = 0;	
#endif

    /* clear the PWM control registers */
    PTCON       =       0;
    PWMCON1     =       0;
    PWMCON2     =       0;
}

#endif

/* PWM1-3 are defined in following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/**********************************************************************
* Function Name     :ConfigIntMCPWM
* Description       :This function Enable/Disable interrupts and 
*                    sets Interrupt priority for period match, 
*                    FaultA and FaultB.  
* Parameters        :unsigned int Config
* Return Value      :None 
**********************************************************************/
void ConfigIntMCPWM(unsigned int config)
{
    /* clear the Interrupt flags */
    IFS2bits.PWMIF = 0;	
    IFS2bits.FLTAIF = 0;	

    /* Set priority for the period match */
    IPC9bits.PWMIP      = (0x0007 & config);

    /* Set priority for the Fault A */
    IPC10bits.FLTAIP    = (0x0070 & config)>> 4;

    /* enable /disable of interrupt Period match */
    IEC2bits.PWMIE      = (0x0008 & config) >> 3;

    /* enable /disable of interrupt Fault A.*/
    IEC2bits.FLTAIE     = (0x0080 & config) >> 7;

#if defined(__dsPIC30F6010a2__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)
    /* clear the Interrupt flags */
    IFS2bits.FLTBIF = 0;	

    /* Set priority for the Fault B */
    IPC11bits.FLTBIP    = (0x0700 & config)>>8;

    /* enable /disable of interrupt Fault B.*/
    IEC2bits.FLTBIE     = (0x0800 & config) >> 11;

#endif

}

#endif

/* PWM1-3 are defined in following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/*********************************************************************
* Function Name     : OpenMCPWM
* Description       : This function configures PWM module for the 
*                     following parameters:
*                     period, sptime, PWM Mode, Clock Prescale,
*                     Output Postscale, high res mode, I/O pair mode, 
*                     I/O pair mode,I/O pair enable, Special event 
*                     postscale, Special event direction, override 
*                     synchronization.
* Parameters        : unsigned int period
*                     unsigned int sptime
*                     unsigned int config1
*                     unsigned int config2, 
*                     unsigned int config3
* Return Value      : None 
**********************************************************************/
void OpenMCPWM(unsigned int period, unsigned int sptime, unsigned int 
               config1, unsigned int config2, unsigned int config3)
{
    PTPER   = period;
    SEVTCMP = sptime;
    PWMCON1 = config2;
    PWMCON2 = config3;
    PTCON   = config1;
}

#endif

/* PWM1-3 are defined in following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/*********************************************************************
* Function Name     : OverrideMCPWM
* Description       : This function set OVDCON register includes PWM 
*                     output override bits and PWM Manual Output Bits
* Parameters        : unsigned int config
* Return Value      : None 
*********************************************************************/
void OverrideMCPWM(unsigned int config)
{
    OVDCON = config;
}

#endif


/* PWM1-3 are defined in following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/*********************************************************************
* Function Name     : SetDCMCPWM
* Description       : This function updates the dutycycle register and 
*                     updatedisable bit.
* Parameters        : unsigned int dutycyclereg for selection of reg
*                      (ie PDC1, PDC2...)
*                     unsigned int dutycycle
*                     char updatedisable
* Return Value      : None 
**********************************************************************/
void SetDCMCPWM(unsigned int dutycyclereg, unsigned int dutycycle,
                char updatedisable)
{
    PWMCON2bits.UDIS = updatedisable & 0x1;
    
    /* Assign dutycycle to the duty cycle register */
    *(&PDC1+dutycyclereg -1) = dutycycle; 
}

#endif

#if defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/*************************************************************************
* Function Name     : SetMCPWMDeadTimeAssignment
* Description       : This function configures the assignment of dead time 
*                     units to PWM output pairs                        
* Parameters        : unsigned int config
* Return Value      : None 
**************************************************************************/
void SetMCPWMDeadTimeAssignment(unsigned int config)
{
    DTCON2 = config ; 
}

#endif

#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/***********************************************************************
* Function Name     : SetMCPWMDeadTimeSourceGeneration 
* Description       : This function configures dead time values and clock                       
*					  prescalers.
* Parameters        : unsigned int config
* Return Value      : None 
************************************************************************/
void SetMCPWMDeadTimeGeneration (unsigned int config)
{
    DTCON1 = config;
}

#endif


/* PWM1-3 are defined in following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F4012__) || \
    defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/**********************************************************************
* Function Name     : SetMCPWMFaultA
* Description       : This function sets Fault A override and enables 
*                     for pins of PWM     
* Parameters        : unsigned int config includes the FAULT A override
*                     value, 
*                     Fault A mode, Fault A Pairs Enable
* Return Value      : None
**********************************************************************/
void SetMCPWMFaultA(unsigned int config)
{
    FLTACON = config;
}

#endif


/* PWM1-4 are defined in following devices */
#if defined(__dsPIC30F6010__) || defined(__dsPIC30F5015__)

/**********************************************************************
* Function Name     : SetMCPWMFaultB
* Description       : This function sets Fault B override and enables 
*                     for pins of PWM     
* Parameters        : unsigned int config includes the FAULT B override
*                     value, 
*                     Fault B mode, Fault B Pairs Enable
* Return Value      : None 
***********************************************************************/
void SetMCPWMFaultB(unsigned int config)
{
   FLTBCON = config; 
}

#endif



