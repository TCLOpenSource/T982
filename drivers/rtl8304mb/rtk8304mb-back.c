#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h> 
#include <linux/fs.h> 
#include <linux/types.h> 
#include <linux/moduleparam.h> 

#include <linux/sched.h>  
#include <linux/kernel.h>
#include <linux/err.h>  
#include <linux/delay.h>
 
struct task_struct *tthread = NULL;

#include "rtk_api.h"
#include "rtk_api_ext.h"
#include "rtl8309n_types.h"
#include "rtl8309n_asicdrv_ext.h"

#define TPRINT printk
#if 1
#include <linux/types.h>
#include "mhal_gpio_reg.h"
#include "mdrv_gpio.h"
#define REG_ETH0_LED_MODE 0x101eb4UL
#define _BIT0    (1<<0)
#define _DISABLE 0
#define _ENABLE 1

//#include "gpio.h"
//#include "mdcmdio.h"      /*RTL8651B file*/
#include "rtl8309n_asictypes.h"
#include "rtl8309n_types.h"
#define DELAY 100
#define SMI_HIGHSPEED 0
#if (!SMI_HIGHSPEED)
#define CLK_DURATION(clk)   { uint32 i; for (i=0; i< (clk); i++); }

uint32 smi_MDC;         /* GPIO used for SMI Clock Generation */
uint32 smi_MDIO;       /* GPIO used for SMI Data signal */
#endif

#define MS838_8304MB
#ifdef MS838_8304MB

#include <linux/spinlock.h>

#define MDC_PIN 82
#define MDIO_PIN 81

spinlock_t ms838_8304mb_gpio_lock;
unsigned long flags;
/* Is it required to disable interrupt? */
void rtlglue_drvMutexLock(void)
{
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	spin_lock_irqsave(&ms838_8304mb_gpio_lock, flags); 
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
}
void rtlglue_drvMutexUnlock(void)
{
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	spin_unlock_irqrestore(&ms838_8304mb_gpio_lock, flags);
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
}

#endif


#if (!SMI_HIGHSPEED)

/*
 *Low speed smi is a general MDC/MDIO interface, it is realized by call gpio api 
 *function, could specified any gpio pin as MDC/MDIO
 */
static void _smiZbit(void)
{
#ifdef MS838_8304MB
	MDrv_GPIO_Pull_Low(smi_MDC);
	MDrv_GPIO_Pull_Low(smi_MDIO);
	CLK_DURATION(DELAY);
#else
    _rtl865x_initGpioPin(smi_MDIO, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);
    _rtl865x_setGpioDataBit(smi_MDC, 0);
    _rtl865x_setGpioDataBit(smi_MDIO, 0);
    CLK_DURATION(DELAY);
#endif
}

/*Generate  1 -> 0 transition and sampled at 1 to 0 transition time,
 *should not sample at 0->1 transition because some chips stop outputing
 *at the last bit at rising edge
 */
static void _smiReadBit(uint32 * pdata) 
{
#ifdef MS838_8304MB

	uint32 u;
    MDrv_GPIO_Pad_Odn(smi_MDIO);

	MDrv_GPIO_Pull_High(smi_MDC);
	CLK_DURATION(DELAY);
	MDrv_GPIO_Pull_Low(smi_MDC);
	u = MDrv_GPIO_Pad_Read(smi_MDIO);
	*pdata = u;
	CLK_DURATION(DELAY);

#else

    uint32 u;
    
    _rtl865x_initGpioPin(smi_MDIO, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);
    _rtl865x_setGpioDataBit(smi_MDC, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_MDC, 0);
    _rtl865x_getGpioDataBit(smi_MDIO, &u);
    *pdata = u;
	CLK_DURATION(DELAY);
#endif
}



/* Generate  0 -> 1 transition and put data ready during 0 to 1 whole period */
static void _smiWriteBit(uint32 data) 
{
#ifdef MS838_8304MB
	MDrv_GPIO_Pad_Oen(smi_MDIO);

	if(data)
		MDrv_GPIO_Pull_High(smi_MDIO);
	else
		MDrv_GPIO_Pull_Low(smi_MDIO);

	MDrv_GPIO_Pull_Low(smi_MDC);
	CLK_DURATION(DELAY);
	MDrv_GPIO_Pull_High(smi_MDC);
	CLK_DURATION(DELAY);
	
#else

    _rtl865x_initGpioPin(smi_MDIO, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (data) 
    {  /*write 1*/
        _rtl865x_setGpioDataBit(smi_MDIO, 1);
        _rtl865x_setGpioDataBit(smi_MDC, 0);
        CLK_DURATION(DELAY);
        _rtl865x_setGpioDataBit(smi_MDC, 1);
		CLK_DURATION(DELAY);
    } 
    else 
    {
        _rtl865x_setGpioDataBit(smi_MDIO, 0);
        _rtl865x_setGpioDataBit(smi_MDC, 0);
        CLK_DURATION(DELAY);
        _rtl865x_setGpioDataBit(smi_MDC, 1);
		CLK_DURATION(DELAY);
    }
#endif
}


/* Function Name:
 *      smiRead
 * Description:
 *      Read data from phy register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 * Output:
 *      data    -  Register value 
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *     This function could read register through MDC/MDIO serial 
 *     interface, and it is platform  related. It use two GPIO pins 
 *     to simulate MDC/MDIO timing. MDC is sourced by the Station Management 
 *     entity to the PHY as the timing reference for transfer of information
 *     on the MDIO signal. MDC is an aperiodic signal that has no maximum high 
 *     or low times. The minimum high and low times for MDC shall be 160 ns each, 
 *     and the minimum period for MDC shall be 400 ns. Obeying frame format defined
 *     by IEEE802.3 standard, you could access Phy registers. If you want to 
 *     port it to other CPU, please modify static functions which are called 
 *      by this function.
 */
int32 smiRead(uint32 phyad, uint32 regad, uint32 * data) 
{

    int32 i;
    uint32 readBit;

    if ((phyad > 8) || (regad > 31) || (data == NULL))  
        return FAILED;

    /*it lock the resource to ensure that SMI opertion is atomic, 
     *the API is based on RTL865X, it is used to disable CPU interrupt,
     *if porting to other platform, please rewrite it to realize the same function
     */

    //rtlglue_drvMutexLock();   

    /* 32 continuous 1 as preamble*/
    for(i=0; i<32; i++)
        _smiWriteBit(1);

    /* ST: Start of Frame, <01>*/
    _smiWriteBit(0);
    _smiWriteBit(1);
    
    /* OP: Operation code, read is <10>*/
    _smiWriteBit(1);
    _smiWriteBit(0);
    
    /* PHY Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((phyad>>i)&0x1);
    
    /* Register Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((regad>>i)&0x1);
    
    /* TA: Turnaround <z0>, zbit has no clock in order to steal a clock to
     *  sample data at clock falling edge 
     */
    _smiZbit();
	//_smiWriteBit(0);
    _smiReadBit(&readBit);

    /* Data */
    *data = 0;
    for(i=15; i>=0; i--) 
    {
        _smiReadBit(&readBit);
        *data = ((*data) << 1) | readBit;
    }

    /*add  an extra clock cycles for robust reading , ensure partner stop 
     *output signal and not affect the next read operation, because TA 
     *steal a clock*/     
    _smiWriteBit(1);
    _smiZbit();

    /*unlock the source, enable interrupt*/    
    //rtlglue_drvMutexUnlock();
    
    return  SUCCESS;
}

/* Function Name:
 *      smiWrite
 * Description:
 *      Write data to Phy register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31)
 *      data    -  Data to be written into Phy register
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *     This function could read register through MDC/MDIO serial 
 *     interface, and it is platform  related. It use two GPIO pins 
 *     to simulate MDC/MDIO timing. MDC is sourced by the Station Management 
 *     entity to the PHY as the timing reference for transfer of information
 *     on the MDIO signal. MDC is an aperiodic signal that has no maximum high 
 *     or low times. The minimum high and low times for MDC shall be 160 ns each, 
 *     and the minimum period for MDC shall be 400 ns. Obeying frame format defined
 *     by IEEE802.3 standard, you could access Phy registers. If you want to 
 *     port it to other CPU, please modify static functions which are called 
*      by this function.
 */

int32 smiWrite(uint32 phyad, uint32 regad, uint32 data)
{
    int32 i;

    if ((phyad > 31) || (regad > 31) || (data > 0xFFFF))  
        return FAILED;

    /*it lock the resource to ensure that SMI opertion is atomic, 
      *the API is based on RTL865X, it is used to disable CPU interrupt,
      *if porting to other platform, please rewrite it to realize the same function
      */
    //rtlglue_drvMutexLock();   

    /* 32 continuous 1 as preamble*/
    for(i=0; i<32; i++)
        _smiWriteBit(1);
    
    /* ST: Start of Frame, <01>*/
    _smiWriteBit(0);
    _smiWriteBit(1);

    /* OP: Operation code, write is <01>*/
    _smiWriteBit(0);
    _smiWriteBit(1);

    /* PHY Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((phyad>>i)&0x1);

    /* Register Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((regad>>i)&0x1);

    /* TA: Turnaround <10>*/
    _smiWriteBit(1);
    _smiWriteBit(0);

    /* Data */
    for(i=15; i>=0; i--) 
        _smiWriteBit((data>>i)&0x1);
	
    _smiZbit();

    /*unlock the source, enable interrupt*/        
    //rtlglue_drvMutexUnlock();
            
    return SUCCESS; 
}


/* Function Name:
 *      smiInit
 * Description:
 *      Init Rtl8651B smi interface
 * Input:
 *      port        - Specify Rtl8651B GPIO port
 *      pinMDC    - Set which gpio pin as MDC 
 *      pinMDIO   - Set which gpio pin as MDIO
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 *      This function is only for Rtl8651B, use it to specify
 *      GPIO pins as MDC/MDIO signal. It should be called at first.
 */
#if 0
#if(PAD_GPIO19_IS_GPIO != GPIO_NONE)
#define PAD_GPIO19_OEN (PAD_GPIO19_IS_GPIO == GPIO_IN ? BIT1: 0)
#define PAD_GPIO19_OUT (PAD_GPIO19_IS_GPIO == GPIO_OUT_HIGH ? BIT0: 0)
_RVM1(0x2b13, PAD_GPIO19_OUT, BIT0),
_RVM1(0x2b13, PAD_GPIO19_OEN, BIT1),
//reg_agc_dbg
_RVM1(0x1e9e, 0, BIT7 ),   //reg[101e9e]#7 = 0b
//reg_tso_evd_mode[1:0]
_RVM1(0x1e21, 0, BIT2 | BIT1 ),   //reg[101e21]#2 ~ #1 = 00b
//reg_et_mode
_RVM1(0x1edf, 0, BIT0 ),   //reg[101edf]#0 = 0b
//reg_led_mode
_RVM1(0x1eb4, 0, BIT4 ),   //reg[101eb4]#4 = 0b
//reg_seconduartmode[1:0]
_RVM1(0x1e05, 0, BIT1 | BIT0 ),   //reg[101e05]#1 ~ #0 = 00b
//reg_od2nduart[1:0]
_RVM1(0x1ea9, 0, BIT1 | BIT0 ),   //reg[101ea9]#1 ~ #0 = 00b
//reg_miic_mode3[1:0]
_RVM1(0x1edf, 0, BIT2 | BIT1 ),   //reg[101edf]#2 ~ #1 = 00b
//reg_allpad_in
_RVM1(0x1ea1, 0, BIT7 ),   //reg[101ea1]#7 = 0b
#endif

#if(PAD_GPIO20_IS_GPIO != GPIO_NONE)
#define PAD_GPIO20_OEN (PAD_GPIO20_IS_GPIO == GPIO_IN ? BIT1: 0)
#define PAD_GPIO20_OUT (PAD_GPIO20_IS_GPIO == GPIO_OUT_HIGH ? BIT0: 0)
_RVM1(0x2b14, PAD_GPIO20_OUT, BIT0),
_RVM1(0x2b14, PAD_GPIO20_OEN, BIT1),
//reg_agc_dbg
_RVM1(0x1e9e, 0, BIT7 ),   //reg[101e9e]#7 = 0b
//reg_tso_evd_mode[1:0]
_RVM1(0x1e21, 0, BIT2 | BIT1 ),   //reg[101e21]#2 ~ #1 = 00b
//reg_et_mode
_RVM1(0x1edf, 0, BIT0 ),   //reg[101edf]#0 = 0b
//reg_led_mode
_RVM1(0x1eb4, 0, BIT4 ),   //reg[101eb4]#4 = 0b
//reg_seconduartmode[1:0]
_RVM1(0x1e05, 0, BIT1 | BIT0 ),   //reg[101e05]#1 ~ #0 = 00b
//reg_od2nduart[1:0]
_RVM1(0x1ea9, 0, BIT1 | BIT0 ),   //reg[101ea9]#1 ~ #0 = 00b
//reg_miic_mode3[1:0]
_RVM1(0x1edf, 0, BIT2 | BIT1 ),   //reg[101edf]#2 ~ #1 = 00b
//reg_allpad_in
_RVM1(0x1ea1, 0, BIT7 ),   //reg[101ea1]#7 = 0b
#endif

#endif


int32 smiInit(void)
{
#ifdef MS838_8304MB

	MDrv_GPIO_Init();
    MDrv_GPIO_WriteRegBit(REG_ETH0_LED_MODE,_DISABLE,_BIT0);

	smi_MDC = MDC_PIN;
	MDrv_GPIO_Pad_Oen(smi_MDC);
	smi_MDIO = MDIO_PIN;
	MDrv_GPIO_Pad_Odn(smi_MDIO);
	return SUCCESS;

#else
    uint32 gpioId;
    int32 res;

    /* Initialize GPIO smi_MDC  as SMI MDC signal */
    gpioId = GPIO_ID(port, pinMDC);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != SUCCESS)
        return res;
    smi_MDC = gpioId;

    /* Initialize GPIO smi_MDIO  as SMI MDIO signal */
    gpioId = GPIO_ID(port, pinMDIO);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);
    if (res != SUCCESS)
        return res;
    smi_MDIO = gpioId;
#endif

    return SUCCESS;

}
void smiDeInit(void)
{
	MDrv_GPIO_WriteRegBit(REG_ETH0_LED_MODE,_ENABLE,_BIT0);
	smi_MDC = NULL;
	smi_MDIO = NULL;
}


#else  /*high speed smi*/

/*
 *high speed smi is a special interface, it is realsized by configuring rtl865x gpio register directly,
 *it specifies GPIO port C pin1 as MDC, pin 0 as MDIO
 */
/* Change clock to 1 */
static void _smiZBit(void) 
{
    uint32 i;
    REG32(PABCDIR) = (REG32(PABCDIR)& 0xFFFFFCFF) | 0x200;
    REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF);
    for(i=0; i< DELAY; i++);
}

/* Generate  1 -> 0 transition and sampled at 1 to 0 transition time,
 *should not sample at 0->1 transition because some chips stop outputing
 *at the last bit at rising edge
 */

static void _smiReadBit(uint32 * pdata) 
{
    uint32 i;
    REG32(PABCDIR) = (REG32(PABCDIR)& 0xFFFFFCFF) | 0x200;
    REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF) | 0x200;
    for(i=0; i< DELAY; i++);
    REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF);
    *pdata = (REG32(PABCDAT) & 0x100)?1:0;
}

/* Generate  0 -> 1 transition and put data ready during 0 to 1 whole period */
static void _smiWriteBit(uint32 data) 
{
    uint32 i;

    REG32(PABCDIR) = REG32(PABCDIR) | 0x300;
    if(data) 
    {  /* Write 1 */
        REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF) | 0x100;
        for(i=0; i< DELAY; i++);
        REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF) | 0x300;
    } 
    else 
    {
        REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF);
        for(i=0; i< DELAY; i++);
        REG32(PABCDAT) = (REG32(PABCDAT) & 0xFFFFFCFF) | 0x200;
    }
}

int32 smiRead(uint32 phyad, uint32 regad, uint32 * data) 
{
    int32 i;
    uint32 readBit;

    /* Configure port C pin 1, 0 to be GPIO and disable interrupts of these two pins */
    REG32(PABCCNR) = REG32(PABCCNR) & 0xFFFFFCFF;
    REG32(PCIMR) = REG32(PCIMR) & 0xFFFFFFF;
    
    /* 32 continuous 1 as preamble*/
    for(i=0; i<32; i++)
        _smiWriteBit(1);
    
    /* ST: Start of Frame, <01>*/
    _smiWriteBit(0);
    _smiWriteBit(1);
    
    /* OP: Operation code, read is <10> */
    _smiWriteBit(1);
    _smiWriteBit(0);

    /* PHY Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((phyad>>i)&0x1);

    /* Register Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((regad>>i)&0x1);
    
    /* TA: Turnaround <z0> */
    _smiZBit();
    _smiReadBit(&readBit);
    
    /* Data */
    *data = 0;
    for(i=15; i>=0; i--) 
    {
        _smiReadBit(&readBit);
        *data = (*data<<1) | readBit;
    }
    
    /*add  an extra clock cycles for robust reading , ensure partner stop output signal
      *and not affect the next read operation, because TA steal a clock
      */     
    _smiWriteBit(1);
    _smiZbit();
    
    return SUCCESS ;
}

int32 smiWrite(uint32 phyad, uint32 regad, uint32 data) 
{
    int32 i;

    /*Configure port C pin 1, 0 to be GPIO and disable interrupts of these two pins */
    REG32(PABCCNR) = REG32(PABCCNR) & 0xFFFFFCFF;
    REG32(PCIMR) = REG32(PCIMR) & 0xFFFFFFF;
    
    /* 32 continuous 1 as preamble*/
    for(i=0; i<32; i++)
        _smiWriteBit(1);
    
    /* ST: Start of Frame, <01>*/
    _smiWriteBit(0);
    _smiWriteBit(1);
    
    /* OP: Operation code, write is <01> */
    _smiWriteBit(0);
    _smiWriteBit(1);
    
    /* PHY Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((phyad>>i)&0x1);

    /* Register Address */
    for(i=4; i>=0; i--) 
        _smiWriteBit((regad>>i)&0x1);
    
    /* TA: Turnaround <10> */
    _smiWriteBit(1);
    _smiWriteBit(0);
    
    /* Data */
    for(i=15; i>=0; i--) 
        _smiWriteBit((data>>i)&0x1);
    _smiZBit();
    
    return SUCCESS;
}


#endif /* if  SMI_HIGHSPEED*/


/* Function Name:
 *      smiReadBit
 * Description:
 *      Read one bit of PHY register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 *      bit       -  Register bit (0~15)   
 * Output:
 *      pdata    - the pointer of  Register bit value 
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 */

int32 smiReadBit(uint32 phyad, uint32 regad, uint32 bit, uint32 * pdata) 
{
    uint32 regData;

    if ((phyad > 31) || (regad > 31) || (bit > 15) || (pdata == NULL) ) 
        return  FAILED;
    
    if(bit>=16)
        * pdata = 0;
    else 
    {
        smiRead(phyad, regad, &regData);
        if(regData & (1<<bit)) 
            * pdata = 1;
        else
            * pdata = 0;
    }
    return SUCCESS;
}

/* Function Name:
 *      smiWriteBit
 * Description:
 *      Write one bit of PHY register
 * Input:
 *      phyad   - PHY address (0~31)
 *      regad   -  Register address (0 ~31) 
 *      bit       -  Register bit (0~15)   
 *      data     -  Bit value to be written
 * Output:
 *      none
 * Return:
 *      SUCCESS         -  Success
 *      FAILED            -  Failure
 * Note:
 */

int32 smiWriteBit(uint32 phyad, uint32 regad, uint32 bit, uint32 data) 
{
    uint32 regData;
    
    if ((phyad > 31) || (regad > 31) || (bit > 15) || (data > 1) ) 
        return  FAILED;
    smiRead(phyad, regad, &regData);
    if(data) 
        regData = regData | (1<<bit);
    else
        regData = regData & ~(1<<bit);
    smiWrite(phyad, regad, regData);
    return SUCCESS;
}

int32 rtl8309n_reg_get(uint32 phyad, uint32 regad, uint32 npage, uint32 *pvalue)
{
    uint32 rdata; 

    if ((phyad > RTL8309N_MAX_PORT_ID) || (npage >= RTL8309N_PAGE_NUMBER))
        return FAILED;

	/*it lock the resource to ensure that reg read/write opertion is thread-safe, 
      *if porting to other platform, please rewrite it to realize the same function
      */
	  //printk("%s[%d]\n",__FUNCTION__,__LINE__);
    rtlglue_drvMutexLock(); 
//printk("%s[%d]\n",__FUNCTION__,__LINE__);
    /* Select MAC or PHY page, configure PHY 8 Register 31 bit[15] */
	smiRead(8, 31, &rdata);
	rdata &= ~((0x1 << 15) | 0xFF);
	/* select mac page, configure phy 8 register 31 bit[0:7]*/
	rdata |= npage;
	smiWrite(8, 31, rdata);
	/* slelect phy and reg number, write data into register */
    smiRead(phyad, regad, &rdata);

	*pvalue = rdata & 0xFFFF;
	
	/*unlock the source, enable interrupt*/    
    rtlglue_drvMutexUnlock();
	
    return SUCCESS;

}
#endif
rtk_api_ret_t mrtk_port_phyStatus_get(rtk_port_t phy, rtk_port_linkStatus_t *pLinkStatus, rtk_port_speed_t *pSpeed, rtk_port_duplex_t *pDuplex)
{
	uint32 regVal;
	int32 retVal;
	
    if(phy > RTK_PHY_ID_MAX)
        return RT_ERR_PORT_ID; 

	if(NULL == pLinkStatus)
		return RT_ERR_NULL_POINTER;

    if((retVal= rtl8309n_reg_get(2, 18,14,&regVal)) != SUCCESS)
        return RT_ERR_FAILED;

    if(regVal & (0x1 << phy))
    {
        *pLinkStatus = PORT_LINKUP;
		 if((retVal= rtl8309n_reg_get(2, 19,14,&regVal)) != SUCCESS)
        	return RT_ERR_FAILED;
        *pSpeed = (regVal & (0x1 << phy)) ? PORT_SPEED_100M : PORT_SPEED_10M; 
		if((retVal= rtl8309n_reg_get(2, 20,14,&regVal)) != SUCCESS)
        	return RT_ERR_FAILED;
        *pDuplex = (regVal & (0x1 << phy)) ? PORT_FULL_DUPLEX : PORT_HALF_DUPLEX;
    }
    else
    {
        *pLinkStatus = PORT_LINKDOWN;
        *pSpeed = PORT_SPEED_10M;
        *pDuplex = PORT_HALF_DUPLEX; 
    }
	printk("phy = %d, link = %s,speed=%s duplex =%s\n",\
					phy,\
					*pLinkStatus==PORT_LINKUP?"up":"down",\
					*pSpeed==PORT_SPEED_100M?"100M":"10M",\
					*pDuplex==PORT_FULL_DUPLEX?"FULL":"HALF");
	return RT_ERR_OK;
}

typedef struct rtk_port_status{
	rtk_port_linkStatus_t LinkStatus;
	rtk_port_speed_t Speed;
	rtk_port_duplex_t Duplex;
};

struct rtk_port_status *_pStatusPort_1;
struct rtk_port_status *_pStatusPort_2;

int _is_status_change(rtk_port_t phy)
{
	rtk_api_ret_t ret;
	//rtk_port_t phy = RTL8304MB_PORT0;
	rtk_port_linkStatus_t LinkStatus,LinkStatus_pre;
	rtk_port_speed_t Speed,Speed_pre;
	rtk_port_duplex_t Duplex, Duplex_pre;
	rtk_port_status *pStatusPort_pre;
	ret = mrtk_port_phyStatus_get( phy,
									&LinkStatus,
									&Speed,
									&Duplex);
	if(unlikly RT_ERR_OK != ret)
	{
		TPRINT( "rtk_port_phyStatus_get ErrNum[%d]\n",ret);
		return FALSE;
	}
	else if(phy == RTL8304MB_PORT0)
	{
		pStatusPort_pre = _pStatusPort_1;
	}
	else
	{
		pStatusPort_pre = _pStatusPort_2;
	}

	if(pStatusPort_pre->LinkStatus != LinkStatus
		|| pStatusPort_pre->Speed != Speed
		|| pStatusPort_pre->Duplex != Duplex)
	{
		pStatusPort_pre->LinkStatus = LinkStatus;
		pStatusPort_pre->Speed= Speed;
		pStatusPort_pre->Duplex= Duplex;
		return TRUE;
	}

	return FALSE;
}

int _monitor_thread(void * data)
{
    TPRINT( "TCLKING [%s][%d] enter _poting_rtl8304mb \n",__func__,__LINE__);
	while(!kthread_should_stop())
	{
		TPRINT( "TCLKING [%s][%d] kk enter loop \n",__func__,__LINE__);
		if(_is_status_change(RTL8304MB_PORT0))
		{
			_notify_ethernet_change();
			continue;
		}
		if(_is_status_change(RTL8304MB_PORT1))
		{
			_notify_ethernet_change();
			continue;
		}

		schedule_timeout_interruptible(2 * HZ);
	}
	
	return 0;
}

static int __init rtl8304mb_init(void)
{
	TPRINT( "TCLKING [%s][%d] hello \n",__func__,__LINE__);
	smiInit();
	_pStatusPort_1 = (rtk_port_status *)vmalloc(sizeof(rtk_port_status)*2);
	_pStatusPort_2 = _pStatusPort_1 + sizeof(rtk_port_status);

	tthread = kthread_run(_monitor_thread, NULL, "poting_eth0");
	if( IS_ERR(tthread))
		TPRINT("run _poting_rtl8304mb fail !\n");
    TPRINT( "TCLKING [%s][%d] rtl8304mb_init done \n",__func__,__LINE__);
	return 0;
}
static int __exit rtl8304mb_exit(void)
{
	kthread_stop(tthread);
	smiDeInit();
	vfree(_pStatusPort_1);
	TPRINT( "TCLKING [%s][%d] bye \n",__func__,__LINE__);
	return 0;
}

module_init(rtl8304mb_init);
module_exit(rtl8304mb_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("TCLKING");
