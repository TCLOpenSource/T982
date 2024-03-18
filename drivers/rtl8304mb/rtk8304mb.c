#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/net_namespace.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include "rtl8304mb_gpio_ops.h"
#include "rtk8304mb.h"
#include "rtk_api.h"

//R-SDK-V105: RTL8309M_8306MB_8304MB_Switch_SDK_v1.0.5
//1912: 2019/12
#define DRV_VERSION "R-SDK-V105.1912.1"

//#define TCL_DEBUG

#ifdef TCL_DEBUG
#define TG_DEBUG(fmt, args...)    pr_info("[%s:%d]rtl8304mb_gpio_ops: " fmt, __func__, __LINE__, ##args)
#else
#define TG_DEBUG(fmt, args...)
#endif

struct task_struct *tthread = NULL;
struct rtk_port_status *_pStatusPort_1;
struct rtk_port_status *_pStatusPort_2;
static uint MDC = 0;         /* GPIO used for SMI Clock Generation */
static uint MDIO = 0;       /* GPIO used for SMI Data signal */
static bool DBG = 0;
spinlock_t ms838_8304mb_gpio_lock;
unsigned long flags;

static struct hrtimer hrtimer_8304_timer;
ktime_t kt;

static struct platform_device *g_tcl_rtl8304mb_pdev;

module_param(MDC,uint,S_IWUSR);
MODULE_PARM_DESC(MDC,"the gpio number which connects to MDC pin of rtl8304mb.");
module_param(MDIO,uint,S_IWUSR);
MODULE_PARM_DESC(MDIO,"the gpio number which connect to MDIO pin of rtl8304mb.");
module_param(DBG,bool,S_IWUSR);
MODULE_PARM_DESC(DBG,"enable debug: DBG=1");

//#define TPRINT(...) printk(KERN_INFO"8304:" __VA_ARGS__)

/* Is it required to disable interrupt? */
static void rtlglue_drvMutexLock(void)
{
    //TG_DEBUG("%s[%d]\n",__FUNCTION__,__LINE__);
    spin_lock_irqsave(&ms838_8304mb_gpio_lock, flags);
    //TG_DEBUG("%s[%d]\n",__FUNCTION__,__LINE__);
}
static void rtlglue_drvMutexUnlock(void)
{
    //TG_DEBUG("%s[%d]\n",__FUNCTION__,__LINE__);
    spin_unlock_irqrestore(&ms838_8304mb_gpio_lock, flags);
    //TG_DEBUG("%s[%d]\n",__FUNCTION__,__LINE__);
}

///*
// *Low speed smi is a general MDC/MDIO interface, it is realized by call gpio api
// *function, could specified any gpio pin as MDC/MDIO
// */
//static void _smiZbit(void)
//{
//    MDrv_GPIO_Pull_Low(MDC);
//    MDrv_GPIO_Pull_Low(MDIO);
//    //CLK_DURATION(DELAY);
//    ndelay(500);
//}

/*Generate  1 -> 0 transition and sampled at 1 to 0 transition time,
 *should not sample at 0->1 transition because some chips stop outputing
 *at the last bit at rising edge
 */
//static void _smiReadBit(uint * pdata)
//{
//    uint u;
//    MDrv_GPIO_Pad_Odn(MDIO);
//
//    MDrv_GPIO_Pull_High(MDC);
//    //CLK_DURATION(DELAY);
//    ndelay(500);
//    MDrv_GPIO_Pull_Low(MDC);
//    u = MDrv_GPIO_Pad_Read(MDIO);
//    *pdata = u;
//    //CLK_DURATION(DELAY);
//    ndelay(500);
//}

///* Generate  0 -> 1 transition and put data ready during 0 to 1 whole period */
//static void _smiWriteBit(uint data)
//{
//    MDrv_GPIO_Pad_Oen(MDIO);
//
//    if(data)
//        MDrv_GPIO_Pull_High(MDIO);
//    else
//        MDrv_GPIO_Pull_Low(MDIO);
//
//    MDrv_GPIO_Pull_Low(MDC);
//    //CLK_DURATION(DELAY);
//    ndelay(500);
//    MDrv_GPIO_Pull_High(MDC);
//    //CLK_DURATION(DELAY);
//    ndelay(500);
//}


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
static int smiRead(uint phyad, uint regad, uint * data)
{

    int i;
    uint readBit;

    if ((phyad > 8) || (regad > 31) || (data == NULL))
        return FAILED;

    /*it lock the resource to ensure that SMI opertion is atomic,
     *the API is based on RTL865X, it is used to disable CPU interrupt,
     *if porting to other platform, please rewrite it to realize the same function
     */

    //rtlglue_drvMutexLock();

    /* 32 continuous 1 as preamble*/
    for(i=0; i<32; i++)
        rtl8304mb_gpio_write_bit(1);

    /* ST: Start of Frame, <01>*/
    rtl8304mb_gpio_write_bit(0);
    rtl8304mb_gpio_write_bit(1);

    /* OP: Operation code, read is <10>*/
    rtl8304mb_gpio_write_bit(1);
    rtl8304mb_gpio_write_bit(0);

    /* PHY Address */
    for(i=4; i>=0; i--)
        rtl8304mb_gpio_write_bit((phyad>>i)&0x1);

    /* Register Address */
    for(i=4; i>=0; i--)
        rtl8304mb_gpio_write_bit((regad>>i)&0x1);

    /* TA: Turnaround <z0>, zbit has no clock in order to steal a clock to
     *  sample data at clock falling edge
     */
    _smiZbit();
    //rtl8304mb_gpio_write_bit(0);
    rtl8304mb_gpio_read_bit(&readBit);

    /* Data */
    *data = 0;
    for(i=15; i>=0; i--)
    {
        rtl8304mb_gpio_read_bit(&readBit);
        *data = ((*data) << 1) | readBit;
    }

    /*add  an extra clock cycles for robust reading , ensure partner stop
     *output signal and not affect the next read operation, because TA
     *steal a clock*/
    rtl8304mb_gpio_write_bit(1);
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

static int smiWrite(uint phyad, uint regad, uint data)
{
    int i;

    if ((phyad > 31) || (regad > 31) || (data > 0xFFFF))
        return FAILED;

    /*it lock the resource to ensure that SMI opertion is atomic,
     *the API is based on RTL865X, it is used to disable CPU interrupt,
     *if porting to other platform, please rewrite it to realize the same function
     */
    //rtlglue_drvMutexLock();

    /* 32 continuous 1 as preamble*/
    for(i=0; i<32; i++)
        rtl8304mb_gpio_write_bit(1);

    /* ST: Start of Frame, <01>*/
    rtl8304mb_gpio_write_bit(0);
    rtl8304mb_gpio_write_bit(1);

    /* OP: Operation code, write is <01>*/
    rtl8304mb_gpio_write_bit(0);
    rtl8304mb_gpio_write_bit(1);

    /* PHY Address */
    for(i=4; i>=0; i--)
        rtl8304mb_gpio_write_bit((phyad>>i)&0x1);

    /* Register Address */
    for(i=4; i>=0; i--)
        rtl8304mb_gpio_write_bit((regad>>i)&0x1);

    /* TA: Turnaround <10>*/
    rtl8304mb_gpio_write_bit(1);
    rtl8304mb_gpio_write_bit(0);

    /* Data */
    for(i=15; i>=0; i--)
        rtl8304mb_gpio_write_bit((data>>i)&0x1);

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

static int smiInit(void)
{
    //MDrv_GPIO_Init();
    //MDrv_GPIO_WriteRegBit(REG_ETH0_LED_MODE,_DISABLE,_BIT4);
    //if(MDC == 0    || MDIO == 0)
    //{
    //    MDIO = DEF_MDIO_PIN;
    //    MDC = DEF_MDC_PIN;
    //}
    //MDrv_GPIO_Pad_Oen(MDC);  //output
    //MDrv_GPIO_Pad_Odn(MDIO);  // input
    //TG_DEBUG( "Register[0x%p]BIT[0x%x] set [%d]\n",
    //        (void*)REG_ETH0_LED_MODE,_BIT4,_DISABLE);
    //TG_DEBUG( "MDC = [%d] MDIO = [%d]\n",
    //        MDC,MDIO);
    return rtl8304mb_gpio_init(g_tcl_rtl8304mb_pdev);;
}

static void smiDeInit(void)
{
    //MDrv_GPIO_WriteRegBit(REG_ETH0_LED_MODE,_ENABLE,_BIT4);
    //MDC = 0;
    //MDIO = 0;
    rtl8304mb_gpio_deinit(g_tcl_rtl8304mb_pdev);
}

static int rtl8309n_reg_get(uint phyad, uint regad, uint npage, uint *pvalue)
{
    uint rdata;

    if ((phyad > RTL8309N_MAX_PORT_ID) || (npage >= RTL8309N_PAGE_NUMBER))
        return FAILED;

    /*it lock the resource to ensure that reg read/write opertion is thread-safe,
     *if porting to other platform, please rewrite it to realize the same function
     */
    rtlglue_drvMutexLock();
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

static rtk_api_ret_t mrtk_port_phyStatus_get(
        rtk_port_t phy,
        rtk_port_linkStatus_t *pLinkStatus,
        rtk_port_speed_t *pSpeed,
        rtk_port_duplex_t *pDuplex)
{
    uint regVal;
    int retVal;

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
        TG_DEBUG( "[%s][%d] \n",__func__,__LINE__);

    }
    else
    {
        TG_DEBUG( "[%s][%d] \n",__func__,__LINE__);
        *pLinkStatus = PORT_LINKDOWN;
        *pSpeed = PORT_SPEED_10M;
        *pDuplex = PORT_HALF_DUPLEX;
    }
    TG_DEBUG("phy[%d],[%s],[%s],[%s]\n",\
            phy,\
            *pLinkStatus==PORT_LINKUP?"up":"down",\
            *pSpeed==PORT_SPEED_100M?"100M":"10M",\
            *pDuplex==PORT_FULL_DUPLEX?"FULL":"HALF");
    return RT_ERR_OK;
}

static int init_status_change(rtk_port_t phy)
{
    rtk_api_ret_t ret;
    rtk_port_linkStatus_t LinkStatus;
    rtk_port_speed_t Speed;
    rtk_port_duplex_t Duplex;
    //struct rtk_port_status *pPrePortStatus;
    struct net_device* ndev = dev_get_by_name(&init_net, "eth0");
    if( ndev ==  NULL)
    {
        TG_DEBUG( "dev_get_by_name fail!\n");
    }
    ret = mrtk_port_phyStatus_get( phy, &LinkStatus, &Speed, &Duplex);
    if(unlikely(RT_ERR_OK != ret))
    {
        TG_DEBUG( "rtk_port_phyStatus_get ErrNum[%d]\n",ret);
        return FALSE;
    }
    if(LinkStatus == PORT_LINKUP){
        //netif_carrier_on(ndev);
        TG_DEBUG( "[%s][%d] entry\n",__func__,__LINE__);
    }
    else{
        //netif_carrier_off(ndev);
        TG_DEBUG( "[%s][%d] entry\n",__func__,__LINE__);
    }

    dev_put(ndev);
    return TRUE;
}
static int _is_status_change(rtk_port_t phy)
{
    rtk_api_ret_t ret;
    rtk_port_linkStatus_t LinkStatus;
    rtk_port_speed_t Speed;
    rtk_port_duplex_t Duplex;
    struct rtk_port_status *pPrePortStatus;
    ret = mrtk_port_phyStatus_get( phy,
            &LinkStatus,
            &Speed,
            &Duplex);
    if(unlikely(RT_ERR_OK != ret))
    {
        TG_DEBUG( "rtk_port_phyStatus_get ErrNum[%d]\n",ret);
        return FALSE;
    }
    else if(phy == RTL8304MB_PORT0)
    {
        pPrePortStatus = _pStatusPort_1;
    }
    else
    {
        pPrePortStatus = _pStatusPort_2;
    }

    if(pPrePortStatus->LinkStatus != LinkStatus
            || pPrePortStatus->Speed != Speed
            || pPrePortStatus->Duplex != Duplex)
    {
        pPrePortStatus->LinkStatus = LinkStatus;
        pPrePortStatus->Speed= Speed;
        pPrePortStatus->Duplex= Duplex;
        return TRUE;
    }
    return FALSE;
}

static int _notify_ethernet_change(void)
{
    static int fristInit = 0;
    uint regVal;
    int retVal;
    struct net_device* ndev = dev_get_by_name(&init_net, "eth0");
    if( ndev ==  NULL)
    {
        TG_DEBUG( "dev_get_by_name fail!\n");
    }

    if((retVal= rtl8309n_reg_get(2, 18,14,&regVal)) != SUCCESS)
        return RT_ERR_FAILED;
    if(fristInit == 0){
        fristInit++;
        netif_carrier_off(ndev);
    }
    else if(regVal & (0x1 << RTL8304MB_PORT0))
    {
        netif_carrier_on(ndev); //设置为1------->连接
        TG_DEBUG("[%s][%d]-----------netif_carrier_on----entry2\n",__func__,__LINE__);
    }
    else
    {
        netif_carrier_off(ndev);    //设置为0------>断开
        TG_DEBUG("[%s][%d]-----------netif_carrier_off----entry2\n",__func__,__LINE__);
    }
    TG_DEBUG("[%s][%d]----entry2---regVal[0x%x]\n",__func__,__LINE__,regVal);
    TG_DEBUG( "entry2 [%s] done          \n",__func__);

    dev_put(ndev);
    return RT_ERR_OK;
}

static enum hrtimer_restart _monitor_thread(struct hrtimer *hrtimer_8304_timer)
{
    int i = 0;
    struct net_device* ndev = NULL;
    //uint regVal;
    //int retVal;
    TG_DEBUG( "[%s][%d] entry\n",__func__,__LINE__);
    ndev = dev_get_by_name(&init_net, "eth0");
    if( ndev ==  NULL)
    {
        TG_DEBUG( "dev_get_by_name fail!\n");
        kt = ktime_set(3,0); // 设置下次过期时间
        hrtimer_forward_now(hrtimer_8304_timer, kt);
        return HRTIMER_RESTART;//该参数将重新启动定时器
    }

    if( i < 60){
        _notify_ethernet_change();
        i++;
    }
    else if(_is_status_change(RTL8304MB_PORT0)||_is_status_change(RTL8304MB_PORT1))
    {
        TG_DEBUG("[%s][%d]----notify_ethernet_change----- entry2\n",\
        __func__,__LINE__);
        _notify_ethernet_change();
    }
    TG_DEBUG( "[%s][%d] exit \n",__func__,__LINE__);

    kt = ktime_set(3,0); // 设置下次过期时间
    hrtimer_forward_now(hrtimer_8304_timer, kt);
    return HRTIMER_RESTART;//该参数将重新启动定时器
}

static int tcl_rtl8304mb_probe(struct platform_device *pdev)
{
    int retVal;
    g_tcl_rtl8304mb_pdev = pdev;
    retVal = smiInit();
    if(retVal == FAILED)
    {
        return -1;
    }
    TG_DEBUG("1");
    mdelay(3000);
    _pStatusPort_1 = (struct rtk_port_status*)vmalloc(sizeof(struct rtk_port_status)*2);
    if (_pStatusPort_1 == NULL) {
        TG_DEBUG(KERN_EMERG "%s vamlloc fail.\n",__func__);
        return -1;
    }
    _pStatusPort_2 = (struct rtk_port_status*)(_pStatusPort_1 + sizeof(struct rtk_port_status));
    _is_status_change(RTL8304MB_PORT0);
    _is_status_change(RTL8304MB_PORT1);
    init_status_change(RTL8304MB_PORT0);
    kt = ktime_set(3,0);
    hrtimer_init(&hrtimer_8304_timer,CLOCK_MONOTONIC,HRTIMER_MODE_REL);// hrtimer初始化
    hrtimer_8304_timer.function = _monitor_thread;// 设置回调函数
    hrtimer_start(&hrtimer_8304_timer,kt,HRTIMER_MODE_REL);// hrtimer启动
    return 0;
}

static int tcl_rtl8304mb_remove(struct platform_device *pdev)
{
    smiDeInit();
    if(_pStatusPort_1 != NULL)
    {
        vfree(_pStatusPort_1);
        _pStatusPort_1 = NULL;
        _pStatusPort_2 = NULL;
    }
    hrtimer_cancel(&hrtimer_8304_timer);
    TG_DEBUG(KERN_INFO "[%s][%d] EXIT \n",__func__,__LINE__);
    return 0;
}


static const struct of_device_id of_match_tcl_rtl8304mb[] = {
    {.compatible = "rtl8304mb",},
    {/* sentinel */},
};

static struct platform_driver tcl_rtl8304_drv = {
    .probe = tcl_rtl8304mb_probe,
    .driver = {
        .name = "rtl8304mb",
        .of_match_table = of_match_tcl_rtl8304mb,
    },
    .remove = tcl_rtl8304mb_remove,
};

static int __init rtl8304mb_init(void)
{
    platform_driver_register(&tcl_rtl8304_drv);
    return 0;
}

static void __exit rtl8304mb_exit(void)
{
    platform_driver_unregister(&tcl_rtl8304_drv);
}


late_initcall(rtl8304mb_init);
module_exit(rtl8304mb_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("rtl8304mb_porting");
MODULE_AUTHOR("TCLKING");
