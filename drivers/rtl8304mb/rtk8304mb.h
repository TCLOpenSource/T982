//Mst headers
//#include "mhal_gpio_reg.h"
//#include "mdrv_gpio.h"
//Rtk headers
#include "rtk_api.h"
#include "rtk_api_ext.h"
#include "rtl8309n_types.h"
//#include "rtl8309n_asicdrv_ext.h"
#include "rtl8309n_asictypes.h"


#define _BIT4    (1<<4)
#define _DISABLE 0
#define _ENABLE 1
#define DELAY 100
#define SMI_HIGHSPEED 0
#define CLK_DURATION(clk) \
	{uint i; for (i=0; i< (clk); i++);}

/* MS838A REG:0x101eb4 eable led mode
	|_ gpio82 <-> MDC
	|_ gpio81 <-> MDIO
*/

//#define REG_ETH0_LED_MODE       0x101eb4UL
//#define DEF_MDC_PIN             GPIOC_1      //out
//#define DEF_MDIO_PIN            GPIOC_0      //in

struct rtk_port_status{
	rtk_port_linkStatus_t LinkStatus;
	rtk_port_speed_t Speed;
	rtk_port_duplex_t Duplex;
};


