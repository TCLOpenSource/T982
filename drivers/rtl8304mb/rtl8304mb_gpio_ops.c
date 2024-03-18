#include "rtl8304mb_gpio_ops.h"
#include "rtl8309n_types.h"

#define TCL_DEBUG

#ifdef TCL_DEBUG
#define TG_DEBUG(fmt, args...)	pr_info("[%s:%d]rtl8304mb_gpio_ops: " fmt, __func__, __LINE__, ##args)
#else
#define TG_DEBUG(fmt, args...)
#endif

static struct gpio_desc * s_mdio_gpio_desc = NULL;
static struct gpio_desc * s_mdc_gpio_desc = NULL;

int rtl8304mb_gpio_init(struct platform_device *pdev)
{
    TG_DEBUG("here\n");
    if(pdev == NULL)
    {
        dev_err(&pdev->dev,"rtl8304mb_gpio_ops: platform_device is null\n");
        return FAILED;
    }

    s_mdio_gpio_desc = gpiod_get_optional(&pdev->dev, "mdio", GPIOD_ASIS);
    if (IS_ERR_OR_NULL(s_mdio_gpio_desc))
    {
        //TG_DEBUG("can't find %s or already requested by someone\n", "DEF_MDIO_PIN");
        dev_err(&pdev->dev,"rtl8304mb_gpio_ops: can't find %s or already requested by someone\n", "DEF_MDIO_PIN");
        return FAILED;
    }
    else
    {
        if (gpiod_get_direction(s_mdio_gpio_desc) != GPIOF_DIR_IN)
        {
            gpiod_direction_input(s_mdio_gpio_desc);
        }
        else
        {
            dev_err(&pdev->dev,"rtl8304mb_gpio_ops: %s had set direction in as what we want\n", "DEF_MDIO_PIN");
            TG_DEBUG("%s had set direction in as what we want\n", "DEF_MDIO_PIN");
        }
        //gpiod_put(s_mdio_gpio_desc);
    }

    s_mdc_gpio_desc = gpiod_get_optional(&pdev->dev, "mdc", GPIOD_ASIS);
    if (IS_ERR_OR_NULL(s_mdc_gpio_desc))
    {
        dev_err(&pdev->dev,"rtl8304mb_gpio_ops: can't find %s or already requested by someone\n", "DEF_MDC_PIN");
        TG_DEBUG("can't find %s or already requested by someone\n", "DEF_MDC_PIN");
        return FAILED;
    }
    else
    {
        if (gpiod_get_direction(s_mdc_gpio_desc) != GPIOF_DIR_OUT)
        {
            gpiod_direction_output(s_mdc_gpio_desc, 1); // set default value
        }
        else
        {
            dev_err(&pdev->dev,"rtl8304mb_gpio_ops: %s had set direction out  as what we want\n", "DEF_MDC_PIN");
            TG_DEBUG("%s had set direction out  as what we want\n", "DEF_MDC_PIN");
        }
        //gpiod_put(s_mdc_gpio_desc);
    }
    return SUCCESS;
}

int rtl8304mb_gpio_deinit(struct platform_device *pdev)
{
    if(s_mdio_gpio_desc != NULL)
    {
        gpiod_put(s_mdio_gpio_desc);
    }

    if(s_mdc_gpio_desc != NULL)
    {
        gpiod_put(s_mdc_gpio_desc);
    }

    return SUCCESS;
}

void rtl8304mb_gpio_read_bit(uint * pdata)
{
    int u;
    //MDrv_GPIO_Pad_Odn(MDIO);
    gpiod_direction_input(s_mdio_gpio_desc);
    gpiod_direction_output(s_mdc_gpio_desc, 1);
    //CLK_DURATION(DELAY);
    ndelay(500);
    gpiod_direction_output(s_mdc_gpio_desc, 0);
    u = gpiod_get_value(s_mdio_gpio_desc);
    *pdata = u;
    //CLK_DURATION(DELAY);
    ndelay(500);
}

void rtl8304mb_gpio_write_bit(uint data)
{
    gpiod_direction_output(s_mdio_gpio_desc, 0);

    if(data)
        gpiod_direction_output(s_mdio_gpio_desc, 1);
    else
        gpiod_direction_output(s_mdio_gpio_desc, 0);

    gpiod_direction_output(s_mdc_gpio_desc, 0);
    //CLK_DURATION(DELAY);
    ndelay(500);
    gpiod_direction_output(s_mdc_gpio_desc, 1);
    //CLK_DURATION(DELAY);
    ndelay(500);
}

void _smiZbit(void)
{
	gpiod_direction_output(s_mdc_gpio_desc, 0);
	gpiod_direction_output(s_mdio_gpio_desc, 0);
	//CLK_DURATION(DELAY);
	ndelay(500);
}

/*Generate  1 -> 0 transition and sampled at 1 to 0 transition time,
 *should not sample at 0->1 transition because some chips stop outputing
 *at the last bit at rising edge
 */
//static void _smiReadBit(uint * pdata)
//{
//	uint u;
//	MDrv_GPIO_Pad_Odn(MDIO);
//
//	MDrv_GPIO_Pull_High(MDC);
//	//CLK_DURATION(DELAY);
//	ndelay(500);
//	MDrv_GPIO_Pull_Low(MDC);
//	u = MDrv_GPIO_Pad_Read(MDIO);
//	*pdata = u;
//	//CLK_DURATION(DELAY);
//	ndelay(500);
//}

///* Generate  0 -> 1 transition and put data ready during 0 to 1 whole period */
//static void _smiWriteBit(uint data)
//{
//	MDrv_GPIO_Pad_Oen(MDIO);
//
//	if(data)
//		MDrv_GPIO_Pull_High(MDIO);
//	else
//		MDrv_GPIO_Pull_Low(MDIO);
//
//	MDrv_GPIO_Pull_Low(MDC);
//	//CLK_DURATION(DELAY);
//	ndelay(500);
//	MDrv_GPIO_Pull_High(MDC);
//	//CLK_DURATION(DELAY);
//	ndelay(500);
//}


///*
// *Low speed smi is a general MDC/MDIO interface, it is realized by call gpio api
// *function, could specified any gpio pin as MDC/MDIO
// */
//static void _smiZbit(void)
//{
//	MDrv_GPIO_Pull_Low(MDC);
//	MDrv_GPIO_Pull_Low(MDIO);
//	//CLK_DURATION(DELAY);
//	ndelay(500);
//}

