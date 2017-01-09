#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define MIN_VOLTAGE (500)
#define MAX_VOLTAGE (700)

#define LCM_DSI_CMD_MODE                                    0

#define FRAME_WIDTH                                         (720)
#define FRAME_HEIGHT                                        (1280)

#define LCM_ID                                              (0x55)

#define REGFLAG_DELAY                                       0xFE
#define REGFLAG_END_OF_TABLE                                0xFFE   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
static unsigned int lcm_rst_pin;
static unsigned int lcm_gpio_enn;
static unsigned int lcm_gpio_enp;
#define SET_RESET_PIN(v)    lcm_rst_pin_set(lcm_rst_pin,v)//(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#ifndef BUILD_LK
extern atomic_t ESDCheck_byCPU;
#endif
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern void dsi_enter_hs(bool enter);

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

    /*
    Note :

    Data ID will depends on the following rule.

        count of parameters > 1 => Data ID = 0x39
        count of parameters = 1 => Data ID = 0x15
        count of parameters = 0 => Data ID = 0x05

    Structure Format :

    {DCS command, count of parameters, {parameter list}}
    {REGFLAG_DELAY, milliseconds of time, {}},

    ...

    Setting ending by predefined flag

    {REGFLAG_END_OF_TABLE, 0x00, {}}
    */

    //DCS write ,no parameter  Data type:05
    //DCS write ,1  parameter  Data type:15
    //DCS write ,long Write    Data type:39
    {0xFF,4,{0xAA,0x55,0x25,0x01}},
    {0x6F,1,{0x16}},
    {0xF7,1,{0x10}},

    {0XFF,4,{0XAA,0X55,0X25,0X01}},  //parameter 4
    {0X6F,1,{0X21}},
    {0XF7,1,{0X01}},
    {0X6F,1,{0X21}},
    {0XF7,1,{0X00}},
    {0XFC,1,{0X08}},
    {0XFC,1,{0X00}},
    {0X6F,1,{0X1A}},
    {0XF7,1,{0X05}},
    {0XFF,4,{0XAA,0X55,0X25,0X00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xB1,2,{0x68,0x21}},
    {0xB6,1,{0x08}},
    {0x6F,1,{0x02}},
    {0xB8,1,{0x08}},
    {0xBC,2,{0x00,0x00}},
    {0xBD,5,{0x01,0xF0,0xCA,0xCA,0x00}},
    {0xC8,1,{0x80}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
    {0xB3,2,{0x2D,0x2D}},
    {0xB4,2,{0x0F,0x0F}},
    {0xBC,2,{0x90,0x00}},
    {0xBD,2,{0x90,0x00}},
    {0xBE,1,{0x6A}},
    {0xB9,2,{0x33,0x33}},
    {0xBA,2,{0x25,0x25}},
    {0xC0,1,{0x0C}},
    {0xCA,1,{0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
    {0xEE,1,{0x01}},
    {0xB0,16,{0x00,0x00,0x00,0x0B,0x00,0x22,0x00,0x38,0x00,0x4E,0x00,0x6C,0x00,0x89,0x00,0xBE}},
    {0xB1,16,{0x00,0xE8,0x01,0x2F,0x01,0x68,0x01,0xC4,0x02,0x11,0x02,0x13,0x02,0x59,0x02,0x9E}},
    {0xB2,16,{0x02,0xC1,0x02,0xF9,0x03,0x14,0x03,0x42,0x03,0x60,0x03,0x83,0x03,0x9B,0x03,0xB4}},
    {0xB3,4,{0x03,0xE3,0x03,0xFF}},
    {0x6F,1,{0x17}},
    {0xF4,1,{0x60}},
    {0x6F,1,{0x11}},
    {0xF3,1,{0x01}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
    {0x6F,1,{0x02}},
    {0xBD,1,{0x0F}},
    {0xC2,1,{0xA1}},
    {0xD1,5,{0x03,0x00,0x3D,0x07,0x10}},
    {0xD2,5,{0x03,0x00,0x3D,0x07,0x10}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},
    {0xB0,2,{0x00,0x10}},
    {0xB1,2,{0x12,0x16}},
    {0xB2,2,{0x18,0x31}},
    {0xB3,2,{0x31,0x31}},
    {0xB4,2,{0x31,0x34}},
    {0xB5,2,{0x34,0x34}},
    {0xB6,2,{0x34,0x31}},
    {0xB7,2,{0x31,0x31}},
    {0xB8,2,{0x31,0x2D}},
    {0xB9,2,{0x2E,0x04}},
    {0xBA,2,{0x05,0x2E}},
    {0xBB,2,{0x2D,0x31}},
    {0xBC,2,{0x31,0x31}},
    {0xBD,2,{0x31,0x34}},
    {0xBE,2,{0x34,0x34}},
    {0xBF,2,{0x34,0x31}},
    {0xC0,2,{0x31,0x31}},
    {0xC1,2,{0x31,0x19}},
    {0xC2,2,{0x17,0x13}},
    {0xC3,2,{0x11,0x01}},
    {0xC4,2,{0x05,0x19}},
    {0xC5,2,{0x17,0x13}},
    {0xC6,2,{0x11,0x31}},
    {0xC7,2,{0x31,0x31}},
    {0xC8,2,{0x31,0x34}},
    {0xC9,2,{0x34,0x34}},
    {0xCA,2,{0x34,0x31}},
    {0xCB,2,{0x31,0x31}},
    {0xCC,2,{0x31,0x2E}},
    {0xCD,2,{0x2D,0x01}},
    {0xCE,2,{0x00,0x2D}},
    {0xCF,2,{0x2E,0x31}},
    {0xD0,2,{0x31,0x31}},
    {0xD1,2,{0x31,0x34}},
    {0xD2,2,{0x34,0x34}},
    {0xD3,2,{0x34,0x31}},
    {0xD4,2,{0x31,0x31}},
    {0xD5,2,{0x31,0x10}},
    {0xD6,2,{0x12,0x16}},
    {0xD7,2,{0x18,0x04}},
    {0xD8,5,{0x00,0x00,0x00,0x00,0x00}},
    {0xD9,5,{0x00,0x00,0x00,0x00,0x00}},
    {0xE5,2,{0x31,0x31}},
    {0xE6,2,{0x31,0x31}},
    {0xE7,1,{0x00}},
    {0x62,1,{0x01}},
    /*
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xEE,4,{0x87,0x78,0x02,0x40}},
    {0xEF,2,{0x07,0xFF}},
    */
    {0x11,1,{0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0x29,1,{0x00}},
    {REGFLAG_DELAY, 30, {}},
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

                case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                            = LCM_THREE_LANE;

    params->dsi.data_format.color_order             = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq               = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding                 = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format                  = LCM_DSI_FORMAT_RGB888;

    params->dsi.packet_size                         = 256;
    params->dsi.PS                                  = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active                = 8;
    params->dsi.vertical_backporch                  = 14;
    params->dsi.vertical_frontporch                 = 20;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 12;
    params->dsi.horizontal_backporch                = 110;
    params->dsi.horizontal_frontporch               = 110;
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    params->dsi.PLL_CLOCK                           = 299;
    params->dsi.ssc_disable                         = 1;
    params->dsi.esd_check_enable                    = 1;
    params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}

static void lcm_get_pin(void)
{
    static struct device_node *node;
    node = of_find_compatible_node(NULL, NULL, "mediatek,lcm_gpio_node");
    lcm_rst_pin = of_get_named_gpio(node, "lcm_rst_pin", 0);
    lcm_gpio_enn = of_get_named_gpio(node, "lcm_gpio_enn", 0);
    lcm_gpio_enp = of_get_named_gpio(node, "lcm_gpio_enp", 0);
    gpio_request(lcm_rst_pin, "lcm_rst_pin");
    gpio_request(lcm_gpio_enn, "lcm_gpio_enn");
    gpio_request(lcm_gpio_enp, "lcm_gpio_enp");
}

static void lcm_rst_pin_set(unsigned int GPIO, unsigned int output)
{
    lcm_get_pin();
    gpio_set_value(GPIO, output);
}

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
    {0x28, 0, {}},
    {REGFLAG_DELAY, 10, {}},
    {0x10, 0, {}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_init(void)
{
    lcm_get_pin();

    //gpio_direction_input(GPIO_READ_ID_PIN);//Set as input
    //gpio_get_value(GPIO_READ_ID_PIN);//Get the input value and save it to the variable
    gpio_direction_output(lcm_gpio_enn, 0);//Set to output
    MDELAY(10);
    gpio_direction_output(lcm_gpio_enp, 0);//Set to output
    MDELAY(10);
    gpio_set_value(lcm_gpio_enn, 1);//Set output level to high
    MDELAY(10);
    gpio_set_value(lcm_gpio_enp, 1);//Set output level to high
    MDELAY(10);

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);
    lcm_get_pin();

    //gpio_direction_input(GPIO_READ_ID_PIN);//Set as input
    //gpio_get_value(GPIO_READ_ID_PIN);//Get input value and save to variable
    gpio_direction_output(lcm_gpio_enn, 0);//Set to output
    MDELAY(10);
    gpio_direction_output(lcm_gpio_enp, 0);//Set to output
    MDELAY(10);
    gpio_set_value(lcm_gpio_enn, 0);//Set output level to low
    MDELAY(10);
    gpio_set_value(lcm_gpio_enp, 0);//et output level to low
    MDELAY(10);
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    int id=0;

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0] = 0x00063902;
    array[1] = 0x52AA55F0;
    array[2] = 0x00000108;
    dsi_set_cmdq(array, 3, 1);

    array[0] = 0x00033700; //read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xC5, buffer, 3);
    id = buffer[1]; //we only need ID

#ifdef BUILD_LK
    printf("nt35521 lk %s,buffer=%x,%x,%x,%x\n", __func__,buffer[0],buffer[1],buffer[2],id);
    printf("%s id = 0x%08x \n", __func__, id);
#else
    printk("nt35521 kernel %s \n", __func__);
    printk("%s id = 0x%08x \n", __func__, id);
#endif

    // It was not entirely clear to me, so force to 1 for now
#if 0
    return (LCM_ID == id)?1:0;
#else
    return 1;
#endif
}

static unsigned int rgk_lcm_compare_id(void)
{
    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
    #ifdef BUILD_LK
        printf("[adc_uboot]: get data error\n");
    #endif
        return 0;
    }
#endif

    lcm_vol = data[0]*1000+data[1]*10;

#ifdef BUILD_LK
    printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#else
    printk("[adc_kernel]: lcm_vol= %d\n",lcm_vol);
#endif
    if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE && lcm_compare_id())
    {
        return 1;
    }

    return 0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    return 1;
#ifndef BUILD_LK
    int array[4];
    char buf[5];
    int id=0;

    array[0] = 0x00033700;// read id return two byte,version and id
    atomic_set(&ESDCheck_byCPU,1);
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x04, buf, 3);
    atomic_set(&ESDCheck_byCPU,0);
    id = buf[1]; //we only need ID

    return (0x80 == id)?1:0;
#else
    return 0;
#endif
}

LCM_DRIVER nt35521_dsi_vdo_yushun_cmi_hd720_lcm_drv =
{
    .name           = "nt35521_dsi_vdo_yushun_cmi_hd720",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = rgk_lcm_compare_id,
    .ata_check      = lcm_ata_check,
};
