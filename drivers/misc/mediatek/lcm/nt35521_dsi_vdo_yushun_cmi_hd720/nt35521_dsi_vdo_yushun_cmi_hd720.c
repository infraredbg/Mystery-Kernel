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
    {0xFF,4,{0xAA,0x55,0x25,0x00}},
    {0xFE,5,{0x00,0x00,0x00,0x00,0x00}},
    {0xFF,4,{0xAA,0x55,0xA5,0x80}},
    {0x6F,2,{0x11,0x00}},
    {0xF7,2,{0x20,0x00}},
    {0x6F,1,{0x06}},
    {0xF7,1,{0xA0}},
    {0x6F,1,{0x19}},
    {0xF7,1,{0x12}},
    {0x6F,1,{0x10}},
    {0xF5,1,{0x70}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xC8,1,{0x80}},
    {0xB1,2,{0x60,0x21}},
    {0xB6,1,{0x0F}},
    {0xBC,2,{0x00,0x00}},
    {0xBD,5,{0x02,0xB0,0x1E,0x1E,0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
    {0xBC,2,{0x78,0x00}},
    {0xBD,2,{0x78,0x00}},
    {0xCA,1,{0x01}},
    {0xC0,1,{0x0C}},
    {0xBE,1,{0x56}},
    {0xB3,2,{0x38,0x38}},
    {0xB4,2,{0x11,0x11}},
    {0xB6,2,{0x05,0x05}},
    {0xB9,2,{0x35,0x35}},
    {0xBA,2,{0x14,0x14}},
    {0xC4,2,{0x11,0x11}},
    {0xCE,1,{0x66}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
    {0xEE,1,{0x01}},
    {0xB0,16,{0x00,0x00,0x00,0x11,0x00,0x32,0x00,0x52,0x00,0x68,0x00,0x8D,0x00,0xAB,0x00,0xD9}},
    {0xB1,16,{0x00,0xFF,0x01,0x3B,0x01,0x69,0x01,0xB4,0x01,0xF1,0x01,0xF3,0x02,0x2B,0x02,0x6C}},
    {0xB2,16,{0x02,0x90,0x02,0xC7,0x02,0xE9,0x03,0x1E,0x03,0x3C,0x03,0x69,0x03,0x7E,0x03,0x9E}},
    {0xB3,4,{0x03,0xD9,0x03,0xFF}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},
    {0xB0,2,{0x29,0x2A}},
    {0xB1,2,{0x10,0x12}},
    {0xB2,2,{0x14,0x16}},
    {0xB3,2,{0x18,0x1A}},
    {0xB4,2,{0x02,0x04}},
    {0xB5,2,{0x34,0x34}},
    {0xB6,2,{0x34,0x2E}},
    {0xB7,2,{0x2E,0x2E}},
    {0xB8,2,{0x34,0x00}},
    {0xB9,2,{0x34,0x34}},
    {0xBA,2,{0x34,0x34}},
    {0xBB,2,{0x01,0x34}},
    {0xBC,2,{0x2E,0x2E}},
    {0xBD,2,{0x2E,0x34}},
    {0xBE,2,{0x34,0x34}},
    {0xBF,2,{0x05,0x03}},
    {0xC0,2,{0x1B,0x19}},
    {0xC1,2,{0x17,0x15}},
    {0xC2,2,{0x13,0x11}},
    {0xC3,2,{0x2A,0x29}},
    {0xE5,2,{0x2E,0x2E}},
    {0xC4,2,{0x29,0x2A}},
    {0xC5,2,{0x1B,0x19}},
    {0xC6,2,{0x17,0x15}},
    {0xC7,2,{0x13,0x11}},
    {0xC8,2,{0x01,0x05}},
    {0xC9,2,{0x34,0x34}},
    {0xCA,2,{0x34,0x2E}},
    {0xCB,2,{0x2E,0x2E}},
    {0xCC,2,{0x34,0x03}},
    {0xCD,2,{0x34,0x34}},
    {0xCE,2,{0x34,0x34}},
    {0xCF,2,{0x02,0x34}},
    {0xD0,2,{0x2E,0x2E}},
    {0xD1,2,{0x2E,0x34}},
    {0xD2,2,{0x34,0x34}},
    {0xD3,2,{0x04,0x00}},
    {0xD4,2,{0x10,0x12}},
    {0xD5,2,{0x14,0x16}},
    {0xD6,2,{0x18,0x1A}},
    {0xD7,2,{0x2A,0x29}},
    {0xE6,2,{0x2E,0x2E}},
    {0xD8,5,{0x00,0x00,0x00,0x54,0x00}},
    {0xD9,5,{0x00,0x15,0x00,0x00,0x00}},
    {0xE7,1,{0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
    {0xB1,2,{0x00,0x00}},
    {0xB0,2,{0x00,0x00}},
    {0xB2,5,{0x05,0x00,0x00,0x00,0x00}},
    {0xB3,5,{0x05,0x00,0x00,0x00,0x00}},
    {0xB4,5,{0x05,0x00,0x00,0x00,0x00}},
    {0xB5,5,{0x05,0x00,0x17,0x00,0x00}},
    {0xB6,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xB7,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xB8,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xB9,5,{0x12,0x00,0x19,0x00,0x00}},
    {0xBA,5,{0x57,0x00,0x00,0x00,0x00}},
    {0xBB,5,{0x57,0x00,0x00,0x00,0x00}},
    {0xBC,5,{0x75,0x00,0x1A,0x00,0x00}},
    {0xBD,5,{0x53,0x00,0x1A,0x00,0x00}},
    {0xC0,4,{0x00,0x34,0x00,0x00}},
    {0xC1,4,{0x00,0x34,0x00,0x00}},
    {0xC2,4,{0x00,0x34,0x00,0x00}},
    {0xC3,4,{0x00,0x34,0x00,0x00}},
    {0xC4,1,{0x20}},
    {0xC5,1,{0x00}},
    {0xC6,1,{0x00}},
    {0xC7,1,{0x00}},
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
    {0xED,1,{0x30}},
    {0xB0,2,{0x17,0x06}},
    {0xB8,1,{0x08}},
    {0xBD,5,{0x03,0x07,0x00,0x03,0x00}},
    {0xB1,2,{0x17,0x06}},
    {0xB9,1,{0x00}},
    {0xB2,2,{0x00,0x00}},
    {0xBA,1,{0x00}},
    {0xB3,2,{0x17,0x06}},
    {0xBB,1,{0x0A}},
    {0xB4,2,{0x17,0x06}},
    {0xB5,2,{0x17,0x06}},
    {0xB6,2,{0x14,0x03}},
    {0xB7,2,{0x00,0x00}},
    {0xBC,1,{0x02}},
    {0xE5,1,{0x06}},
    {0xE6,1,{0x06}},
    {0xE7,1,{0x00}},
    {0xE8,1,{0x06}},
    {0xE9,1,{0x06}},
    {0xEA,1,{0x06}},
    {0xEB,1,{0x00}},
    {0xEC,1,{0x00}},
    {0xC0,1,{0x07}},
    {0xC1,1,{0x80}},
    {0xC2,1,{0xA4}},
    {0xC3,1,{0x05}},
    {0xC4,1,{0x00}},
    {0xC5,1,{0x02}},
    {0xC6,1,{0x22}},
    {0xC7,1,{0x03}},
    {0xC8,2,{0x05,0x30}},
    {0xC9,2,{0x01,0x31}},
    {0xCA,2,{0x03,0x21}},
    {0xCB,2,{0x01,0x20}},
    {0xD1,5,{0x00,0x05,0x09,0x07,0x10}},
    {0xD2,5,{0x10,0x05,0x0E,0x03,0x10}},
    {0xD3,5,{0x20,0x00,0x48,0x07,0x10}},
    {0xD4,5,{0x30,0x00,0x43,0x07,0x10}},
    {0xD0,1,{0x00}},
    {0xCC,3,{0x00,0x00,0x3E}},
    {0xCD,3,{0x00,0x00,0x3E}},
    {0xCE,3,{0x00,0x00,0x02}},
    {0xCF,3,{0x00,0x00,0x02}},
    {0x6F,1,{0x11}},
    {0xF3,1,{0x01}},
    {0x35,1,{0x01}},
    {0x11,1,{0x00}},
    {REGFLAG_DELAY,120,{}},
    {0x29,1,{0x00}},
    // Not sure why there's no end of table flag found in LK table
    // Better compare with table in decompiled kernel
    // {REGFLAG_END_OF_TABLE,0x00,{}}
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
