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
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_gpio.h>
	#include <mt-plat/mt_gpio.h>
//#include <linux/xlog.h>
#endif
// ---------------------------------------------------------------------------
//#include <cust_adc.h>    	
#define MIN_VOLTAGE (800)
#define MAX_VOLTAGE (1000)
#define LCM_ID (0x8394)

//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

unsigned static int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))

#define UDELAY(n)               (lcm_util.udelay(n))
#define MDELAY(n)               (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)             lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)           lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#if 1
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#else
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.rgk_dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#endif
#define   LCM_DSI_CMD_MODE                                      0

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static LCM_setting_table_V3 lcm_initialization_setting[] = {
	{0x39,0xB9,3,{0xFF,0x83,0x94}},
     
     {0x39,0xBA,8,{0x72,0x83, 0xA0, 0x6D, 0xB2, 0x00, 0x00, 0x40}},
     
     //{0x39,0xB1,15,{0x7C,0x12,0x12,0x23,0x04,0x11,0xF1,0x81,0x66,0xE2,0x23,0x80,0xC0,0xD2,0x58}},//SET POWER
     {0x39,0xB1,15,{0x7C,0x12,0x12,0x23,0x04,0x11,0xF1,0x80,0xde,0xE3,
	                0x23,0x80,0xC0,0xD2,0x58}},//SET POWER
     
     {0x39,0xB2,11,{0x00,0x64,0x0E,0x0D,0x12,0x23,0x08,0x08,0x1C,0x4D,
	                0x00}},
     
     {0x39,0xB4,12,{0x00,0xFF,0x5C,0x5A,0x5C,0x5A,0x5C,0x5A,0x01,0x76,
	                0x01,0x76}},

     {0x39,0xBF,3,{0x41,0x0E,0x01}},
     
     {0x39,0xD3,30,{0x00,0x00,0x00,0x00,0x00,0x12,0x10,0x32,0x10,0x00,
	                0x00,0x00,0x32,0x13,0xC0,0x00,0x00,0x32,0x10,0x08,
					0x00,0x00,0x47,0x04,0x02,0x02,0x47,0x04,0x00,0x47}},

     {0x39,0xD5,44,{0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x20,0x21,
	                0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x19,0x19,0x18,0x18}},       

     {0x39,0xD6,44,{0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x23,0x22,
	                0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x19,0x19}},

     {0x39,0xE0,42,{0x00,0x08,0x0B,0x33,0x3A,0x3F,0x19,0x3D,0x06,0x09,0x0D,0x17,0x0E,0x12,0x14,0x13,0x14,0x07,0x12,0x19,0x1D,
	                0x00,0x07,0x0A,0x33,0x3A,0x3F,0x19,0x3D,0x07,0x09,0x0D,0x17,0x0F,0x12,0x15,0x12,0x13,0x06,0x13,0x1A,0x1E}},

     {0x15,0xCC,1,{0x09}}, 

     {0x39,0xC7,4,{0x00,0xC0,0x40,0xC0}},	 

     {0x39,0xC0,2,{0x30,0x14}},	 
     
     {0x15,0xBC,1,{0x07}},    

     {0x39,0xB6,2,{0x64,0x64}},	 //68
     
     {0x15,0xB9,1,{0x00}},  

     {0x05,0x11,0,{}},
     {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,200,{}},
     
     //  29,DISP-ON
     {0x05,0x29,0,{}},
     {REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,10,{}},

};

static void lcm_register(void)
{
		unsigned int  data_array[32];
			
		data_array[0] = 0x00043902;
		data_array[1] = 0x9483FFB9;
		dsi_set_cmdq(&data_array, 2, 1);
		MDELAY(10);
		 
		data_array[0] = 0x00033902; 						
		data_array[1] = 0x008372BA;
		//data_array[2] = 0x0909b265;
		//data_array[3] = 0x00001040;
		dsi_set_cmdq(&data_array, 2, 1);
		MDELAY(3);

		data_array[0] = 0x00103902; 						
		data_array[1] = 0x12126CB1;//7c
		data_array[2] = 0xF1110424;
		data_array[3] = 0x23982E81;//0x23543A81
		data_array[4] = 0x58D2C080;
		dsi_set_cmdq(&data_array, 5, 1);
		MDELAY(5);
	
		data_array[0] = 0x000C3902; 						
		data_array[1] = 0x106400B2;
		data_array[2] = 0x081C3207;
		data_array[3] = 0x004D1C08;
		dsi_set_cmdq(&data_array, 4, 1);

		data_array[0] = 0x000D3902; 						
		data_array[1] = 0x50FF00B4;
		data_array[2] = 0x50515051;//5a
		data_array[3] = 0x026a0251;//
		data_array[4] = 0x0000006a;
		dsi_set_cmdq(&data_array, 5, 1);
		
		 
		//0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0xCC,0x00,0x00,0x00,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,0x67,0x45,0x23,0x01,0x23,0x88,0x88,0x88,0x88}},
		  
		data_array[0] = 0x07BC1500;//			  
		dsi_set_cmdq(&data_array, 1, 1);
		 

		data_array[0] = 0x00043902; 						
		data_array[1] = 0x010E41BF;
		dsi_set_cmdq(&data_array, 2, 1);

//		data_array[0] = 0x55D21500; 		 
//		dsi_set_cmdq(&data_array, 1, 1);
		 
		 
		data_array[0] = 0x001f3902; 						
		data_array[1] = 0x000600D3;//0f
		data_array[2] = 0x00000740; 						
		data_array[3] = 0x00081032;
		data_array[4] = 0x0f155208; 						
		data_array[5] = 0x10320f05;
		data_array[6] = 0x47000000; 						
		data_array[7] = 0x470c0c44;
		data_array[8] = 0x00470c0c;
		//data_array[9] = 0x0A000000;	
		//data_array[10] = 0x00000100;					  
		dsi_set_cmdq(&data_array, 9, 1);
		
 
		data_array[0] = 0x002D3902; 						
		data_array[1] = 0x002120D5;
		data_array[2] = 0x04030201;
		data_array[3] = 0x18070605;
		data_array[4] = 0x18181818;
		data_array[5] = 0x18181818;
		data_array[6] = 0x18181818;
		data_array[7] = 0x18181818;
		data_array[8] = 0x18181818;
		data_array[9] = 0x18181818;
		data_array[10] = 0x19181818;
		data_array[11] = 0x24181819;
		data_array[12] = 0x00000025;
		dsi_set_cmdq(&data_array, 13, 1);
		

		data_array[0] = 0x002D3902; 						
		data_array[1] = 0x072524D6;
		data_array[2] = 0x03040506;
		data_array[3] = 0x18000102;
		data_array[4] = 0x18181818;
		data_array[5] = 0x58181818;
		data_array[6] = 0x18181858;
		data_array[7] = 0x18181818;
		data_array[8] = 0x18181818;
		data_array[9] = 0x18181818;
		data_array[10] = 0x18181818;
		data_array[11] = 0x20191918;
		data_array[12] = 0x00000021;
		dsi_set_cmdq(&data_array, 13, 1);
		

		data_array[0] = 0x002B3902;						  
		data_array[1] = 0x140E00E0;
		data_array[2] = 0x213f3530;//1d
		data_array[3] = 0x0B090444;
		data_array[4] = 0x15120F17;
		data_array[5] = 0x15081513;
		data_array[6] = 0x0E001B16;
		data_array[7] = 0x3f353014;
		data_array[8] = 0x09044421;
		data_array[9] = 0x120F170B;
		data_array[10] = 0x08151315;
		data_array[11] = 0x001B1615;
		dsi_set_cmdq(&data_array, 12, 1);

		data_array[0] = 0x09CC1500;  
		dsi_set_cmdq(&data_array, 1, 1);
		
		data_array[0] = 0x00033902; 						
		data_array[1] = 0x001430C0;
		dsi_set_cmdq(&data_array, 2, 1);
		
		   
		data_array[0] = 0x00053902; 						
		data_array[1] = 0x40C000C7;
		data_array[2] = 0x000000C0;
		dsi_set_cmdq(&data_array, 3, 1);
		
		 
		data_array[0] = 0x00033902; 						
		data_array[1] = 0x009595B6;//7c-85
		dsi_set_cmdq(&data_array, 2, 1);
		
	/*	data_array[0] = 0x88DF1500; 		   
		dsi_set_cmdq(&data_array, 1, 1);
		
		data_array[0] = 0x00351500;//			  
		dsi_set_cmdq(&data_array, 1, 1); 
*/		
		 
		data_array[0] = 0x00110500;   
		dsi_set_cmdq(&data_array, 1, 1);
		MDELAY(120);
		
		data_array[0] = 0x00290500;   
		dsi_set_cmdq(&data_array, 1, 1);
		MDELAY(10);

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_THREE_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting	
	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	params->dsi.vertical_sync_active				= 6;
	params->dsi.vertical_backporch					= 10;
	params->dsi.vertical_frontporch					= 12;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 20;//24;
	params->dsi.horizontal_backporch				= 60;
	params->dsi.horizontal_frontporch				= 60;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	//params->dsi.LPX=8; 

	// Bit rate calculation
	params->dsi.PLL_CLOCK= 253; //[D5110_qmobile][bug id:DWYYL-2068]modified by lijian for rf test 280->253  20150730
	params->dsi.ssc_disable                         = 1;
//	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
//	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
//	#if (LCM_DSI_CMD_MODE)
//	params->dsi.fbk_div =7;

	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
	
	params->dsi.lcm_esd_check_table[1].cmd          = 0xd9;
	params->dsi.lcm_esd_check_table[1].count        = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;

	params->dsi.lcm_esd_check_table[2].cmd          = 0x09;
	params->dsi.lcm_esd_check_table[2].count        = 3;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[2].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[2].para_list[2] = 0x06;

}


static void lcm_init(void)
{
/*
	#ifdef BUILD_LK
		printf("sym  lcm_init start\n");
	#else
		printk("sym  lcm_init  start\n");
	#endif

	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(12);
	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(40);
*/
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	//dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
	lcm_register();
	#ifdef BUILD_LK
		printf("sym  lcm_init end\n");
	#else
		printk("sym  lcm_init end\n");
	#endif
}


static  LCM_setting_table_V3 lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x05, 0x28, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

	// Sleep Mode On
	{0x05, 0x10, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};

static void lcm_suspend(void)
{	
	unsigned int data_array[35];
#if 0
	//dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(40);
#endif	
	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	SET_RESET_PIN(0);
	MDELAY(10);
    //disable VSP & VSN
    /*lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
	MDELAY(12);
	lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
	MDELAY(12);
    MDELAY(5);	*/
	SET_RESET_PIN(1);
	MDELAY(120);
}


static void lcm_resume(void)
{
	lcm_init();
	
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif


static unsigned int lcm_compare_id(void)
{

#if 1
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;
/*	
    lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(12);
    lcm_util.set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(12);
    */
#if 1
	SET_RESET_PIN(1);
    MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);
#endif
	array[0] = 0x00043902;
	array[1] = 0x9483FFB9;
	dsi_set_cmdq(array, 2, 1);	

	//array[0] = 0x00033902;
	//array[1] = 0x008372BA;
	//dsi_set_cmdq(array, 2, 1);	
	array[0] = 0x00093902;
	array[1] = 0xa08372BA;
	array[2] = 0x0000B26D;
	array[3] = 0x00000040;
	dsi_set_cmdq(array, 4, 1);	
	
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0x04, buffer, 4);
	id = buffer[0]<<8|buffer[1];//we only need ID
	
	#ifdef BUILD_LK
		printf("hx8394 uboot %s,buffer=%x,%x,%x,%x\n", __func__,buffer[0],buffer[1],buffer[2],id);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("hx8394 kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
	   
	return (id == LCM_ID) ? 1 : 0;
#endif 
}

// zhoulidong  add for lcm detect (start)
static int rgk_lcm_compare_id(void)
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
		printf("sym lk [adc_uboot]: get data error\n");
	#endif
		return 0;
	}
#endif

	lcm_vol = data[0]*1000+data[1]*10;

#ifdef BUILD_LK
	printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#endif
	
	if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE && lcm_compare_id())
	{
		return 1;
	}

	return 0;
}

// zhoulidong  add for lcm detect (end)


// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
#ifdef BUILD_LK
	return FALSE;
#else
	char  buffer9[4] ={0};
	char  bufferA[3]={0};
	char  bufferD9[3]={0};
	char  b1_buffer[11]={0};
	int   array[4];

	array[0] = 0x00043902;
	array[1] = 0x9483FFB9;
	dsi_set_cmdq(array, 2, 1);	

	array[0] = 0x00093902;
	array[1] = 0xa08372BA;
	array[2] = 0x0000B26D;
	array[3] = 0x00000040;
	dsi_set_cmdq(array, 4, 1);	

	array[0] = 0x00043700; // return fore byte
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x09, buffer9, 4);

	array[0] = 0x00013700; // return one byte
	dsi_set_cmdq(array, 1, 1); 
	read_reg_v2(0x0A, bufferA, 1);

	array[0] = 0x00013700; // return one byte
	dsi_set_cmdq(array, 1, 1); 
	read_reg_v2(0xD9, bufferD9, 1);	
	
	array[0] = 0x000A3700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xb1, b1_buffer, 10);
	printk("<%s:%d>b1_buffer[0]=%x [1]=%x [2]=%x [3]=%x [4]=%x [5]=%x [6]=%x [7]=%x [8]=%x [9]=%x\n", __func__, __LINE__, b1_buffer[0], b1_buffer[1], b1_buffer[2], b1_buffer[3], b1_buffer[4], b1_buffer[5], b1_buffer[6], b1_buffer[7], b1_buffer[8], b1_buffer[9]); 

	array[0] = 0x00023902;
	array[1] = 0x000009CC;
	dsi_set_cmdq(array, 2, 1);	

	array[1] = 0x00380500;
	dsi_set_cmdq(array, 1, 1);	

	array[1] = 0x00130500;
	dsi_set_cmdq(array, 1, 1);	

	array[1] = 0x00110500;
	dsi_set_cmdq(array, 1, 1);	

	array[1] = 0x00290500;
	dsi_set_cmdq(array, 1, 1);	

	array[0] = 0x00023902;
	array[1] = 0x000000B9;
	dsi_set_cmdq(array, 2, 1);	

	printk("sym hx8394d_dsi_vdo_hlt_hsd_hd720 0x0A=0x%x,0xd9=0x%x,0x09=0x%x,0x%x,0x%x,0x%x\n",bufferA[0],bufferD9[0],buffer9[0],buffer9[1],buffer9[2],buffer9[3]);   
	if((buffer9[0]==0x80)&&(buffer9[1]==0x73)&&(buffer9[2]==0x04)&&(bufferA[0]==0x1C)&&(bufferD9[0]==0x80)&&(b1_buffer[7]==0x80)&&(b1_buffer[8]==0xde)&&(b1_buffer[9]==0xe3))
	{
		printk("sym hx8394d_dsi_vdo_hlt_hsd_hd720 %s %d FALSE\n", __func__, __LINE__);
		return FALSE;
	}
	else
	{	
		printk("sym hx8394d_dsi_vdo_hlt_hsd_hd720 %s %d TRUE\n", __func__, __LINE__);
		return TRUE;
	}
#endif
}

static unsigned int lcm_esd_recover(void)
{
#ifdef BUILD_LK
	printf("sym lcm_esd_recover()\n");
#else
	printk("sym lcm_esd_recover()\n");
#endif	
	
	lcm_init();	

	return TRUE;
}
// zhoulidong add for eds(end)



//add by yangjuwei 
static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	int array[4];
	char buff[5];
	char id_high=0;
	char id_low=0;
	int id=0;
	
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0x04, buff, 4);
	id = buff[0]<<8|buff[1];
	printk("%s id = %d",__func__,id);
	return (LCM_ID == id) ? 1:0;
#else
	return 0;
#endif
}

LCM_DRIVER hx8394d_dsi_vdo_hlt_hsd_hd720_lcm_drv = 
{
	.name               = "hx8394d_dsi_vdo_hlt_hsd_hd720",
	.set_util_funcs     = lcm_set_util_funcs,
	.get_params         = lcm_get_params,
	.init               = lcm_init,
	.suspend            = lcm_suspend,
	.resume             = lcm_resume,
	.compare_id         = lcm_compare_id,
	//.esd_check        = lcm_esd_check,
	//.esd_recover      = lcm_esd_recover,
	.ata_check          = lcm_ata_check,
#if (LCM_DSI_CMD_MODE)
	//.set_backlight    = lcm_setbacklight,
	//.update           = lcm_update,
#endif
};

