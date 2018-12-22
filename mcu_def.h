/********************************
*   @ file hal.h
*   @ desc CPU
*   @ version V1.0.0
*********************************/
#ifndef MCU_DEF_H
#define MCU_DEF_H

#define LOC_NEAR


#ifdef STM8S003
#include "stm8s.h"
#define LOC_NEAR @near

#elif defined(STM8L15X_MD)
#include "stm8l15x.h"
//#define NEAR @near
extern const GPIO_TypeDef * PORT[];
extern const EXTI_Pin_TypeDef EXTI_Pin[];
extern const uint16_t EXTI_IT_Pin[];
extern const uint8_t GPIO_Pin[];


#elif defined(STM8L05X_LD_VL)
#include "stm8l15x.h"
//#define NEAR @near

#elif defined(MINGW)
#include <stdint.h>
typedef uint32_t  u32;
typedef uint16_t u16;
typedef unsigned char  u8;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

#elif defined(S3C2440)
#include<stdint.h>
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

#elif defined(S3C2416)
#include<stdint.h>
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

#ifndef int8_t
//  typedef char int8_t;
#endif

#elif defined(STM32)
#include<stdint.h>
//#include<stm32f10x.h>
typedef uint32_t  u32_t;
typedef uint16_t u16_t;
typedef uint8_t  u8_t;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t  u32;

#define SYSCLK_FREQ_72MHz  72000000
#define LWIP_NO_STDINT_H 1
typedef int32_t socklen_t;
#define SOCKLEN_T_DEFINED 1

#elif defined(LPC178X)

#elif defined(WINCE) ||defined(USE_HAL_DRIVER)  //STM32_HAL
#ifdef STM32F103xE
#include "stm32f1xx_hal.h"
#endif
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "timers.h"

typedef signed   char   int8_t;
typedef unsigned char   uint8_t;

typedef signed   short  int16_t;
typedef unsigned short  uint16_t;

typedef signed   long   int32_t;
typedef unsigned long   uint32_t;
typedef uint32_t  u32_t;
typedef uint16_t u16_t;
typedef uint8_t  u8_t;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t  u32;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

#elif defined(QT) || defined(QT_CORE_LIB)
typedef unsigned int uint32_t;
typedef int  s32;
typedef short s16;
typedef char  s8;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int  u32;
#elif defined(WIN32)
//#include<windows.h>
typedef signed   char   int8_t;
typedef unsigned char   uint8_t;

typedef signed   short  int16_t;
typedef unsigned short  uint16_t;
typedef int   int32_t;
typedef unsigned uint32_t;
typedef int8_t  s8;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t  u32;
#elif defined(CC2530)
#include <hal_types.h>
typedef signed   char   int8_t;
typedef unsigned char   uint8_t;

typedef signed   short  int16_t;
typedef unsigned short  uint16_t;

typedef signed   long   int32_t;
typedef unsigned long   uint32_t;

#if USE_ASSERT
/**
 * @brief      Assert Function
 *
 * @param[in]  expr  Expression to be evaluated
 *
 * @return     None
 *
 * @details    If the expression is false, an error message will be printed out
 *             from debug port (UART0 or UART1).
 */
#define ASSERT_PARAM(expr)  { if (!(expr)) { AssertError((uint8_t*)__FILE__, __LINE__); } }

void AssertError(uint8_t* file, uint32_t line);
#else
#define ASSERT_PARAM(expr)
#endif

#define assert_param(expr)  ASSERT_PARAM(expr)
#elif defined(linux)
#elif defined(__ANDROID_API__)
#include <stdint.h>
#else
#warning "!!! must define cpu type"
#endif



#ifdef CC2530       //
#define fprintf(x,...)
#endif

#ifndef NULL
#define NULL 0
#endif

#if defined(QT)
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>
#endif

#ifdef RT_THREAD
#include <rtthread.h>
#include <rthw.h>
void rtthread_startup(void);
void rtthread_exit(void);
void rtthread_run(void);

#endif

# if defined(DLLBUILD)
/* define DLLBUILD when building the DLL */
#  define DLL_API __declspec(dllexport)
# else
# define DLL_API
#endif

typedef struct
{
    uint8_t port;
    uint8_t pin;
} io_def;

//  #ifndef STM32
//  typedef enum
//  {
//      EXTI_Trigger_Rising = 0x08,
//      EXTI_Trigger_Falling = 0x0C,
//      EXTI_Trigger_Rising_Falling = 0x10
//  } EXTITrigger_TypeDef;
//  #endif

uint16_t bswap_16(uint16_t x);
uint32_t bswap_32(uint32_t x);

void bit_value(int reg, int bit, uint8_t value);
void set_bit(int reg, int bit);
void clear_bit(int reg, int bit);
void set_bits(int reg, int bit, int width, int value);




#ifdef DEBUG
#define kprintk (printf("%s[%d]:", __FILE__, __LINE__), printf)
#else
#define kprintk
#endif



#define parent_struct(obj, type, member)   ((type *)((char *)(obj) - (unsigned long)(&((type *)0)->member)))



#define PROG_FLAG_TRY 0x1234
#define PROG_FLAG_IAP 0x1

//* program name
extern const u8* progName;
//* build number from jenkins
extern const u16 buildNumber;

struct devicedesc
{
    uint16_t cmd;
    unsigned char model[8];
    unsigned int bootMode;
    unsigned int bootver;
};

struct CMD_BOOT_UP
{
    uint16_t cmd;
    unsigned int Blength;
    unsigned int bootVer;
    unsigned int checksum;
    uint16_t status;
};


struct burn_in
{
    uint16_t cmd;
    uint16_t BlockNum;
    unsigned int Blength;       //
    unsigned int StartAddress;
    uint16_t checksum;
    unsigned char buf[64];
};


#ifndef MCU_TINY
enum CMD_MODBUS
{
    CMD_CLOCK_YEAR = 10000,
    CMD_CLOCL_MONTH,
    CMD_CLOCK_DAY,
    CMD_CLOCK_HOUR,
    CMD_CLOCK_MINUTE,
    CMD_CLOCK_SECOND,
    CMD_CLOCK_READ,
    CMD_CLOCK_WRITE,
    CMD_CLOCK_YEAR_FLOAT = 11000,
    CMD_CLOCK_MONTH_FLOAT = 11002,
    CMD_CLOCK_DAY_FLOAT = 11004,
    CMD_CLOCK_HOUR_FLOAT = 11006,
    CMD_CLOCK_MINUTE_FLOAT = 11008,
    CMD_CLOCK_SECOND_FLOAT = 11010,
    MODBUS_MEM_ADDR =20000,
    MODBUS_MEM_DATA =20100,
    CMD_BLOCK_WRITE = 22000,
    CMD_BOOT_VER_READ,      //
    CMD_ENTER_IAP,          //
    CMD_BOOT_UPDATE,        //
    CMD_BOOT_DATA,          //
    CMD_READ_DATA,          //
    CMD_WRITE_DATA,          //
    CMD_RESTORE = 22222,  //
    CMD_REBOOT = 23456,
    CMD_RECIPE_NR= 30001, //
    CMD_WATCH_DOG_EN,
    CMD_WATCHDOG_INTERVAL=40000,
    CMD_PLC_PERIOD=40001,
    CMD_MOUSE_CLICK,
    CMD_MOUSE_CLICK_1,
    CMD_GET_WINDOW_NAME,        //
    CMD_GET_MEM_TOTAL,
    CMD_GET_MEM_REMAIN,
    CMD_GET_MODBUS_ORDER_2 = 49990,
    CMD_GET_MODBUS_ORDER_4,
    MODBUS_CH_AI_CALICATION=50000,
    MODBUS_CH_AI_CALICATION_EN=51234,
    CMD_MODBUS_HISTORY_BLK=60000,
    CMD_MODBUS_HISTORY_ITEM,
    CMD_MODBUS_HISTORY_CTRL,
    CMD_MODBUS_HISTORY_DATA=61000,
};

#endif

#define HAL_PASSWORD_FLAG_EN_MODIFY
typedef struct
{
    char *pass;
    char *orig; //
    int flag;
} password_t;


typedef struct
{
    int ch;
    unsigned port;
    unsigned pin;
} hal_di_t;

typedef struct
{
    int ch;
    unsigned port;
    unsigned pin;
    unsigned char irqed;
} di_irq_dat_t;


#ifdef AT070TN83
#define HDP  800
#define HT  850
#define HPS  88
#define LPS  3
#define HPW  48

#define VDP  480
#define VTT  530
#define VPS  50
#define FPS  30
#define VPW  3
#elif defined(AT056TN83)
#define HDP 639
#define  HT 800
#define HPS  134
#define LPS  16
#define HPW  10

#define VDP 479
#define VT  524
#define VPS  32
#define FPS  11
#define VPW  2
#elif defined(AT056TN83_HV)
#define HDP 639
#define  HT 799
#define HPS  169
#define LPS  26
#define HPW  9

#define VDP 479
#define VT  524
#define VPS  44
#define FPS  33
#define VPW  1
#elif defined(LB0432)
#define HDP 639
#define  HT 800
#define HPS  134
#define LPS  16
#define HPW  10

#define VDP 479
#define VT  524
#define VPS  32
#define FPS  11
#define VPW  2
#endif

typedef struct
{
    unsigned short w,h;         //
    char rorate;                //
    char x_mirror,y_mirror;     //
    char *caption;
} lcd_t;


typedef struct
{
    uint16_t hdp;
    uint16_t  ht;
    uint16_t hps;
    uint16_t lps;
    uint16_t hpw;

    uint16_t vdp;
    uint16_t vt;
    uint16_t vps;
    uint16_t fps;
    uint16_t vpw;
    uint8_t x_mirror;
    uint8_t width;          //:0=24bit,1=16bit
    uint8_t bgr;        //:0=RGB,1=BGR
} ssd1963_p;

extern const ssd1963_p hw_ssd1963;


typedef struct
{
    unsigned int ch;
    unsigned int value;
    const io_def *port;
    unsigned int us;
    unsigned short Polarity;
    unsigned short Filter;
#ifdef STM32
    void *TIM;   //
    int channel;
#endif
} counter_dat_t;


typedef struct
{
    const io_def cs;
#ifdef STM32
    void *spi;
#endif
    const io_def miso;
    const io_def mosi;
    const io_def clk;
    unsigned int remap;
} spi_flash_def;

extern void (*module_init[])(void);

typedef struct
{
    unsigned int prog_address;      //
    unsigned int iap_address;   //
    void (*hw_rtc_init)();
    void (*hw_lcd_init)();
    void (*hw_touch_init)();
    short(*rtgui_mouse_click)(int index,unsigned short axia);
    unsigned short(*read_sw_value)();  //
} hw_config_t;

extern const hw_config_t kt_hw_config;



typedef struct
{
    const io_def sel;
    const io_def cs;
    const io_def reset;
    const io_def revp;
    const io_def sig;
#ifdef STM32
    void *spi;
#endif
    const io_def miso;
    const io_def mosi;
    const io_def clk;
    const char *spi_name;
} ATT7022CU_hw;
typedef struct
{
    unsigned int ch, gain;
} hal_Att7022_t;
extern const ATT7022CU_hw hw_att7022cu;
int hal_Att7022c_init(hal_Att7022_t *d);
int hal_Att7022_get_DINT(hal_Att7022_t *d,unsigned char idx);
void hal_Att7022c_set_value(hal_Att7022_t *d,unsigned char idx,int v);




struct filter_middle_t
{
    int val_last, val_last_last;
    //      int value;
};


struct filter_avr_t
{
    int sum;
    uint16_t times;
    uint16_t point;
    int max, min;
    int *buf;   //
};


typedef struct
{
    int value,pass;
} filter_pass_t;


typedef struct
{
    uint8_t ch, gain;
    int32_t val;
} hal_hx711_t;


typedef struct
{
    io_def pin_DOUT;
    io_def pin_SCK;
    unsigned int irq;
    unsigned int irq_line;
    hal_hx711_t *CH[2];
    int current;    //
} hx711_chip;


typedef struct
{
    int ch;
    unsigned port;
    unsigned pin;
} hal_lsdi_t;

typedef struct
{
    unsigned int bits;  //
    int val;
} hal_ssi_t;

typedef struct
{
    int ch;
    unsigned port;
    unsigned pin;
    unsigned int htime,ltime;
#ifdef RT_THREAD
    rt_thread_t thread;
#endif
    unsigned int val;
} hal_do_t;

typedef struct
{
    int ch;
    unsigned port;
    unsigned pin;
} hal_hcdo_t;

typedef struct
{
    int value;
} hal_key_t;

enum
{
    AI_TYPE_5V=0,
    AI_TYPE_420MA=1,
    AI_TYPE_PT100=2,
    AI_TYPE_KCOUPLE=3
};

typedef struct
{
    unsigned char check_sum;
    int humidity, temperature;
    unsigned int time_index;
    unsigned int time_data[48] ;
    unsigned int HT_data[5];
    char sel; //dht11/AM2303 sel
} hal_dht11_t;



typedef struct
{
    unsigned int ch, gain;
    float val,val_raw;
    int zero;       //
} hal_pt100_t;

#define  ADS1232_FLAG_POR_SINGLE  (0x1<<0) //
#define ADS1232_FLAG_AUTO_TRACK   (0x1<<1)    //
#define ADS1232_FLAG_IS_CALI      (0x1<<2)

typedef struct
{
    unsigned int ch, gain;
    //  hx711_chip *chip;
    //      unsigned int ab;
    unsigned int flag;
    unsigned int mask;
    int val,val_raw;
    int zero;       //
    float ranger, sense,r_val,r_zero;
    float dest;   //
    struct filter_avr_t filter;
    struct filter_middle_t mid_dat;
    filter_pass_t f_pass;
    int zero_time,lit_time;
} hal_ADS1232_t;


typedef struct
{
    io_def pin_DOUT;
    io_def pin_SCK;
    io_def g0;
    io_def g1;
    io_def pdwn;
    io_def speed;
    io_def a0;
    io_def  temp;
    unsigned int irq;
    unsigned int irq_line;
    hal_ADS1232_t *CH[2];
    int current;    //
} ADS1232_chip;


typedef struct
{
    unsigned int ch, gain;
    float vref;
    int val;
    float scale, ranger, sense;
    float zero;   //
    struct filter_avr_t filter;
} hal_tm7710_t;

typedef struct
{
    const io_def pin_DOUT;
    const io_def pin_SCK;
    const io_def pin_DRDY;
    hal_tm7710_t *CH[2];
#ifdef RT_THREAD
    rt_thread_t thread;
#endif
} TM7710_chip;


typedef struct
{
    unsigned int mask;
    float vref;
    int val;
    float zero;   //
    struct filter_avr_t filter;
} hal_TM7711_t;

typedef struct
{
    const io_def pin_DOUT;
    const io_def pin_SCK;
    unsigned int irq;
    unsigned int irq_line;
    hal_TM7711_t *hal;
} TM7711_chip;

typedef struct
{
    float t,rh,td;
#ifdef RT_THREAD
    rt_device_t port;
#endif
    char txbuf[8]; //
    char rxbuf[64];    //
} HMP110_dat;

typedef struct
{
    io_def port;

} hal_ds18b20_t;



typedef struct
{
    unsigned int ch;
    unsigned int freq;
    unsigned int last_freq;
    unsigned char filter_scale; //
    unsigned int counter;
    unsigned int last_count;
} pi_dat_t;


typedef struct
{
    const char *url;
    unsigned short port;
    unsigned int status;
} http_client_t;

//

#define PERIOD_TIMES 15
typedef struct
{
    unsigned int ch;
    unsigned int last_tick;
    unsigned int last_sysclk;
    unsigned int period, last; //,last_period;
    unsigned int min_time;
    unsigned int count; //
    float freq;
    unsigned char filter_scale; //
    int vaild;  //
    struct filter_middle_t filter_mid;
    struct filter_avr_t filter;
    int times,times2,times3;        //
#ifdef RT_THREAD
    rt_thread_t tmr;    //
#endif
    int buf[PERIOD_TIMES];
    int buf_index;
} hal_ppi_t;
void hal_ppi_get_freq_min(hal_ppi_t *p, float *v);

#define PWI_FLAG_ACTION     (1<<0)
typedef struct
{
    unsigned int flag;
    unsigned int ch;
    unsigned int value;
    unsigned int times; //
    unsigned int nr_times;  //
    unsigned short Polarity;
#ifdef STM32
    void *TIM;   //
    int channel;
#endif
    int buf[PERIOD_TIMES];
    uint8_t buf_len;
    int buf_index;
} hal_pwi_t;
void hal_pwi_get_freq_min(hal_pwi_t *p, float *v);
void hw_pwi_init(hal_pwi_t *dat);
void hw_pwi_set_us(hal_pwi_t *p);
void hw_pwi_set_edge_filter(hal_pwi_t *dat);



typedef struct
{
    unsigned int flag;
    unsigned int ch;
    unsigned int period_all;  //
    unsigned short period,last_value; //
    void *port; //encoder_port_def
    float freq,freq_min,freq_max;
    int counter;
    unsigned int us;
#ifdef STM32
    void *TIM;   //TIM_TypeDef
    int channel;
    unsigned short Filter;//0-15
#endif
} hal_pei_t;

enum
{
    OUT_MODE_SINGLE,    //
    OUT_MODE_DUAL,  //
    OUT_MODE_PD     //
};

#define PWM_FLAG_DIRECTION  (0x1<<5)
#define PWM_FLAG_RUN_CMD    (0x1<<4)  //
#define PWM_FLAG_GIVEN_MODE (0x1<<3)  //
#define PWM_FLAG_STOP_LEVEL (0x1<<2)  //
#define PWM_FLAG_COUNT_EN   (0x1<<1)  //

typedef struct
{
    unsigned char ch;
    unsigned int flag;
    unsigned char state;
    unsigned int set_freq;  //1000
    unsigned int run_freq;  //1000
    unsigned int dutycycle;
    unsigned int duty_max;
    unsigned int limit_max_freq;    //1000
    unsigned int limit_min_freq;    //1000
    unsigned int set_pluses,sumofpluse;
    int set_position,position;
    io_def porta, portb, portd;
    unsigned int period,ccwmode;
    unsigned char prescale;
    float up_time,down_time;  //
} hal_po_t;
int hw_pwm_init(hal_po_t *p);
int hw_po_starta(hal_po_t *p);
int hw_po_startb(hal_po_t *p);
int hw_po_stopa(hal_po_t *p);
int hw_po_stopb(hal_po_t *p);
int hw_po_dir(hal_po_t *p);
int hw_po_set_freq(hal_po_t *p);
int hw_po_set_hz(hal_po_t *p);
int hw_po_set_duty(hal_po_t *p);
int hw_po_set_pluses(hal_po_t *p);
int hw_po_set_position(hal_po_t *p);
int hw_po_get_position(hal_po_t *p);
int hw_po_get_running(hal_po_t *p);


typedef struct
{
    unsigned int ch;
    unsigned short val,count;
    unsigned short max;
    float amp,set_amp;
    unsigned short status;
#ifdef RT_THREAD
    rt_thread_t thread;
#endif
} hal_scr_out_t;


struct TABLE
{
    int MIN, MAX;
    int scale_in;
    float *val;
};

///@ CAN
#ifndef MCU_TINY
typedef struct
{
    const char *name;       //
    unsigned char ch;           //
    unsigned char pin_config;   //
} can_device_t;
typedef  const struct
{
    uint8_t   SJW;
    uint8_t   BS1;
    uint8_t   BS2;
    uint16_t  PreScale;
} tCAN_InitIterm;
extern const  tCAN_InitIterm  CAN_InitTab[];
int stm32_hw_can_init(const can_device_t *dev);

#endif
///@ end


///@ SHT11
typedef struct
{
    io_def sck;
    io_def sda;
} hw_sht11_t;

typedef struct
{
    hw_sht11_t *hw;
    uint16_t raw_temperature;
    uint8_t raw_temperature_crc8; /* read */
    uint8_t raw_temperature_crc8c; /* calculated */
    uint16_t raw_humidity;
    uint8_t raw_humidity_crc8;
    uint8_t raw_humidity_crc8c;
    uint8_t status_reg;
    uint8_t status_reg_crc8;
    uint8_t status_reg_crc8c;
    double temperature;
    double humidity_linear;
    double humidity_compensated;
    double dewpoint;
    uint8_t cmd; /* command to send */
    uint16_t result; /* result of the command */
    uint8_t crc8; /* crc8 returned */
    uint8_t crc8c; /* crc8 calculated */

} sht11_t;
extern const hw_sht11_t hw_sht11;
void sht11_init(sht11_t *sht11);
int16_t sht11_read_humidity(sht11_t *sht11);
int16_t sht11_read_temperature(sht11_t *sht11);

///@end


///@LED
int led_run_is_on(void);

///@end


/* @brief  */

/*************************************************

*************************************************/
typedef struct
{
    void *var;      //
    int varlen;     //
    int function;   //MODBUS
    int channel;    //MODBUS
    int last_update_time;   //
    int status;             //
} data_def;

typedef struct
{
    char *filename;     //
    char en;            //
    int times;          //
    int gap;            //
#ifdef WIN32
//      WORD devid;
#endif
} sound_def;
void sound(uint8_t times);



///!
typedef struct
{
    float scale;
    float zero;
} ai_calication_t;


//ADC
typedef struct
{
    void *adc;
    uint16_t channel;
} adc_def;

typedef struct
{
#ifdef STM32
    void * Tim;     //
#endif
    int outch;   //
    int remap;   //
    int remapen;
} pwm_in_def;

//!
typedef struct
{
#ifdef STM32

    void * TimA;        //
    int outchA;      //
    int remapA;      //
    int remapenA;
//      unsigned int YO_chA;    //,1,0

    void * TimB;        //
    int outchB;      //
    int remapB;      //
    int remapenB;
#endif
    unsigned int YO_chB;    //,0

    unsigned int YO_chC;    //,0

} pwm_out_def;


///@ ,GPIO
typedef struct
{
    io_def pin;     //
    io_def led;     //LED
    uint16_t    keyVal;     //(
} touch_key_t;

extern const int MAL_ADD_AI_CALICATION; ///
extern const int rtgui_touch_swap;
extern const io_def hw_beep_pin;


extern const io_def Xi_port[];
extern const io_def Yo_port[];
extern const adc_def ai_channel[];
extern const pwm_out_def pwm_out_port[];
extern const pwm_in_def pwm_in_port[];
extern const spi_flash_def spi_flash;
extern const io_def lcd_pwr;
extern void (*lcd_brightness)(unsigned char b);

#define DFU_SIZ_DEVICE_DESC            18

#define DFU_SIZ_CONFIG_DESC          47

#define DFU_SIZ_STRING_LANGID           4
#define DFU_SIZ_STRING_VENDOR           38
#define DFU_SIZ_STRING_PRODUCT          20
#define DFU_SIZ_STRING_SERIAL           26
#define DFU_SIZ_STRING_INTERFACE0       96    /* Flash Bank 0 */

#define DFU_SIZ_STRING_INTERFACE1       98     /* SPI Flash : M25P64*/
#define DFU_SIZ_STRING_INTERFACE2       106    /* NOR Flash : M26M128*/

extern  const uint8_t DFU_DeviceDescriptor[DFU_SIZ_DEVICE_DESC];
extern  const uint8_t DFU_ConfigDescriptor[DFU_SIZ_CONFIG_DESC];
extern  const uint8_t DFU_StringLangId     [DFU_SIZ_STRING_LANGID];
extern  const uint8_t DFU_StringVendor     [DFU_SIZ_STRING_VENDOR];
extern  const uint8_t DFU_StringProduct    [DFU_SIZ_STRING_PRODUCT];
extern  uint8_t DFU_StringSerial     [DFU_SIZ_STRING_SERIAL];
extern  const uint8_t DFU_StringInterface0 [DFU_SIZ_STRING_INTERFACE0];
extern  const uint8_t DFU_StringInterface1 [DFU_SIZ_STRING_INTERFACE1];
extern  const uint8_t DFU_StringInterface2_1 [DFU_SIZ_STRING_INTERFACE2];
extern  const uint8_t DFU_StringInterface2_2 [DFU_SIZ_STRING_INTERFACE2];
extern  const uint8_t DFU_StringInterface2_3 [DFU_SIZ_STRING_INTERFACE2];
extern  const uint8_t DFU_StringInterface2_4 [DFU_SIZ_STRING_INTERFACE2];

#if 0  /* armfly */
#define BMAXPACKETSIZE0             0x100     /* bMaxPacketSize0 = 64 bytes   */
#else
#define BMAXPACKETSIZE0             0x40     /* bMaxPacketSize0 = 64 bytes   */
#endif

#define wTransferSize               0x0400   /* wTransferSize   = 1024 bytes  */
/* bMaxPacketSize0 <= wTransferSize <= 32kbytes */
#define wTransferSizeB0             0x00
#define wTransferSizeB1             0x04

struct uart_hw_def
{
    void* uart_device;  //USART_TypeDef  LPC_UART_TypeDef   uartport
    int irq;    //IRQn_Type
    io_def tx_en;   //
    io_def rx_en;   //
    uint8_t remap;
};


#ifdef RT_THREAD
#if defined(S3C2440)
typedef struct uartport
{
    volatile rt_uint32_t ulcon;
    volatile rt_uint32_t ucon;
    volatile rt_uint32_t ufcon;
    volatile rt_uint32_t umcon;
    volatile rt_uint32_t ustat;
    volatile rt_uint32_t urxb;
    volatile rt_uint32_t ufstat;
    volatile rt_uint32_t umstat;
    volatile rt_uint32_t utxh;
    volatile rt_uint32_t urxh;
    volatile rt_uint32_t ubrd;

} uartport;


#elif defined(S3C2416)
typedef struct uartport
{
    volatile rt_uint32_t ULCON;                 // line control reg
    volatile rt_uint32_t UCON;                  // control reg
    volatile rt_uint32_t UFCON;                 // FIFO control reg
    volatile rt_uint32_t UMCON;                 // modem control reg
    volatile rt_uint32_t UTRSTAT;               // tx/rx status reg
    volatile rt_uint32_t UERSTAT;               // rx error status reg
    volatile rt_uint32_t UFSTAT;                    // FIFO status reg
    volatile rt_uint32_t UMSTAT;                    // modem status reg
    volatile rt_uint32_t UTXH;                  // tx buffer reg
    volatile rt_uint32_t URXH;                  // rx buffer reg
    volatile rt_uint32_t UBRDIV;                    // baud rate divisor
    volatile rt_uint32_t UDIVSLOT;              // baud rate divisor
} uartport;


#endif

#endif

#ifdef __cplusplus
extern "C"
{
#endif

///!
int MAL_Read(unsigned char *dest, unsigned int SectorAddress, unsigned int DataLength);
int MAL_Write(unsigned int SectorAddress,unsigned char *buf, unsigned int DataLength);

uint8_t calc_xor(uint8_t *buf, uint8_t len);
uint8_t calc_checsum(uint8_t *buf, uint8_t len);
int CRC16_CCITT(uint8_t *DataBuf, uint8_t BufLen);
unsigned short crc16(unsigned char *buffer, unsigned short buffer_length);


int host_init(void);
void board_init(void);

///@ LED
int led_run_init(void);
int led_run(int v);
int led_msg_init(void);
int led_msg(int v);
void LEDRUNOff(void);
void LEDRUNOn(void);
void LEDMSGOn(void);
void LEDMSGOff(void);
int led_msg_is_on(void);

///@end



int backlight_init(void);
int backlight(int v);
int hw_beep_init(void);
void beep(uint8_t v);


void hal_di_init(hal_di_t *, int);
unsigned char hal_di_get_value(hal_di_t *p);
void hal_di_cleanup(hal_di_t *p, int ch);


///!@ AI
#define NUM_FFT 64  //fft N
#define FFT_N       6   //282562N12N
typedef struct
{

    //T
    //unsigned char ADC_Count = 0;
    /********unsigned char1unsigned char16 Fft_Real[128]6signedsigned int
     * Fft_RealFft_Image*********/

    // 56fftS66

    //fft

    //unsigned short TEMP1; //
    short Fft_Real[NUM_FFT]; //fft28
    short Fft_Image[NUM_FFT]; //fft28
    unsigned char FFTSwitch ;
    unsigned char CalculateSW ;
    float Vrms;
    unsigned short Count ;
    unsigned short Vpeak[30];
    unsigned int index;
} FFT_t;
#ifdef RT_THREAD
void fft_in(FFT_t *ft, unsigned short val, unsigned short vref);
#endif

#define HAL_AD_FLAG_FILTER_PASS_EN 0x1
#define HAL_AD_FLAG_FILTER_AVR_EN 0x2
#define HAL_AD_FLAG_FILTER_MID_EN 0x4
typedef struct
{
    unsigned int ch;
    int flag;

    unsigned int pool_sum;
    float range_max, range_min;

    //pass_filter
    unsigned int last_value;
    float pass;

    //
    struct filter_avr_t filter_avr;
    struct filter_middle_t filter_mid;
    filter_pass_t fil_pass;
    unsigned short *pool, pool_nr, pool_index, value_raw; //
    unsigned short bufSize;     //
    unsigned short value, value_max, value_min;
    unsigned char rank;
    u32 speed;	//! cps times per second
#ifdef RT_THREAD
    FFT_t *fft;
#endif

} hal_ai_t;
int ai_prepare(int chnum, int buftimes);
int ai_init_end();
void hal_ai_set_type(hal_ai_t *p,unsigned int v);
unsigned short hw_ai_get_UINT(hal_ai_t *p);
unsigned short hw_ai_get_UINT_max(hal_ai_t *p);

int hal_ai_get_val(hal_ai_t *p);
void hal_ai_set_filter(hal_ai_t *p,const float *v);
///!@end

///!
void hw_gpio_out_init(const io_def);
void hw_gpio_out_od_init(const io_def  port);
void hw_gpio_out(const io_def,unsigned char);


///!
void hw_gpio_in_init(const io_def,uint8_t);
void hw_gpio_ai_init(const io_def port);
void hw_gpio_SFN2_init(const io_def port);
void hw_gpio_in_float_init(const io_def port);

/** defgroup exti externel interrupt
* @{
**/
typedef enum
{
    HW_EXTI_Trigger_Rising = 0x08,
    HW_EXTI_Trigger_Falling = 0x0C,
    HW_EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_Type;

void hw_gpio_exti_init(const io_def port, EXTITrigger_Type EXTI_Trigger);
void set_port_irq_stop(const io_def port);
/* irq install
notice: only one irq per pin(A,B...)
*/
int hal_irq_init(const io_def port,  EXTITrigger_Type EXTI_Trigger,int (*handle)(void *),void *arg);
extern int (*EXTI_handles[16])(void *);
extern int *EXTI_arg[16];

//* @}

uint8_t hw_gpio_in(const io_def);




uint16_t MAL_Init(void);

///!SPI
typedef struct
{
    const io_def cs;
    const io_def clk;
    const io_def mosi;
    const io_def miso;
    uint8_t cpol,cpha;//CPOLSCKCPOL0CPOL1CPHACPHA=0CPHA1
    uint8_t delay;  //()
} spi_io_def;

void hw_spi_cs(spi_io_def *,uint8_t);
uint8_t hw_spi_rw(spi_io_def*,uint8_t byte);
void hw_spi_init(spi_io_def *hwspi_io_def);

//io sim
void hw_spi_sim_init(const spi_io_def *dat);
unsigned char hw_spi_rw_sim(const spi_io_def *dat,unsigned char data);
int spi_io_read_word(const spi_io_def *dat);
///end


///@touch
typedef struct
{
    spi_io_def spi;
    const io_def busy;
    const io_def irq;
} touch_chip_t;
extern const touch_chip_t touch;
///end

typedef struct
{
    spi_io_def spi;
    const io_def cs;
    const io_def busy;
    const io_def miso;
    const io_def mosi;
    const io_def clk;
} ads7844_def;

#ifdef  STM8S003
void @far @interrupt HX711_process(void);
#else
void HX711_process(hx711_chip *cp);
#endif




void delay_us(uint8_t us);
void delay_ms(volatile unsigned int ms);


void rt_hw_lcd_init_ssd1963();
void rt_hw_lcd_init_ili9327();
void rt_hw_lcd_init_r61509();
void rt_hw_lcd_init_hx8352();
void rt_hw_lcd_init_ili9341();
void rt_hw_lcd_init_2440(void);

void rt_hw_touch_init(void);

unsigned short get_Year();
unsigned short get_Month();
unsigned short get_Day();
unsigned short get_Week();
unsigned short get_Hour();
unsigned short get_Minute();
unsigned short get_Second();
int SYS_YEAR(char *buf);
int SYS_MONTH(char *buf);
int SYS_DAY(char *buf);
int SYS_WEEK(char *buf);
int SYS_HOUR(char *buf);
int SYS_MINUTE(char *buf);
int SYS_SECOND(char *buf);
int set_Year(unsigned short tt);
int set_Month(unsigned short tt);
int set_Day(unsigned short tt);
int set_Hour(unsigned short tt);
int set_Minute(unsigned short tt);
int set_Second(unsigned short tt);


#ifdef RT_THREAD
void rs485_to_tx(void *dev);
void rs485_to_rx(void *dev);
void rs422_to_tx(void *dev);
void rs422_to_rx(void *dev);
void set_serial_config(void *dev,unsigned int baud,unsigned short dataBits,unsigned short stopBits,char parity);//struct rt_serial_device

void KEYUP(uint16_t key);       //,
void KEYDOWN(uint16_t key); //,
void next_win();
void touch_key_down(touch_key_t key);
void touch_key_up(touch_key_t key);


#endif
//@clock
void sel_hsi(void);
void sel_lsi(void);
u32 sysclk();
//@end

//@clock tick
void timer_init(void);
extern uint32_t sys_ticks;
//@end

//@sleep
void Active_halt(uint16_t second);   //
void RTC_Config(void);
//@end

//!zigbee
void SendTheMessage(unsigned char *theMessageData,uint8_t len);

///@test
void test_delay_us(io_def pin);
void test_delay_ms(io_def pin);

///@end


/// @segment AO
typedef struct
{
    unsigned int ch;
    unsigned port;
    unsigned pin;
    unsigned short val;
    float max,min;
#ifndef STM32
    hal_po_t po;
#endif
} hal_ao_t;

int hw_ao_init(hal_ao_t *);
int hw_ao_set_UINT(hal_ao_t *p);
int hw_ao_max(hal_ao_t *p);
/// @end


///!@ MCP4822
typedef struct
{
    io_def cs;
    io_def clk;
    io_def sdi;
    io_def ldac;
    uint8_t cpol;
    uint16_t delay;
	unsigned gain;	//0=2*VREF,1=VREF
} hw_mcp4822_t;
extern const hw_mcp4822_t hw_mcp4822[];
typedef struct
{
    uint8_t ch;
    uint16_t value;
    uint16_t cmd;
    float max,min;
    const hw_mcp4822_t *hw;
} hal_mcp4822_t;
void hal_mcp4822_init(hal_mcp4822_t  *,uint8_t ch);
void hal_mcp4822_set_out(hal_mcp4822_t  *,uint16_t val);
void hal_mcp4822_set_c_out(hal_mcp4822_t  *dat,float* val);
void hal_mcp4822_set_max(hal_mcp4822_t  *dat,float* val);
void hal_mcp4822_set_min(hal_mcp4822_t  *dat,float* val);
void hal_mcp4822_cleanup(hal_mcp4822_t  *,uint8_t ch);
///!@end


///!@ hal_sys
//
#ifndef MCU_TINY

//  #include<time.h>


typedef struct
{
    uint32_t time;
    int sys_log_type;
    void *pc;
} sys_log_t;

#define LOG_NRS 16

struct calibration_data
{
    uint16_t min_x, max_x;
    uint16_t min_y, max_y;
};


///@ sys
enum
{
    SYS_LOG_CRASH = 1,
    SYS_LOG_HARD_RESET,
    SYS_LOG_WATCHDOG_RESET,
    SYS_LOG_SLEEP_RESET,
    SYS_LOG_DEEP_STOP_RESET,
    SYS_LOG_SOFT_RESET,
    SYS_LOG_PWR_FAIL,
    SYS_LOG_MODBUS_C
};



typedef struct
{
    uint8_t prog_name[8];   //use to erp lookfor
    unsigned int ver_boot;          //BOOT
    uint32_t ver_app;
    unsigned int dhcp;
    unsigned char localip[4];
    unsigned char gateway[4];
    unsigned char netmask[4];
    unsigned int progLength;        //RAM
    unsigned int prog_chksum;       //
    unsigned int iap_flag;          //x5a5aiap
    unsigned char enRemoteIAP;    // 12
    unsigned int prog_ok;           //0x5a5a
    unsigned int log_p;         //
    unsigned int max_time;      //!
    unsigned int total_time;    //!
    unsigned short min_x, max_x;
    unsigned short min_y, max_y;
    int site_id;        //()
    //    float ai_adj[12];
    //      float pt100[12];    //pt100
    sys_log_t log[LOG_NRS];         //!
    unsigned short retain_interval;

    unsigned char updatefromflash;  //flashflash
    unsigned int pstart, plength, pchecksum; //
    unsigned int time_sleep, time_supsend;  //
    unsigned int normal_brightness, supend_brightness;  //LCD
} hal_sys_t;
#ifdef BOOTLOADER
extern hal_sys_t  sys_log_in_boot;
#else
extern hal_sys_t *sysValue;
#endif


void hal_sys_init(hal_sys_t *t,int ch);
void hal_sys_set_site(hal_sys_t *t,unsigned short val);
void hal_sys_set_period(hal_sys_t *t,unsigned int v);
int hal_sys_get_mem(hal_sys_t *dat,int ch);
void hal_sys_set_beep(hal_sys_t *dat,int v);
unsigned short hal_sys_get_year(hal_sys_t *t);
unsigned short hal_sys_get_mon(hal_sys_t *t);
unsigned short hal_sys_get_day(hal_sys_t *t);
unsigned short hal_sys_get_hour(hal_sys_t *t);
unsigned short hal_sys_get_min(hal_sys_t *t);
unsigned short hal_sys_get_sec(hal_sys_t *t);
void hal_sys_set_year(hal_sys_t *t,unsigned short v);
void hal_sys_set_mon(hal_sys_t *t,unsigned short v);
void hal_sys_set_day(hal_sys_t *t,unsigned short v);
void hal_sys_set_hour(hal_sys_t *t,unsigned short v);
void hal_sys_set_min(hal_sys_t *t,unsigned short v);
void hal_sys_set_sec(hal_sys_t *t,unsigned short v);



///@end

#endif
///@end

///!@ MODBUS
int cmp_little2large(char *dest,const char *source,unsigned short len);

///@end


///!@ SPI
unsigned char hw_spi_rw_sim_rt(const spi_io_def *dat,unsigned char data);
///@end


///!@ DHT11
int hal_dht11_init(hal_dht11_t *dat,int ch);
void hal_dht11_get_dht11_h(hal_dht11_t *dat,float *v);
void hal_dht11_get_dht11_t(hal_dht11_t *dat,float *v);
void hal_dht11_cleanup(hal_dht11_t *dat,int ch);
void hal_dht11_set_SEL(hal_dht11_t *dat,char v);

///@end

///!@ Device Module Base
typedef struct
{
    void (*init)(void *);
    void (*reterive)(void *);
    void (*publish)(void *);
    void (*cleanup)(void *);
} device_module;

///@end


///!@ RTC DEVICE
void rt_hw_rtc_init(void);

void rt_hw_rtc_lse_init(void);
void rt_hw_rtc_hse_init(void);
typedef struct
{
    io_def scl;
    io_def sda;
} hw_pcf8563_t;
extern hw_pcf8563_t hw_pcf8563;
void rt_hw_pcf8563_init(void);
///!@ end

///!@ IIC DEVICE
void rt_hw_iic_init(void);
extern void (*i2c1_ev_irq)(void *);
extern void *i2c1_ev_irq_dat;
extern void (*i2c1_er_irq)(void *);
extern void *i2c1_er_irq_dat;
///!@ end

///!@ SDCARD
void sdcard_init(void);
///!@ end

///!@ net
typedef struct
{
    const char *name;       //8
    const io_def eint;      //
#ifdef CORTEX_M3
    const volatile rt_uint16_t *port_io;
    const volatile rt_uint16_t *port_data;
#endif
} hw_lan_def;
extern const hw_lan_def hw_lan_config;
void stm32f107_eth_init();
///!@end



///@brief: serial port class
/**
@startuml
scale 350 width
[*] --> :
[*] --> :
 -->
 -->
 -->
}
@enduml
**/
#undef PARITY_NONE
#undef PARITY_EVEN
#undef PARITY_ODD
enum uart_parity
{
    PARITY_NONE = 0, //!< PARITY mode is PARITY_NONE.
    PARITY_EVEN = 2, //!< PARITY mode is PARITY_EVEN.
    PARITY_ODD  = 3  //!< PARITY mode is PARITY_ODD.
};

enum
{
    COMM_STATUS_SERIAL_OPEN,    //
    COMM_STATUS_OK,
    COMM_STATUS_TIMEOUT,
    COMM_STATUS_CRC_ERR,
    COMM_STATUS_UNKNOWN_ERROR,
    STATUS_CASH_FULL,
    COMM_STATUS_ILL_CMD,
    COMM_STATUS_WEIGHT_IS_ZERO,
    COMM_SATUS_SET_ZERO,
    COMM_STATUS_IDLE,
    COMM_STATUS_RECEIVE
};
enum
{
    COMM_FLAG_DUPLTEX,      //双工模式,备用
    COMM_FLAG_DECODE_ALL   //统一接收,所有接收协议统一处理
};

///@brief :
#define COMM_STATUS_SET(sta,x) sta|=1<<x
///@brief :
#define COMM_STATUS_CLEAR(sta,x)    sta&=~(1<<x)
///@brief :
#define COMM_STATUS(sta,x) (sta&(1<<x))

typedef struct
{
    u8 *buf;
    u8 *rinx;
    u8 *winx;
    u16 size;
} buf_data;
buf_data *buf_data_init(buf_data *dat,u8 *buf,u16 size);
int buf_data_len(buf_data *dat);
int buf_data_put(buf_data *dat,const u8 *buf,u8 len);
void buf_data_clear(buf_data *dat);
int buf_data_read(buf_data *dat,u8 *dest,u16 len);
int buf_data_put_char(buf_data *dat,u8 buf);


typedef struct _comm_port comm_port_t;
struct _comm_port
{
    int (*init)(comm_port_t *dat,const void *hw);
    int (*open)(comm_port_t *,u32 baud,u8 dataBits,u8 stopBits,enum uart_parity);
    int (*write)(comm_port_t *dat,uint8_t* buf,u8 len,void *other);
    int (*read)(comm_port_t *,uint8_t* buf,u8 len);
    int (*select)(comm_port_t *,u8 maxlen,const u32 *timeout);
    int (*rxcb)(comm_port_t *);
    int (*poll)(comm_port_t *);
    int (*close)(comm_port_t *);
    void *data;     //
    uint32_t flag;
    u32 interval;
    u32 status;
    u32 tick;	//内部计时器
    buf_data tx_buf,rx_buf;
};
comm_port_t *comm_port_win(const char *);
comm_port_t *comm_port_qt(const char *);
comm_port_t *comm_port_rt(const char *);
comm_port_t *comm_port_sim(int index);
extern comm_port_t uart_stm8;
#ifdef USE_HAL_DRIVER
comm_port_t *comm_port_stm32_hal_init(void *dat,void *uart);
typedef struct
{
    comm_port_t base;
#ifdef STM32F103xE
    UART_HandleTypeDef *huart;
    TimerHandle_t tmr;
#if configSUPPORT_STATIC_ALLOCATION
    osStaticTimerDef_t tmrBlk;
#endif
#endif
    u8 buf;

} comm_port_hal_t;
#endif
///!@end serial



///!@ IAP
#ifndef MCU_TINY
#define IAP_ADDRESS 0
typedef enum
{
    IAP_INIT,
    IAP_WAIT,
    IAP_ERROR,
    IAP_OK,
} iap_status;

typedef struct
{
    uint16_t blklen;
    uint16_t cindex;
    uint16_t sec_Size;
    uint16_t index_nrs;
    uint32_t prog_address;
    iap_status status;
    uint8_t *buf;
    comm_port_t *port;
} iap_t;
#endif
extern uint16_t boot_Ver;
extern uint16_t boot_build;
int gotoiap();

///!@ end IAP


///!@ mqtt client
typedef struct
{
    int (*init)(void *base);
    int (*send)(void *mq,u8 *topic,u8 *payload,u16 len);    //
    int (*connectcb)();
    int (*disconnectcb)();
    int (*PublishedCb)();
    int (*DataCb)();
    char *host;
    void *client;
    uint32_t ipaddr;
    uint16_t port;
    char *clientid;
    uint16_t tid;   //tx id
    u32 tx_count,tx_fail_count;
    s8 tx_ret;
    u32 conn_status;
} mqtt_t;
extern mqtt_t mqtt_client;

///!@end mqtt client

///!@ spi flash device
typedef struct
{
    int (*init)(void *);
    int (*read)(void *,uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
    int (*write)(void *,uint32_t SectorAddress,unsigned char *buf, uint32_t DataLength);
    int (*erase)(void *,uint32_t SectorAddr);
} mem_dev;
extern mem_dev *sf_dev;
extern mem_dev mal;
extern mem_dev dev_spi_flash;
void hw_spi_flash_init();
typedef struct
{
    mem_dev base;
    const char *filename;
    unsigned append;    //
} file_dev;
mem_dev * file_dev_init(const char *filename,unsigned append);
///!@end

///!@ History Module
typedef struct
{
    uint32_t index;
    uint32_t timestamp;
    uint16_t crc;
    uint16_t status;
} history_data;

typedef struct
{
    uint32_t address;   //start save address
    uint32_t nrs;       // max record num
    uint32_t data_len;  //userdata length
    uint32_t current_index; //
    uint32_t total;     //all record num
    uint32_t crc;
} history_ctrl;     //selection save to flash

typedef struct
{
    history_ctrl base;
    uint32_t address_ctrl;
    history_data *userdata; //userdata address
    uint32_t cmd;       ///! commands ,user process with modbus
    mem_dev *mem;
} history_t;

extern void* __ADDRESS_HISTORY_CTRL;    //
extern void* __ADDRESS_HISTORY_DATA;
extern void* __ADDRESS_RETAIN;
extern int(*read_history)(void* ctx,uint16_t addr,uint8_t len,uint16_t * dst);
extern int (*write_history)(void* ctx,const uint16_t *dats,uint16_t addr,uint8_t len);
int cb_his_modbus_write(void* ctx,const uint16_t* data,uint16_t index,uint8_t len);
int cb_modbus(void* ctx,uint16_t index,uint8_t len,uint16_t* data);

//@function     history_init
//@param    mem :
//@param    index   :
//@param    data_address :
//@param    user_dat    :
//@param    len     :
//@param    start_address   :
//@param    nrs     :
//@param    force   :
int history_init(mem_dev *mem,uint8_t index,uint32_t data_address,void *user_dat,uint16_t len,uint16_t nrs,uint8_t force);

int history_save(uint8_t index);
///@end


///!@ debounce module
typedef struct
{
    //@param
    uint16_t    debounce_max;       /* max number of additional readings */
    float   debounce_tol;       /* tolerance used for filtering */
    uint16_t    debounce_rep;       /* additional consecutive good readings*/
    //@internal
    uint16_t read_cnt,read_rep;
    float val,last_read;
} hal_debounce_t;
int hal_debounce_init(hal_debounce_t *,int);
int hal_debounce_get_out(hal_debounce_t *,float * val);
int hal_debounce_set_in(hal_debounce_t *ts, float *val);
int hal_debounce_set_range(hal_debounce_t *ts, float * val);
int hal_debounce_set_rep(hal_debounce_t *ts, int val);
int hal_debounce_cleanup(hal_debounce_t *dat,int ch);
///!@ end

///!@ io encoder
typedef struct
{
    io_def A, B;
#ifdef STM32
    void *TIM;   //
    int channel_a,channel_b;
#endif
} encoder_port_def;
extern encoder_port_def encoder_port[];
///!@ end


///!@Module retain
#if defined(S3C2440)||defined(S3C2416)

#define PROG_NAND_ADDRESS 0x100000  //0x150000~0x04000000 ,
#define PROG_RUN_ADDRESS 0x30000000

#elif defined(STM32)
#define PROG_RUN_ADDRESS 0x08010000

#elif defined(LPC178X)
#define PROG_RUN_ADDRESS 0x08010000


#elif defined(WIN32)||defined(__ARMLINUX__)||defined(QT)
///@ memo
#define retainName "retain.dat"
#define PROG_RUN_ADDRESS 0x30000000
#endif
typedef struct
{
    unsigned short checksum;
    unsigned short prjversion;
} retain_t;
extern const unsigned int retain_address;
void retain_init();
void retain_output();

///!@
void __init_retain();     //
extern unsigned short prjversion;
extern mem_dev *retain_dev;
/// !@end


void modbus_close(void *ctx);

int modbus_set_slave(void* ctx, int slave);
int modbus_read_bits(void *ctx, int addr, int nb, uint8_t *dest);
int modbus_read_input_bits(void *ctx, int addr, int nb, uint8_t *dest);
int modbus_read_registers(void *ctx, int addr, int nb, uint16_t *dest);
int modbus_read_input_registers(void *ctx, int addr, int nb, uint16_t *dest);
int modbus_write_bit(void *ctx, int coil_addr, int status);
int modbus_write_register(void *ctx, int reg_addr, int value);
int modbus_write_bits(void *ctx, int addr, int nb, const uint8_t *data);
int modbus_write_registers(void *ctx, int addr, int nb, const uint16_t *data);
int modbus_write_and_read_registers(void *ctx, int write_addr, int write_nb,
                                    const uint16_t *src, int read_addr, int read_nb,
                                    uint16_t *dest);
void modbus_set_bits_from_byte(uint8_t *dest, int idx, const uint8_t value);
void modbus_set_bits_from_bytes(uint8_t *dest, int idx, unsigned int nb_bits,
                                const uint8_t *tab_byte);
uint8_t modbus_get_byte_from_bits(const uint8_t *src, int idx, unsigned int nb_bits);
void modbus_get_dword(const uint16_t *src,uint8_t *dest);
void modbus_get_dword_abcd(const uint16_t *src,uint8_t *dest);
void modbus_get_dword_dcba(const uint16_t *src,uint8_t *dest);
void modbus_get_dword_badc(const uint16_t *src,uint8_t *dest);
void modbus_get_dword_cdab(const uint16_t *src,uint8_t *dest);
void modbus_get_lword_abcd(const uint16_t *src,uint8_t *dest);
void modbus_get_lword_dcba(const uint16_t *src,uint8_t *dest);
void modbus_get_lword_badc(const uint16_t *src,uint8_t *dest);
void modbus_get_lword_cdab(const uint16_t *src,uint8_t *dest);

void modbus_get_word_ab(const uint16_t *src,uint8_t *dest);
void modbus_get_word_ba(const uint16_t *src,uint8_t *dest);
void modbus_set_word_ba(const uint8_t * f, uint16_t *dest);
void modbus_set_word_ab(const uint8_t * f, uint16_t *dest);

void modbus_set_dword(const uint8_t * f, uint16_t *dest);
void modbus_set_dword_abcd(const uint8_t * f, uint16_t *dest);
void modbus_set_dword_dcba(const uint8_t * f, uint16_t *dest);
void modbus_set_dword_badc(const uint8_t * f, uint16_t *dest);
void modbus_set_dword_cdab(const uint8_t * f, uint16_t *dest);
void modbus_set_lword_abcd(const uint8_t * f, uint16_t *dest);
void modbus_set_lword_dcba(const uint8_t * f, uint16_t *dest);
void modbus_set_lword_badc(const uint8_t * f, uint16_t *dest);
void modbus_set_lword_cdab(const uint8_t * f, uint16_t *dest);

//! addgroup plc
//!@{
struct plc_device
{
    void (*init)();
    void (*start)(void);

    void (*retrieve)();

    void (*publish)();

    void (*cleanup)();
    const char *name;
    char rev_vaild, out_en, inout;
};

typedef struct plc_device *plc_device_t;
#define PLC_DEVICE(a) ((struct plc_device *)a)
#define CONTAINER_OF(obj, type, member)   \
    ((type *)((char *)(obj) - (unsigned long)(&((type *)0)->member)))

extern unsigned int plc_device_nr;

#define PLC_FLAG_RUNING (1<<0)
#define PLC_FLAG_RUN    (1<<1)
#define PLC_FLAG_DEBUG  (1<<2)
typedef struct
{
    struct plc_device **devs;
    int flag;
#ifdef RT_THREAD
    rt_tick_t PLC_tick;
    rt_thread_t thread;
#endif
} plc_t;
extern u16 PLC_TIME;
int plc_init();
void custom_init();
void custom_reterive();
void custom_publish();



void config_run__(unsigned long tick);
void config_init__(void);
void __init_hmi(void);
void __cleanup_hmi(void);
void __retrieve_hmi(void);
void __publish_hmi(void);

void __init_io(void);
void __cleanup_io(void);
void __retrieve_io(void);
void __publish_io(void);
int plc_start(plc_t *plc);
//!@}

// @defgroup mosule_second
// @addtogroup mosule_second
// @{
typedef struct
{
    void (*second)(void *);
    void *arg;
#ifndef USE_HAL_DRIVER
#ifdef RT_THREAD
    rt_list_t *node;
#endif
#endif
} second_cb_t;
// @}

void assert_failed(uint8_t *file, uint32_t  line);


void filter_bubbleSort_int(int arr[], unsigned short count);
#ifndef MCU_TINY
enum CARDTYPE
{
    CARD_TYPE_CIKA,       //
    CARD_TYPE_S50_S70,    // Mefare one S50/S70
    CARD_TYPE_UL,         // Mefare one UL( Ultra Light)
    CARD_TYPE_ISO14443,   //  ISO14443 CPU
    CARD_TYPE_24XX,       //24CXX
    CARD_TYPE_CPU,        // CPU
    CARD_TYPE_SLE4442,    //SLE4442
    CARD_TYPE_SLE4428,    // SLE4428
    CARD_TYPE_AT88SC102,  // AT88SC102
    CARD_TYPE_AT88S1604,  // AT88S1604
    CARD_TYPE_AT45D041,   // AT45D041
    CARD_TYPE_SIM,        //SIM
    CARD_TYPE_AT88SC1608, //AT88SC1608
    CARD_TYPE_UNKNOW
};

//
enum
{
    FLAG_CARD_CONNET,
    FLAG_CARD_INSERT,
    FLAG_CARD_REJECT,
    FLAG_CARD_SAVE,
    FLAG_CARD_FINISH,
    FLAG_CARD_UNCONNET,
    FLAG_CARD_EMPTY,
    FLAG_CARD_PASS_OK,
    FLAG_CARD_ERROR
};

typedef struct
{
    int cmd;
    int card_type;
    int status;
    int operation;
#ifdef RT_THREAD
    rt_device_t dev;
#endif
    int lock;
    int cardin;
    int errcode;

    int password;
    int passok;
    unsigned char dat_buf[256];
    unsigned char buf[32];
} hal_card_t;

typedef struct
{
#ifdef RT_THREAD
    rt_device_t dev;
    rt_size_t tx_len;
    rt_size_t rx_len;
    rt_size_t dat_len;
#endif
    unsigned char tx_buf[32];
    unsigned char rx_buf[32];
    unsigned char *dat;
} jcp05_t;

typedef struct
{
    unsigned int cmd;
    int errcode;
    unsigned int passok;
    unsigned long long password;
    unsigned int card_type;
    unsigned char sn[13];
    unsigned char card_in;
    unsigned char status;
    unsigned char block;
    unsigned char dat_buf[16], wbuf[16];
    unsigned int money;
    jcp05_t jcp05;
} hal_mifare_t;

typedef struct
{
    uint8_t site;
#ifdef RT_THREAD
    rt_device_t dev;
#elif defined(QT)
    QSerialPort *dev;
#endif
    uint32_t status;
    uint32_t count, errorcnt;
    uint8_t buf[64], *inx, st0, st1, st2, e1, e0;
    uint8_t sn[18]; //
    uint16_t cardType;
    uint8_t cardSn[10];     //
    void (*cardOut)(void);  //,->
    void (*cardIn)(void);   //,->
    void (*cardHold)(void); //,*->
} hal_crt571_t;

void card_init(hal_card_t *card_dat_t, const char *devName);
int hal_card_get_in(hal_card_t *cd);
void hal_card_get_real(hal_card_t *cd, float *val);
int hal_card_get_type(hal_card_t *cd);
void hal_card_set_real(hal_card_t *cd, float *val);


#endif

typedef struct
{
#ifdef RT_THREAD
    rt_device_t dev;
#elif defined(QT)
    QSerialPort *dev;
#endif
    uint32_t status;
    char buf[64], *inx;
    uint32_t count, errorcnt;
    uint8_t enbilltype;
    uint8_t err, step;
    uint32_t amount;
    void (*amountChanged)(void);
} hal_rmb_b11_t;

typedef struct
{
    io_def a, b, c, d, e, f, g, h, dp;
} leda_def;

extern const io_def YOUT[];
extern const leda_def hw_led;
extern const io_def key_led[];
extern const io_def led_seg[];
extern const io_def SDA, SCL;

///!PT100
int16_t pt100_get_temp_int(uint16_t r_mul_1000);
float pt100_get_temp_float(float *data, float *result);
void hal_hx711_init(hal_hx711_t *dat, int ch);
void hal_hx711_get_rbri(hal_hx711_t *p, float *weight); //增28
void hal_hx711_get_val(hal_hx711_t *p, float *weight);  //增28
void hal_hx711_cleanup(hal_hx711_t *dat, int ch);
void calc_crc(unsigned char byte);

typedef struct
{
    float B;     //B
    float R, t1; //标准阻
    float r_up;  //阻阻
} hw_ntc_t;

#ifdef CC2530
extern hw_ntc_t hw_ntc;
#else
extern const hw_ntc_t hw_ntc;
#endif

typedef struct
{
    const hw_ntc_t *hw;
    float r; //实际阻测量
    float t; //的温度结
    hal_ai_t ai[2];
} hal_ntc_t;

///@ LOW POWER NTC
typedef struct
{
    hw_ntc_t ntc;
    io_def pwr_p;
    io_def pwr_n;
} hw_ntc_lp_t;
#ifdef CC2530
extern hw_ntc_lp_t hw_ntc_lp;
#else
extern const hw_ntc_lp_t hw_ntc_lp;
#endif
int hal_ntc_lp_init(hal_ntc_t *);
int hal_ntc_calc_t(hal_ntc_t *dat, uint16_t vr, uint16_t vt); //
void hal_ntc_open(hal_ntc_t *dat, char val);
///@end

typedef struct
{
    unsigned short cmd;
} commBlock;

struct ns
{
    char name[6];
    char *buf;
    unsigned int len;
    struct ns *next;
};

typedef struct
{
    unsigned int flag;
    float input_mv;   //
    float tare_mv;    //
    float range;      //
    float mvv;        //
    float resolution; //
    float out;        //
    float dest;       //
    float zero_range; //0
    float little;     //
} hal_weight_t;

#define WEIGHT_FLAG_POR_SINGLE (0x1 << 0) //
#define WEIGHT_FLAG_AUTO_TRACK (0x1 << 1) //
#define WEIGHT_FLAG_IS_CALI (0x1 << 2)    //

typedef struct
{
//      float val[16];
#ifdef RT_THREAD
    rt_device_t dev;
#endif
    struct ns *buf;
    unsigned int nr; //
} nmea0183_t;

struct dat_txt_file
{
    int (*filename)(const char *, char *);
    int (*filehead)(char *);
    int (*filedata)(char *);
    uint8_t *enVar; //
};

//P
typedef struct
{
    unsigned int ram_size, ram_remain; //ram
} hal_data_t;

extern hal_data_t sysData;

#define RECIPE_ADDRESS 0x10000


#define RECIPE_CMD_GET 1  //
#define RECIPE_CMD_PUT 2  //
#define RECIPE_CMD_ORIG 3 //
typedef struct
{
#ifdef RT_THREAD
    struct plc_device parent;
    rt_mailbox_t ack; //
    rt_list_t list;
#endif
    unsigned short amount;    //
    unsigned short current;   //
    unsigned short current_m; //
    unsigned short size;      //
    unsigned int mem;         //
    char *store;              //
    char *orig;               //
} recipe_t;

#define HAL_HB101_FLAG_ACTIVE 0x1
#define HAL_HB101_FLAG_LINKOK 0x2
#define HAL_HB101_FLAG_GPRS_OK 0x3

typedef struct
{
    comm_port_t *dev;
    uint32_t site; //
    uint32_t flag;
    uint32_t status;      //0=空闲，1：应答，2：遥信变位，3：遥测变位，4：总召上传
    uint16_t link_status; //,0:,1:,2:
    uint8_t addr_len;
    uint16_t interval;
    uint8_t frame;
#ifdef RT_THREAD
    rt_tick_t act_tick;
    rt_thread_t thread;
#endif
    uint32_t act_interval, act_interval_inv;

    uint8_t *(*fill_remote_pos_event)(uint8_t *);   //
    uint8_t *(*fill_all_cb)(uint8_t *);             //
    uint8_t *(*fill_fix_value)(uint8_t *, uint8_t); //
    void (*modify_fix_value)(uint16_t addr, int16_t val);
    int (*get_fix_value)(uint16_t addr);
    char *(*init_cmd)(int index);
    uint32_t gprs_init_interval, gprs_init_cmd;
    //
    uint8_t buf[256];
    uint8_t *rx_index;

    uint8_t txBuf[256];
    uint8_t *tx_index;
} hal_banlance101_t;

typedef enum
{
    PROTOCOL_ELEMENT_HEAD, //,,
    PROTOCOL_ELEMENT_LEN,
    PROTOCOL_ELEMENT_FIXED, //
    PROTOCOL_ELEMENT_DATA,  //,
    PROTOCOL_ELEMENT_CHECK  //
} element_type;

typedef struct _protocol protocol_t;
typedef struct _protocol_dev protocol_dev_t;
typedef struct _protocol_element protocol_element_t;
struct _protocol_dev
{
    u32 send_flag;
    int error;
    u8 next,query_nrs,write_nrs;
    comm_port_t *serial;
    int (*encode)(protocol_dev_t *dev,protocol_t *prl,protocol_element_t **,u8 elen);
    int (*decode)(protocol_dev_t *dev,protocol_t *prl,protocol_element_t **,u8 elen);
    int (*receive)(protocol_dev_t *dev,protocol_t *);     //协议接收处理
    int (*test_cb)(protocol_dev_t *dev,protocol_t *prl); //测试用函数,协议发送前调用
    void (*dataChanged)(int prl,int param,int val);
    protocol_t *rx;     //接收的协议,作为receive的参数
    protocol_t **write_array;
    protocol_t **query_array;

    //! @brief 发送方式
    //! 队列发送query_array
    //! 独立发送,需要队列发送空闲,排队
    //! MODE1:write_array,填写协议数据后设置send_flag
    //! MODE2:设置sender指向要发送的协议
    //! MODE3:填充协议数据,指向tx
    protocol_t *sender;     //待发送协议指针 ,提交到发送队列待发送
    buf_data *tx;   //待发送的数据缓冲区 , 先编码,缓冲区提交队列
    protocol_t *lastsend;       //最后发送的协议,供协议接收参用
} ;
struct _protocol_element
{
    uint8_t len;
    int (*encode)(protocol_dev_t *dev,protocol_t *,protocol_element_t *);
    int (*decode)(protocol_dev_t *dev,protocol_t *,protocol_element_t *,u8 *buf);
    int (*check)(protocol_dev_t *dev,protocol_t *,protocol_element_t *);
    const uint8_t *context;
} ;
typedef struct
{
    protocol_element_t base;
    u8 start, end;
} protocol_len_t;

typedef struct
{
    protocol_element_t base;
    uint8_t start_byte;
} protocol_var_t;
typedef struct
{
    protocol_element_t base;
    uint8_t start_byte;
} protocol_chksum_t;
struct _protocol
{
    u16 cmd;
    uint8_t nrs;          //协议元素数量
    protocol_element_t **elts; //    协议元素数组
    protocol_t *rsp;    // 应答协议
    u8 data_len;
    void *context;
} ;


int fill_byte(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int get_byte(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt,u8 *buf);
int fill_byte_parent(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int checksum_8(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int read_cmp_checksum_8(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int checksum_8_0xff(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int read_cmp_checksum_8_0xff(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);

int protocol_decode(protocol_dev_t *dev,protocol_t *prl,protocol_element_t **,u8 elen);
int protocol_encode(protocol_dev_t *dev,protocol_t *prl,protocol_element_t **,u8 elen);
int protocol_poll(protocol_dev_t *dev);
int protocol_receive(protocol_dev_t *dev,protocol_t *prl);
int protocol_send(protocol_dev_t *dev,protocol_t *prl);         //直接发送一条协议
int protocol_send_set(protocol_dev_t *dev,protocol_t *prl);     //!设置协议的发送标志,由队列发送
protocol_dev_t *protocol_dev_init(protocol_dev_t *dev);


unsigned short modbus_swap(unsigned short val);

void recipe_init(recipe_t *dat, unsigned short amount, unsigned short size, unsigned int mem);
void recipe_start(recipe_t *dat);

#ifdef RT_THREAD
void rtgui_sleep_init(); //
rt_err_t rtgui_wakeup();
#endif
typedef struct
{
    int en;
    int status, error;
    int value;
    int operation;

#ifdef RT_THREAD
    rt_device_t dev;
#endif
    unsigned int respose_time; //us
} hal_rmb_t;

#define HAL_JINNUO_CLEAR 1
#define HAL_JINNUO_ZERO 2
typedef struct
{
    comm_port_t *serial;
    unsigned char site;
    float weight, last, llast;
} hal_jinnuo_t;

typedef struct
{
    comm_port_t *serial;
    u8 address[6];
    uint16_t va, vb, vc;
    uint32_t ia, ib, ic;
    uint32_t pp, pa, pb, pc;
} hal_dlt645_t;
int dlt645_query(hal_dlt645_t *dat);
int dlt645_init(hal_dlt645_t *dat, comm_port_t *);

typedef struct
{
    unsigned char site;
    float weight, last, llast;
    uint32_t flag;
#ifdef RT_THREAD
    rt_device_t dev;
#endif
    uint32_t status;
} hal_xl_10000xx_t;

typedef struct
{
    protocol_dev_t base;
    unsigned char site;
    int baud;
    u8 buf[64];
} hal_mfc_t;

typedef struct
{
    uint16_t site;
#ifdef RT_THREAD
    rt_device_t dev;
#elif defined(QT)
    QSerialPort *dev;
#endif
    uint32_t count, errorcnt;
    uint32_t status;
    uint8_t buf[64];
} hal_host_link_t;

typedef struct
{
    comm_port_t *dev;
    uint32_t status;
    uint8_t buf[64], *buf_index;
    void (*scaned)(const char *);
} hal_xkd300_t;

typedef struct
{
    comm_port_t *dev;
    unsigned char id, to_set_id; //id1-99
    unsigned char filter;        //,0-9
    float value;                 //
    unsigned int advalue;        //AD
    unsigned int status;         //
} hal_HR900_t;


void hal_rmb_init(hal_rmb_t *rmb, int);
unsigned short hal_rmb_get_error(hal_rmb_t *rmb);
unsigned short hal_rmb_get_status(hal_rmb_t *rmb);
void hal_rmb_get_val(hal_rmb_t *rmb, float *r);
void hal_rmb_set_operation(hal_rmb_t *rmb, int v);

///!
void ai_calication(hal_ai_t *dat);


int hal_lsdi_init(hal_lsdi_t *, int);
unsigned char hal_lsdi_get_val(hal_lsdi_t *);

void hal_do_init(hal_do_t *, int);
void hal_do_set_value(hal_do_t *, unsigned char);
void hal_do_set_htime(hal_do_t *p, unsigned int d);
void hal_do_set_ltime(hal_do_t *p, unsigned int d);
void hal_do_cleanup(hal_do_t *p, int ch);

void hal_hcdo_init(hal_hcdo_t *, int);
int hal_hcdo_set_val(hal_hcdo_t *, unsigned char);

void ai_set_rcomp(hal_ai_t *p, const float *v);
void ai_set_scale(hal_ai_t *p, const float *v);
void ai_set_zero(hal_ai_t *p, const float *v);
void ai_set_pass_scale(hal_ai_t *p, const float *v);
void hal_ai_set_max(hal_ai_t *p, const float *v);
void hal_ai_set_min(hal_ai_t *p, const float *v);

int hal_ao_init(hal_ao_t *, int);
int hal_ao_set_out(hal_ao_t *p, unsigned short v);
int ao_write_voltage(hal_ao_t *p, double v);
//!*比例输max对 高输出电10V);
int ao_write_scale(hal_ao_t *p, double v, double max);

void ntc_r_to_t(hal_ntc_t *dat);
void ntc_r_to_t2(hal_ntc_t *dat);


#ifdef RT_THREAD
rt_err_t set_date(rt_uint32_t year, rt_uint32_t month, rt_uint32_t day);
rt_err_t set_time(rt_uint32_t hour, rt_uint32_t minute, rt_uint32_t second);
#endif
void bootVer(unsigned char *buf, unsigned int *len);
void reboot(int delay);
void stopall(const unsigned char *bufin, unsigned int len, unsigned char *bufout, unsigned int *retlen);

void hal_tm7710_init(hal_tm7710_t *dat, int ch);
void hal_tm7710_get_kcouple_value(hal_tm7710_t *p, float *weight);

void hal_scr_out_init(hal_scr_out_t *dat, int ch);
int hal_scr_out_get_status(hal_scr_out_t *dat);
void hal_scr_out_set_out(hal_scr_out_t *dat, float *v);
void hal_scr_out_pluses(hal_scr_out_t *dat, unsigned int v);
void hal_scr_out_set_max(hal_scr_out_t *dat, unsigned int v);
void hal_scr_out_get_amp(hal_scr_out_t *dat, float *v);
void hal_scr_out_set_sv_amp(hal_scr_out_t *dat, float *v);
void hal_ai_get_ntc(hal_ntc_t *dat, float *val);
void hal_ai_set_ntc_r(hal_ntc_t *dat, float *val);
void hal_ai_set_ntc_b(hal_ntc_t *dat, float *val);
void hal_ai_set_ntc_up(hal_ntc_t *dat, float *val);
void hal_ai_get_val_4_20_ma(hal_ai_t *p, float *val);
int hal_ai_init(hal_ai_t *p,unsigned int ch);
void hal_ai_cleanup(hal_ai_t *p, unsigned int ch);

void hal_ao_set_c_out(hal_ao_t *p, float *v);
void hal_ao_set_c_max(hal_ao_t *p, float *v);
void hal_ao_cleanup(hal_ao_t *p, int ch);

void hal_pt100_init(hal_pt100_t *p, int ch);
void hal_pt100_get_val(hal_pt100_t *p, float *weight); //128
void hal_pt100_set_in(hal_pt100_t *p, float *inval);
void hal_pt100_cleanup(hal_pt100_t *p, int ch);

void hal_TM7711_init(hal_TM7711_t *dat, int ch);
void hal_TM7711_get_rbri(hal_TM7711_t *p, float *weight);
void hal_TM7711_get_kcouple_value(hal_TM7711_t *p, float *weight);
void hal_TM7711_cleanup(hal_TM7711_t *dat, int ch);

///!进入休眠
void hw_sleep(void);

void hal_pwi_init(hal_pwi_t *dat, int ch);
void hal_pwi_get_freq(hal_pwi_t *dat, float *val);
void hal_pwi_cleanup(hal_pwi_t *dat, int ch);



void ai_get_pt100(hal_ai_t *p, float *val);
void hal_ai_get_pval(hal_ai_t *p, float *val);

int pi_init(pi_dat_t *, int);
unsigned int pi_get_hz(pi_dat_t *);
int PI_set_BOOL(int, char *);
void hal_ppi_set_filter(hal_ppi_t *p, unsigned int v);
void hal_ppi_set_min(hal_ppi_t *p, unsigned int us);
void hal_ppi_get_freq(hal_ppi_t *p, float *v);
int hal_ppi_init(hal_ppi_t *p, unsigned int ch);
void hal_ppi_cleanup(hal_ppi_t *p, unsigned int ch);



int hal_po_init(hal_po_t *p, int ch);
unsigned char hal_po_get_stop(hal_po_t *p);
void hal_po_set_cnt_mod(hal_po_t *p, unsigned char v);
int hal_po_dir(hal_po_t *p);
int hal_po_set_dir(hal_po_t *p, unsigned char v);
long hal_po_get_pos(hal_po_t *p);
int hal_po_init_position(hal_po_t *p, int v);
int hal_po_set_level(hal_po_t *p, unsigned char v);
int hal_po_set_out_mod(hal_po_t *p, int v);
int hal_po_set_run(hal_po_t *p, unsigned char en);
int hal_po_set_duty(hal_po_t *p, float *v);
int hal_po_set_fmax(hal_po_t *p, float *v);
int hal_po_set_fmin(hal_po_t *p, float *v);
int hal_po_set_freq(hal_po_t *p, float *f);
int hal_po_set_count(hal_po_t *p, unsigned int v);
int hal_po_set_pos_give(hal_po_t *p, unsigned char v);
int hal_po_start(hal_po_t *p);
int hal_po_stop(hal_po_t *p);
void hal_po_cleanup(hal_po_t *p, int ch);

//4-20mA
void ma4_20(int in, int in_min, int in_max, const float *ranger_min, const float *ranger_max, float *result);

//,16
void filter_bubbleSort(unsigned short arr[], unsigned short count);

void usb_slave_init(void);
void usb_slave_start(int en);
void usb_slave_close(void);

//LCD-
void lcd_en(int en);

//
void saveRetain(void);

//,kB
unsigned short get_remain_mem();
//,kB
unsigned short get_total_mem();

//
//uart2
void chk_uart2(void);

unsigned int nand_read_id(void);
int nand_read_page(unsigned int page_number, unsigned char *data, int data_len, unsigned char *spare, unsigned int spare_len);
unsigned int nand_write_page(unsigned int page_number, const uint8_t *data, uint32_t data_len, const uint8_t *spare, uint32_t spare_len);
int nand_mark_bad_block(unsigned int addr);
int nand_check_block(unsigned int addr);
unsigned int nand_erase_block(unsigned int block);

struct rtgui_app *rtgui_app_create(const char *title);
int rtgui_server_app_running();
void rt_hw_sdl_start(void);
void make_windows();

int rtgui_system_server_init(void);
//  void rt_hw_lcd_init();
void calibration_init(void);
void custom_font_add();
//  void start_remote();
long list_thread(void);
int stopPLC();
void hal_modbus_server_init();
void hal_modbus_client_init();

void device_init_app();

int hw_ai_init(hal_ai_t *p, unsigned int ch);
void hw_ai_set_period(hal_ai_t *p, unsigned short period);
void hw_ai_set_buf_size(hal_ai_t *p);
uint16_t hw_ai_get_direct(uint16_t ch); //AD,

DLL_API int mfc_uint8_write(hal_mfc_t *dev,  uint8_t classtype, uint8_t instance, uint8_t attrib, uint8_t dat);
DLL_API int mfc_uint8_read(hal_mfc_t *dev,  uint8_t classtype, uint8_t instance, uint8_t attrib, uint8_t *dat);
DLL_API int mfc_uint16_write(hal_mfc_t *dev,  uint8_t classtype, uint8_t instance, uint8_t attrib, uint16_t dat);
DLL_API int mfc_uint16_read(hal_mfc_t *dev,  uint8_t classtype, uint8_t instance, uint8_t attrib, uint16_t *dat);
DLL_API int mfc_fixed16_16_write(hal_mfc_t *dev,  uint8_t classtype, uint8_t instance, uint8_t attrib, uint32_t dat);
DLL_API int mfc_fixed16_16_read(hal_mfc_t *dev, uint8_t classtype, uint8_t instance, uint8_t attrib, uint32_t *dat);
DLL_API int mfc_text32_read(hal_mfc_t *dev,  uint8_t classtype, uint8_t instance, uint8_t attrib, uint8_t *dat);
DLL_API int hal_mfc_init(hal_mfc_t *dat, const char *portName);
int mfc_tran(hal_mfc_t *dev,uint8_t mode,uint8_t dlen,uint8_t classtype,uint8_t instance,uint8_t attrib,uint8_t *dat);
int mfc_lookat(hal_mfc_t *dat,int site);

int hal_banlance101_init(hal_banlance101_t *dat, const char *portName);
uint8_t hal_hl101_bitcmp_send(uint8_t ori, uint8_t newVal, uint16_t addr);
void hal_hl101_val_send(int newVal, uint16_t addr);

void txt_file_init(struct dat_txt_file *dat, int interval);


int read_cmp_context(comm_port_t *dev,  u8 len, const uint8_t *context, u32 timeout);
int read_context(comm_port_t *dev,  u8 len, const uint8_t *context, u32 timeout);
int read_head(comm_port_t *dev,u8 len, const uint8_t *context, u32 timeout);

int read_until(comm_port_t *dev, const uint8_t *context, u8 conetxt_len, u32 timeout);
int elt_read_head(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int elt_read_cmp_context(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int elt_read_context(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int elt_read_len(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);
int elt_read_data(protocol_dev_t *dev,protocol_t *prl,protocol_element_t *elt);


int8_t hal_rmb_b11_reset(hal_rmb_b11_t *dat);
int8_t hal_rmb_b11_status(hal_rmb_b11_t *dat);
int8_t hal_rmb_b11_poll(hal_rmb_b11_t *dat);
int8_t hal_rmb_b11_enbill(hal_rmb_b11_t *dat);
int8_t hal_rmb_b11_stack(hal_rmb_b11_t *dat);
int8_t hal_rmb_b11_return(hal_rmb_b11_t *dat);
char *hal_rmb_b11_ident(hal_rmb_b11_t *dat);
int8_t hal_rmb_b11_hold(hal_rmb_b11_t *dat);
const char *hal_rmb_b11_error(int z1, int z2);
void hal_rmb_b11_init(hal_rmb_b11_t *dat);

///! @comment main framework
void * model_init(void);
void model_process(void *);
///@end


///! OS
int thread_new(void (*func)(void *),void *arg);

///! OS end

typedef struct
{
    protocol_dev_t base;
    u8 level;          //安全等级,0-7,默认2
    unsigned admin;    //管理员mode
    unsigned secrity;  //加密
    unsigned use_pass; //启用密码
    unsigned rmold;    //残余指纹排除
    unsigned enforce;  //增强模式
    unsigned _auto;
    u8 key[8];    //		密钥
    u8 busy;
    u8 baudrate;
    u8 id_len;
    u16 detms;
    u16 id;
    u8 username[9];
    u8 user_role;   //用户角色,'0'=admin ,'1'-'9':用户
    //--------------
    unsigned admin_online, passok;
    u8 voltage;
    u8 voltage_status;
    u8 finger;
    //--------工作过程参数
    protocol_t *current;
    u8 *context,*context1;
} hal_df214_t;




///! @TM7706 ADC mai
typedef struct
{
    spi_io_def spi;
    io_def drdy;
    io_def reset;
    void *hal[4];
#ifdef RT_THREAD
    rt_thread_t thread;
#endif
} hw_tm7706_t;
extern hw_tm7706_t hw_7706[];
typedef struct
{
    //@param
    int raw;	///! @variable ADC getvalue
    float max,min;
    float value;
    uint8_t single; ///
    uint8_t gain,index;
    //@output
    //@internal
    int times,ok;
    u32 _speed,speed;	//cps
    hal_debounce_t filter;
    int status;
    u16 buf[5];
} hal_tm7706_t;
typedef struct
{
    int raw;    ///! @variable ADC getvalue
    int com_val;
    float max,min;
    float value;
    uint8_t single; ///@
    uint16_t freq;  ///
    uint8_t gain;
    int times,ok;
    u32 _speed,speed;	//cps
    int status;
} hal_tm7707_t;


void hal_tm7706_init(hal_tm7706_t *,uint8_t channel);
int hal_tm7706_get_raw(hal_tm7706_t *);
void hal_tm7706_get_vout(hal_tm7706_t *,float*);
void hal_tm7706_set_max(hal_tm7706_t *,float*);
void hal_tm7706_set_min(hal_tm7706_t *,float*);
void hal_tm7706_cleanup(hal_tm7706_t *dat,uint8_t ch);
void hal_tm7707_init(hal_tm7707_t *,uint8_t channel);
int hal_tm7707_get_raw(hal_tm7707_t *);
void hal_tm7707_get_vout(hal_tm7707_t *,float*);
void hal_tm7707_set_max(hal_tm7707_t *,float*);
void hal_tm7707_set_min(hal_tm7707_t *,float*);
void hal_tm7707_get_pt100_r_a(hal_tm7707_t *dat,float* val);

void hal_tm7707_cleanup(hal_tm7707_t *dat,uint8_t ch);
///@end

/** defgroup rf rf wireless
* @{
*/
typedef struct
{
    spi_io_def spi;
    io_def ce;
    io_def irq;
} hw_24L01_t;

typedef struct
{
    const spi_io_def spi;
    io_def rst;
    io_def irq;
} hw_lt8920_t;

typedef struct
{
    io_def ce;
    io_def clk;
    io_def data;
    uint8_t delay;
    uint8_t cpha,cpol;
} hw_xn297lbw_t;
extern const hw_24L01_t hw_24L01;
extern const hw_xn297lbw_t hw_xn297lbw;

typedef struct
{
    comm_port_t base;
    uint8_t addr;
    uint8_t freq;
    const void *hw;
    u32 index;
    int status;
    uint32_t tx_count,rx_count;
    uint32_t poll_count;
    uint32_t err_count;
    uint32_t rx_tick;       ///!last recevice tick
    uint32_t tick_to_rst;
    unsigned inited;
    int (*rx_cb[6])(void *dat, uint8_t *addr,uint8_t *,uint8_t len);
    uint8_t rxbuf[32],txbuf[32];
} hal_rf_client;
typedef struct
{
    comm_port_t base;
    uint8_t addr[5];    ///! @param local address
    uint8_t freq;
    u32 index;
    int status;
    uint32_t tx_count,rx_count;
    uint32_t poll_count;
    uint32_t err_count;
    uint32_t rx_tick;       ///!last recevice tick
    uint32_t tick_to_rst;
    unsigned inited;
    const hw_xn297lbw_t *hw;
    int (*rx_cb[6])(void *dat, uint8_t *addr,uint8_t *,uint8_t len);
    uint8_t rxbuf[64];
} hal_xn297lbw_client;
extern hal_xn297lbw_client hal_xn297lbw;
extern hal_rf_client hal_lt8920;
void lt8920_irq_disable();
void lt8920_irq_enable();

/** rf frame check
*	@return frame number
**/
int rf_check(buf_data *);

//* @}


//*	watchdog
int watchdog_init(int d);
void watchdog_reset();
int watchdog_stop(void);
extern u16 WATCHDOG_TIME;
//*



/** get cpu id
**/
u32 cpu_id();

extern const io_def LEDRUN;
extern const io_def LEDMSG;

#ifdef __cplusplus
}
#endif
#endif
