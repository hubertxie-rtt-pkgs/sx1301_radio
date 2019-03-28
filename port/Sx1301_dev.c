/******************************************************************************

                  版权所有 (C), 2017-2020, 上海汇珏智能通讯科技有限公司

 ******************************************************************************
  文 件 名   : sx1301_spidev.c
  版 本 号   : 初稿
  作    者   : zhengym
  生成日期   : 2017年12月10日
  最近修改   :
  功能描述   : sx1301 spi驱动设备文件
  函数列表   :
  修改历史   :
  1.日    期   : 2017年12月10日
    作    者   : zhengym
    修改内容   : 创建文件

******************************************************************************/

#if !defined(LOG_TAG)
    #define LOG_TAG                    "sx301_dev"
#endif

// 本模块的打印等级
#define DRV_SX1301_SPI_LOG_LVL         ELOG_LVL_INFO

#undef LOG_LVL
#if defined(DRV_SX1301_SPI_LOG_LVL)
    #define LOG_LVL                    DRV_SX1301_SPI_LOG_LVL
#endif

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include <stdint.h>        /* C99 types */
#include <stdbool.h>       /* bool type */
#include <stdio.h>         /* log_i */
#include <string.h>        /* memset */
#include <rtdevice.h>

#include "board.h"
#include "global.h"
#include "tools.h"
#include "elog.h"

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

#include "common.h"
#include "Drv_spi.h"

#include "Sx1301_dev.h"

//#include "Lora_api.h"
//#include "Smoke_protocol_parse.h"
#include "aliYunGateWay.h"

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
#define DEFAULT_RSSI_OFFSET 0.0
#define DEFAULT_NOTCH_FREQ  129000U

//radio A , B freq
#define DEF_FA_FRAQ_HZ     911000000
#define DEF_FB_FRAQ_HZ     910000000

//radio 收发超时时间默认值
#define DEF_BROADCAST_OUT_TIME    20*1000 // (20) *(1S)
#define DEF_RX_OUT_TIME           5*60*1000 // 5min
/*----------------------------------------------*
 * 模块结构体申明                                *
 *----------------------------------------------*/
 struct lora_paramter
 {
    uint32_t    tx_freq_hz;        /*!> center frequency of TX */
    uint32_t    fa_freq_hz;        /*!> center frequency of A radio */
    uint32_t    fb_freq_hz;        /*!> center frequency of B radio */
    uint8_t     rf_chain;       /*!> through which RF chain will the packet be sent */
    int8_t      rf_power;       /*!> TX power, in dBm */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    uint16_t    preamble;       /*!> set the preamble length, 0 for default */
    bool        no_crc;         /*!> if true, do not send a CRC in the packet */ 
 };

//驱动硬件资源
struct lr_drv_src
{
    // 发送中断脚
    uint8_t tx_pin;
    // 复位脚
    uint8_t rst_pin;
    //spi 资源
    uint8_t cs_pin; //选片脚
    struct rt_spi_device sx1301_spihnd;    
};
struct lr_dev
 {
//    rt_mailbox_t lr_api_msg;//lora 驱动引擎接口邮箱
    struct lr_drv_src src;
 };
/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
int LoraSendto(uint8_t *data,uint16_t len,uint32_t timeout,struct radio_pram *pram);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
struct lr_dev lora_dev;

struct lora_paramter lora_param;

//定时器变量
uint32_t broadcast_outTime;
uint32_t rx_out_timeValue;
rt_timer_t broadcast_timer;
rt_timer_t rx_timer;
//网络帧参数

//信道索引号
uint8_t chnl_index;

//广播开关
static rt_sem_t brocast_enable;
//接收超时
static rt_sem_t rx_outTime;

//缓存邮箱
rt_mailbox_t CachMail;
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/
/* describe command line options */
void usage(void) {
    log_i("Library version information: %s\n", lgw_version_info());
    log_i( "Available options:\n");
    log_i( " -h print this help\n");
    log_i( " -a <float> Radio A RX frequency in MHz\n");
    log_i( " -b <float> Radio B RX frequency in MHz\n");
    log_i( " -t <float> Radio TX frequency in MHz\n");
    log_i( " -r <int> Radio type (SX1255:1255, SX1257:1257)\n");
    log_i( " -k <int> Concentrator clock source (0: radio_A, 1: radio_B(default))\n");
}

/*****************************************************************************
 函 数 名  : lora_radio_isr
 功能描述  : lora中断底半处理函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : int
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月20日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
void lora_radio_isr( struct lr_dev *dev )
{
//    if ( !broadcast_timer )
//    {
//        rt_kprintf("broadcast_timer is null in lora_radio_isr\r\n");
//        return;
//    }
//    rt_timer_stop(broadcast_timer);
}
/*****************************************************************************
 函 数 名  : lora_spi_attach
 功能描述  : lora spi 设备挂载
 输入参数  : uint8_t cs_pin      
             uint8_t *dev_name   
             uint8_t *bus_name   
             struct lr_dev *dev  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月21日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
int lora_spi_attach(uint8_t *dev_name,uint8_t *bus_name,struct lr_drv_src *src)
{
    if(RT_EOK != drv_spi_bus_attach_device(PIN_SX1301_SPI_CS, bus_name, dev_name))
    {
        log_e("sx1301 挂载spi总线失败");
        return -1;
    }
    
    rt_device_t dev = rt_device_find(dev_name);
    if ( !dev )
    {
        log_e("dev_name find fail\r\n");
        return -1;
    }
    
      /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = 1000*1000; /* 1M */
        rt_spi_configure((struct rt_spi_device *)dev, &cfg);
    } /* config spi */
    return 0;
}

/*****************************************************************************
 函 数 名  : lora_rst_cfg
 功能描述  : 配置复位脚
 输入参数  : uint8_t
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月21日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
void lora_rst_cfg( uint8_t rst_pin,struct lr_drv_src *src)
{   
    src->rst_pin = rst_pin;
    //配置成输出模式拉低
    rt_pin_mode(rst_pin, PIN_MODE_OUTPUT);
    rt_pin_write(rst_pin, PIN_LOW);
}


/*****************************************************************************
 函 数 名  : Init_sx1301_spidev
 功能描述  : 初始化sx1301 spi驱动
 输入参数  : 无
 输出参数  : 无
 返 回 值  : rt_err_t
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月10日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/

int SX1301_hoard_init( )
{
    uint8_t rt=0;

     //复位脚配置为输出模式
    lora_rst_cfg(PIN_LA_SX1301_RST,&lora_dev.src);
    
    //配置挂载lora spi
    rt += lora_spi_attach(RT_SX1301_SPI_DEV_NAME,RT_SX1301_SPI_BUS_NAME,&lora_dev.src);
   
    return rt;
}
/*****************************************************************************
 函 数 名  : def_init
 功能描述  : radio参数默认配置
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年3月26日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
int defParam_init()
{  
    lora_param.bandwidth = BW_125KHZ;
    lora_param.coderate = CR_LORA_4_5;
    lora_param.fa_freq_hz = 916200000;
    lora_param.fb_freq_hz = 915600000;
    lora_param.no_crc = false;
    lora_param.preamble = 6;
    lora_param.rf_chain = 0;
    lora_param.rf_power = 10;
    lora_param.tx_freq_hz = 916800000; 

    //广播配置频率
//     broadcast_outTime = DEF_BROADCAST_OUT_TIME;
    //收发超时时间
    rx_out_timeValue = DEF_RX_OUT_TIME;
    //信道索引
    chnl_index = 0;
    return 0;
}

int SX1301_init()
{
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;

    int i, j;
    uint32_t fa = 0, fb = 0;
    enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_NONE;
    uint8_t clocksource = 1; /* Radio B is source by default */

    double xd = 0.0;    
    int xi = 0;
    OsDelayMs(2000);

// freq 900 - 931  (M)
    //配置接收频点
    if (lora_param.fa_freq_hz < 900000000 || 931000000 < lora_param.fa_freq_hz)
    {
       log_w(" Radio A RX is illeglity : %u\n", lora_param.fa_freq_hz);
       fa = DEF_FA_FRAQ_HZ;
    }
    fa = lora_param.fa_freq_hz;
    
    if (lora_param.fb_freq_hz < 900000000 || 931000000 < lora_param.fb_freq_hz)
    {
       log_w(" Radio B RX is illeglity : %u\n", lora_param.fb_freq_hz);
       fb = DEF_FB_FRAQ_HZ;
    }
    fb = lora_param.fb_freq_hz;

    /* check input parameters */
    if ((fa == 0) || (fb == 0)) {
        log_e("ERROR: missing frequency input parameter:\n");
        log_e("  Radio A RX: %u\n", fa);
        log_e("  Radio B RX: %u\n", fb);
       
        usage();
        return -1;
    }
    radio_type = LGW_RADIO_TYPE_SX1257;
    if (radio_type == LGW_RADIO_TYPE_NONE) {
        log_i("ERROR: missing radio type parameter:\n");
        usage();
        return -1;
    }

    /* beginning of LoRa concentrator-specific code */
    log_i("Beginning of test for loragw_hal.c\n");

    log_i("*** Library version information ***\n%s\n\n", lgw_version_info());

    /* set configuration for board */
    memset(&boardconf, 0, sizeof(boardconf));

    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    lgw_board_setconf(boardconf);

    /* set configuration for RF chains */
    memset(&rfconf, 0, sizeof(rfconf));

    rfconf.enable = true;
    rfconf.freq_hz = fa;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    rfconf.tx_notch_freq = DEFAULT_NOTCH_FREQ;
    lgw_rxrf_setconf(0, rfconf); /* radio A, f0 */

    rfconf.enable = true;
    rfconf.freq_hz = fb;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    lgw_rxrf_setconf(1, rfconf); /* radio B, f1 */

    /* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
    memset(&ifconf, 0, sizeof(ifconf));

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = -400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(0, ifconf); /* chain 0: LoRa 125kHz, all SF, on f1 - 0.4 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = -200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(1, ifconf); /* chain 1: LoRa 125kHz, all SF, on f1 - 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 0;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(2, ifconf); /* chain 2: LoRa 125kHz, all SF, on f1 - 0.0 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = -400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(3, ifconf); /* chain 3: LoRa 125kHz, all SF, on f0 - 0.4 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = -200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(4, ifconf); /* chain 4: LoRa 125kHz, all SF, on f0 - 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 0;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(5, ifconf); /* chain 5: LoRa 125kHz, all SF, on f0 + 0.0 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(6, ifconf); /* chain 6: LoRa 125kHz, all SF, on f0 + 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(7, ifconf); /* chain 7: LoRa 125kHz, all SF, on f0 + 0.4 MHz */

    /* set configuration for LoRa 'stand alone' channel */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 0;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.datarate = DR_LORA_SF10;
    lgw_rxif_setconf(8, ifconf); /* chain 8: LoRa 250kHz, SF10, on f0 MHz */

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 0;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.datarate = 64000;
    lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, on f1 MHz */

    /* connect, configure and start the LoRa concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        log_i("*** Concentrator started ***\n");
        return 0;
    } else {
        log_e("*** Impossible to start concentrator ***\n");
        return -1;
    }

}


void broadcast_timeout(void * parameter)
{
    log_d("broadcast timeout\r\n");
 
    if ( !brocast_enable )
    {
        log_e("brocast_enable is null\n");
        return;
    }
    rt_sem_release(brocast_enable);
}

void rx_timeout(void * parameter)
{
    log_w("rx_timeout\r\n");
    if ( !rx_outTime )
    {
        log_e("rx_outTime is null\n");
        return;
    }
    rt_sem_release(rx_outTime);
}
/*****************************************************************************
 函 数 名  : Lora_sendto
 功能描述  : lora 发送
 输入参数  : timeout (单位100ms)
 输出参数  : 无
 返 回 值  : int
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月20日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
static struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */
int LoraSendto(uint8_t *data,uint16_t len,uint32_t timeout,struct radio_pram *pram)
{
    rt_err_t rt;

    int i, j;

    uint32_t ft;
    uint8_t status_var = 0;

    ft = lora_param.tx_freq_hz;
    //申请buff内存
//    txpkt.payload = rt_malloc(256);
//    if (!txpkt.payload)
//    {
//       log_e("txpkt.payload is null\r\n");
//    }
    
    /* set configuration for TX packet */
    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz = ft;

    txpkt.tx_mode = IMMEDIATE;
    txpkt.rf_power = 10;
    txpkt.modulation = MOD_LORA;
    txpkt.bandwidth = BW_125KHZ;
//    txpkt.datarate = DR_LORA_SF9;
//    txpkt.coderate = CR_LORA_4_5;

    txpkt.datarate = pram->sf;
    txpkt.coderate = pram->cr;
    memcpy(txpkt.payload, data,len);
    txpkt.size = len;
    txpkt.preamble = 6;
    txpkt.rf_chain = 0;
    txpkt.invert_pol = false;
    txpkt.no_crc = false;
    log_d("TX freq hz %d: TX crc %d",ft,txpkt.no_crc);
    switch (txpkt.modulation) {
        case MOD_LORA: log_i(" LoRa"); break;
        case MOD_FSK: log_i(" FSK"); break;
        default: log_i(" modulation?");
    }
    switch (txpkt.datarate) {
        case DR_LORA_SF7: log_i(" SF7"); break;
        case DR_LORA_SF8: log_i(" SF8"); break;
        case DR_LORA_SF9: log_i(" SF9"); break;
        case DR_LORA_SF10: log_i(" SF10"); break;
        case DR_LORA_SF11: log_i(" SF11"); break;
        case DR_LORA_SF12: log_i(" SF12"); break;
        default: log_i(" datarate?");
    }
    switch (txpkt.coderate) {
        case CR_LORA_4_5: log_i(" CR1(4/5)"); break;
        case CR_LORA_4_6: log_i(" CR2(2/3)"); break;
        case CR_LORA_4_7: log_i(" CR3(4/7)"); break;
        case CR_LORA_4_8: log_i(" CR4(1/2)"); break;
        default: log_i(" coderate?");
    }
    
    i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
    if ( LGW_HAL_SUCCESS !=  i)
    {
        log_w("lgw_send fail");
        return RT_ERROR;
    }

    j = 0;
    log_i("rf path %d, return %d\nstatus -> ", txpkt.rf_chain, i);
    do {
        ++j;
        OsDelayMs(100);
        lgw_status(TX_STATUS, &status_var); /* get TX status */
        log_i("%d:", status_var);
    } while ((status_var != TX_FREE) && (j < timeout));
    
    if (j < timeout)
    {
      log_i("TX finished\n"); 
      rt = RT_EOK;
    }
    else
    {
    // TODO:重启radio
//        broadcast_timeout(NULL);
        rt = RT_ETIMEOUT;
    }
    //释放发送buff内存
//    rt_free(txpkt.payload);
    return rt;
}
struct SX1301Msg
{
    uint8_t *dat;
    uint8_t dat_len;
    uint8_t sf;
    uint8_t cr;
};

#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157
int16_t clac_rssi(int8_t snr,int16_t rssi)
{
    
    if( snr & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        snr = ( ( ~snr + 1 ) & 0xFF ) >> 2;
        snr = -snr;
    }
    else
    {
        // Divide by 4
        snr = ( snr & 0xFF ) >> 2;
    }

    if( snr < 0 )
    {
            rssi = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +  snr;
       
    }
    else
    {
            rssi = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
  
    }

}
/*****************************************************************************
 函 数 名  : lora_drv_thd
 功能描述  : lora 驱动引擎
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月21日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
void lora_recv_thd( struct lr_dev *dev)
{
    rt_err_t rt;
    int i, j;
    int nb_pkt = 0;

    struct lgw_pkt_rx_s *p; /* pointer on a RX packet */

    rt_event_recv(wait_event, WAIT_EVT_NET_REDAY, RT_EVENT_FLAG_AND, RT_WAITING_FOREVER, NULL); 
    OsDelayMs(1000);
   
    //lora 初始化
    if(SX1301_init() < 0)
    {
        return ;
    }
    
    //开启接收超时定时器
    if (rx_timer)
    {
        log_d("rx out time start\r\n");
        rt_timer_start(rx_timer);
    }
    thread_wdt_start(NULL);
    while (1){

        //检测接收是否超时
        rt = rt_sem_take(rx_outTime,RT_WAITING_NO);
        if ( rt == RT_EOK )
        {
            log_i("rx out time radio reset\n");
            //接收超时
            lgw_stop();//停止radio
            
            //设置radio参数
            SX1301_init();

             //开启接收超时定时器
            if (rx_timer)
            {
                log_d("rx out time start\r\n");
                rt_timer_start(rx_timer);
            }                
        }
        
        /* fetch N packets */
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

        if (nb_pkt == 0) {
            OsDelayMs(10);
        } else {
            /* display received packets */
            for(i=0; i < nb_pkt; ++i) {
                p = &rxpkt[i];
                log_i("---\nRcv pkt #%d >>", i+1);
                if (p->status == STAT_CRC_OK) {
                    log_i(" if_chain:%2d", p->if_chain);
                    log_i(" tstamp:%010u", p->count_us);
                    log_i(" size:%3u", p->size);
                    switch (p->modulation) {
                        case MOD_LORA: log_i(" LoRa"); break;
                        case MOD_FSK: log_i(" FSK"); break;
                        default: log_i(" modulation?");
                    }
                    switch (p->datarate) {
                        case DR_LORA_SF7: log_i(" SF7"); break;
                        case DR_LORA_SF8: log_i(" SF8"); break;
                        case DR_LORA_SF9: log_i(" SF9"); break;
                        case DR_LORA_SF10: log_i(" SF10"); break;
                        case DR_LORA_SF11: log_i(" SF11"); break;
                        case DR_LORA_SF12: log_i(" SF12"); break;
                        default: log_i(" datarate?");
                    }
                    switch (p->coderate) {
                        case CR_LORA_4_5: log_i(" CR1(4/5)"); break;
                        case CR_LORA_4_6: log_i(" CR2(2/3)"); break;
                        case CR_LORA_4_7: log_i(" CR3(4/7)"); break;
                        case CR_LORA_4_8: log_i(" CR4(1/2)"); break;
                        default: log_i(" coderate?");
                    }
                    log_i("\n");
                    log_i(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", p->rssi, p->snr, p->snr_min, p->snr_max);

                    cmd_free(NULL, NULL);
                    //刷新数据接收定期器
                    rt_timer_start(rx_timer);

                    int16_t rssi = clac_rssi(p->snr, p->rssi);
                    log_i("recv rssi:%d",rssi);
                    
                    struct SX1301Msg *msg = rt_malloc(sizeof(struct SX1301Msg));
                    if (!msg)
                    {
                        log_e("msg is null\r\n");
                        continue;
                    }
                    
                    uint8_t *ptr;
                    ptr = rt_malloc(p->size);
                    if (!ptr)
                    {
                        log_e("msg->dat is null\r\n");
                        continue;
                    }
                    //填写消息信息
                    msg->cr = p->coderate;
                    msg->sf = p->datarate;
                    msg->dat = ptr;
                    msg->dat_len = p->size;
                    memcpy(msg->dat, p->payload, p->size);
                    if ( !CachMail )
                    {
                        log_e("SX1301_mail is null\r\n");
                        continue;
                    }
                    rt_err_t err = rt_mb_send(CachMail, (rt_uint32_t)msg);
                    if ( err != RT_EOK)
                    {
                        log_e("lora msg send fail\r\n");
                        continue;
                    }
                    
                } else if (p->status == STAT_CRC_BAD) {
                    log_i(" if_chain:%2d", p->if_chain);
                    log_i(" tstamp:%010u", p->count_us);
                    log_i(" size:%3u\n", p->size);
                    log_i(" CRC error, damaged packet\n\n");
                } else if (p->status == STAT_NO_CRC){
                    log_i(" if_chain:%2d", p->if_chain);
                    log_i(" tstamp:%010u", p->count_us);
                    log_i(" size:%3u\n", p->size);
                    log_i(" no CRC\n\n");
                } else {
                    log_i(" if_chain:%2d", p->if_chain);
                    log_i(" tstamp:%010u", p->count_us);
                    log_i(" size:%3u\n", p->size);
                    log_i(" invalid status ?!?\n\n");
                }
            }
        }
     }
    log_i("\nEnd of test for loragw_hal.c\n");
}
#if 0
static uint8_t data[256];
static uint16_t len;
void lora_brocst_thd( struct lr_dev *dev)
{
    rt_err_t err;
    smoke_protocol_frame nwk_frame;

    OsDelayMs(1000);
    //开始广播定时器
    if (broadcast_timer)
    {
        log_d("broadcast outtimer start\r\n");
        rt_timer_start(broadcast_timer);
    }
    while(1)
    {
        err = rt_sem_take(brocast_enable, RT_WAITING_FOREVER);
        if (RT_EOK != err)
        {
            continue;
        }
        //设置信道索引
        nwk_frame.nwk_frame.chnl_index = chnl_index; 
        //打包广播数据发送
        len = smoke_prepare_frame(data,&nwk_frame,NWK_FRAME_MODE);
        if (RT_EOK != LoraSendto(data,len,100))
        {
            log_e("broadcast send fail\r\n");
        }
    }
    
}
#endif

int SX1301Recv(uint8_t *dat, uint8_t *datlen,uint32_t timeout,struct radio_pram *pram)
{
    int rt;
    struct SX1301Msg *msg;
    RT_ASSERT(dat != RT_NULL);
    RT_ASSERT(datlen != RT_NULL);

    //检查邮箱是否存在
    if (!CachMail)
    {
        log_e("CachMail is null\r\n");
        return RT_EEMPTY;
    }
    //读取数据缓存邮箱
    if (RT_EOK != rt_mb_recv(CachMail, (rt_uint32_t *)&msg, TIME_TO_OSTICK(timeout)))
    {
        
       // rt_kprintf("msg is null\r\n");
        return RT_EEMPTY;
    }

    if (!msg)
    {
        log_e("msg is null\r\n");
        return RT_EEMPTY;
    }
    
    if (!msg->dat)
    {
        log_e("msg->dat is null\r\n");
        rt_free(msg);
        return RT_EEMPTY;
    }
    //获取数据
    if (0 != crc8_calc(0, msg->dat, msg->dat_len))
    {
        log_e("crc is fail\r\n");
        rt = RT_EEMPTY;
    }
    else
    {
        pram->sf = msg->sf;
        pram->cr = msg->cr;
        memcpy(dat, msg->dat, msg->dat_len);
        *datlen = msg->dat_len;
        rt = RT_EOK;
    }
    log_mem(ELOG_LVL_INFO,"SX1301Recv",msg->dat,msg->dat_len);
    
    if (msg->dat)
    {
        log_d("msg->dat is free\r\n");
        rt_free(msg->dat);
    }
    //释放资源
    if (msg)
    {
       log_d("msg is free\r\n");
       rt_free(msg);
    }
    return rt;
}
/*****************************************************************************
 函 数 名  : Lora_init
 功能描述  : Lora初始化
 输入参数  : 无
 输出参数  : 无
 返 回 值  : int
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2017年12月20日
    作    者   : zhengym
    修改内容   : 新生成函数

*****************************************************************************/
int lora_DevInit( )
{
    int rt=0;
    
     //初始化sx1301硬件
    rt = SX1301_hoard_init( );
    if ( rt < 0 )
    {
        log_e("SX1301_hoard_init is fail\r\n");
        return rt;
    }
    //参数默认初始化
    defParam_init();
#if 0   
    //创建超时定时器
    broadcast_timer = rt_timer_create("TX outtimer", broadcast_timeout,
                    NULL, TIME_TO_OSTICK(broadcast_outTime), 
                    RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_PERIODIC);

    if (!broadcast_timer)
    {
    log_e("broadcast_timer create fail\r\n");
    }
#endif

    //创建接收超时定时器
    rx_timer = rt_timer_create("RX outtimer", rx_timeout,
                        NULL, TIME_TO_OSTICK(rx_out_timeValue), 
                        RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_ONE_SHOT);

    if (!rx_timer)
    {
        log_e("RX outtimer timer_create fail\r\n");
    }
    
    //接收超时信号量
    rx_outTime = rt_sem_create("rx_outTime", 0, RT_IPC_FLAG_FIFO);
    if (!rx_outTime)
    {
        log_e("rx_outTime sem_create fail\n");
    }
    //创建数据缓存邮箱
    CachMail = rt_mb_create("sx1301 cach",20, RT_IPC_FLAG_FIFO);
    if (!CachMail)
    {
        log_e("CachMail fail\n");
    }

#if 0
    //创建广播发送信号量
    brocast_enable = rt_sem_create("brocast_enable", 0, RT_IPC_FLAG_FIFO);
    if (!brocast_enable)
    {
        log_e("brocast_enable create fail\n");
    }
#endif

    
    //创建lora驱动引擎线程
    rt_thread_t th = rt_thread_create("lora recv", 
                                (void (*)(void *))lora_recv_thd, 
                                &lora_dev,1500, 
                                THREAD_PRIO_RADIO_ERCV, 10);
    if (th)
    {
        rt_thread_startup(th);
    }
    
#if 0
    th = rt_thread_create("lora send", (void (*)(void *))lora_brocst_thd, 
                    &lora_dev, 2048, 
                    THREAD_PRIO_RADIO_BROCST,10);
    if (th)
    {
        rt_thread_startup(th);
    }
#endif

    return rt;
}
INIT_APP_EXPORT(lora_DevInit);
