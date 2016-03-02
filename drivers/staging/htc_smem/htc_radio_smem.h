#ifndef _HTC_RADIO_SMEM_H
#define _HTC_RADIO_SMEM_H

#include <linux/types.h>

#if 0
//hTC radio smem driver version
#define HTC_RADIO_SMEM_VERSION	0x20140901

//APP run mode
#define APP_IN_BOOTLOADER       (1 << 0)
#define APP_IN_TESTBOOTLOADER   (1 << 1)
#define APP_IN_DIAG             (1 << 2)
#define APP_IN_RECOVERY         (1 << 3)
#define APP_IN_MFGKERNEL        (1 << 4)
#define APP_IN_HLOS             (1 << 5)
#endif

struct htc_modem_request_type
{
    uint32_t  cmdseq;
    uint32_t  rspseq;

    uint32_t  opcode;
    uint32_t  reserve;
    uint32_t  parameter[4];
    uint32_t  response[4];

};

struct htc_smem_type
{
/* ========= belows are App write ==================== */
    uint32_t    version;
    uint32_t    struct_size;

    uint32_t    htc_smem_ce_radio_dbg_flag;
    uint32_t    htc_smem_app_run_mode;
    uint32_t    htc_smem_test_flag;
    uint32_t    htc_smem_boot_reason;
    uint32_t    htc_cable_status;
    uint8_t     reserve1[4];

/* ========= belows are modem write ==================== */
    uint32_t    version_R;
    uint32_t    struct_size_R;

    uint32_t    htc_smem_erase_efs_flag;
    uint32_t    htc_smem_flight_mode_flag;
    uint8_t     htc_radio_version_addr[16];  //modem fill it
    uint8_t     htc_protocol_version_addr[16]; // modem fill it

/* HTC_INTEGRATE_8974PCN_DBG_005 */
    uint32_t    htc_debug_FATAL_count;
    uint32_t    htc_debug_FATAL_NO_NW_count;
    uint32_t    htc_enable_modem_auto_fatal;
/* End HTC_INTEGRATE_8974PCN_DBG_005 */

/* HTC_INTEGRATE_8960PCN_DBG_006 */
    uint32_t    htc_err_fatal_handler_magic;
/* End HTC_INTEGRATE_8960PCN_DBG_006 */

/* ========= belows are shared ==================== */
    struct htc_modem_request_type  htc_modem_request;             // for error handling only

/* for eMMC feature */
    uint32_t    tc_emmc_magic_flag;
    uint32_t    htc_emmc_buff_addr;
    uint32_t    htc_emmc_buff_size;
    uint32_t    htc_emmc_config_offset;
    uint32_t    htc_emmc_efs_sync_status;
    uint32_t    htc_emmc_nv_calibrate_status;
    uint32_t    htc_emmc_is_dev_inited;

    uint32_t    htc_smem_user_time_offset;

/* radio debug */
// Use 32 bytes to record the TCXO shutdown time statistics
    uint32_t    htc_tcxo_off_time_total;
    uint32_t    htc_tcxo_off_cnt_total;
    uint32_t    htc_tcxo_off_time_pwrc_suspend;
    uint32_t    htc_tcxo_off_cnt_pwrc_suspend;
    uint32_t    htc_global_garbage_cnt;//Offset: C0
    uint32_t    htc_mssahb_reset_status;
    uint32_t    htc_watchdog_status;
    uint32_t    htc_smlog_start_addr_for_apps;
    uint32_t    htc_smlog_max_size_for_apps;

#if 1 /* HTC_INTEGRATE_CIQ_006 */
    uint32_t    htc_ciq_flag;
#endif /*END HTC_INTEGRATE_CIQ_006 */
/* BEGIN [CR00118]<P: 3/5>================================================ */
/* For UMTS, it is not used. Only for SMEM Aligment with CDMA*/
/*
   Author : Arthur.BY Huang
   Review : Jeff Wu
   Date: 2011/12/22
   Description : Following code is based on SSD Bruce Chang sample code.
                      SSD would like to measure time of each signal bar.
                      We help to review/modifiy the test code from SSD and phase in this CR.
   [2011/12/28-Update] Based on SSD CM Chen's suggestion, move following SMEM from htc_smem_param.h to htc_smem_ram in htc_smem.h.
   [2011/12/31 - Update2] Add CDMA info fron Miles's modification.
*/
/* ========================================================================== */
#if 1 /*HTC Align HTC_SMEM Structure within CDMA/UMTS/AND */
//HTC_WSD_CHG_TC_BAR_TIME  //Roger, 20110927, CR00118

//------------------------------------------------------------------------------
// Not Used
//bc test:DCH
    uint32_t    htc_modem_info_dch_time;
    uint32_t    htc_modem_info_fach_time;
    uint32_t    htc_modem_info_3g_cs_bar1_time;
    uint32_t    htc_modem_info_3g_cs_bar2_time;
    uint32_t    htc_modem_info_3g_cs_bar3_time;
    uint32_t    htc_modem_info_3g_cs_bar4_time;
    uint32_t    htc_modem_info_3g_ps_bar1_time;
    uint32_t    htc_modem_info_3g_ps_bar2_time;
    uint32_t    htc_modem_info_3g_ps_bar3_time;
    uint32_t    htc_modem_info_3g_ps_bar4_time;
    uint32_t    htc_modem_info_2g_cs_bar1_time;
    uint32_t    htc_modem_info_2g_cs_bar2_time;
    uint32_t    htc_modem_info_2g_cs_bar3_time;
    uint32_t    htc_modem_info_2g_cs_bar4_time;
    uint32_t    htc_modem_info_cs_bar1_time_1x;
    uint32_t    htc_modem_info_cs_bar2_time_1x;
    uint32_t    htc_modem_info_cs_bar3_time_1x;
    uint32_t    htc_modem_info_cs_bar4_time_1x;
    uint32_t    htc_modem_info_cs_bar5_time_1x;
    uint32_t    htc_modem_info_ps_bar1_time_ev;
    uint32_t    htc_modem_info_ps_bar2_time_ev;
    uint32_t    htc_modem_info_ps_bar3_time_ev;
    uint32_t    htc_modem_info_ps_bar4_time_ev;
    uint32_t    htc_modem_info_ps_bar5_time_ev;
    uint32_t    htc_modem_info_ps_bar1_time_lte;
    uint32_t    htc_modem_info_ps_bar2_time_lte;
    uint32_t    htc_modem_info_ps_bar3_time_lte;
    uint32_t    htc_modem_info_ps_bar4_time_lte;
    uint32_t    htc_modem_info_ps_bar5_time_lte;
/*
    uint32_t    htc_modem_info_ps_bar1_time_1x;
    uint32_t    htc_modem_info_ps_bar2_time_1x;
    uint32_t    htc_modem_info_ps_bar3_time_1x;
    uint32_t    htc_modem_info_ps_bar4_time_1x;
    uint32_t    htc_modem_info_ps_bar5_time_1x;
*/
#endif //HTC_WSD_CHG_TC_BAR_TIME       //Roger, 20110927, CR00118
/* END   [CR00118]======================================================= */
    uint32_t    htc_global_SYNCACK_garbage_cnt;/*HTC_INTEGRATE_8960PCN_GF_003*///Offset:14C

    uint32_t    htc_smlog_magic;

    uint8_t     htc_rom_ver[16]; /* HTC_INTEGRATE_8974PCN_DBG_009 */

    uint32_t    htc_smem_ce_radio_dbg_flag_ext1;

    uint32_t    htc_smem_ce_radio_dbg_flag_ext2;

#if 1 /* HTC_INTEGRATE_8974PCN_DBG_020*/
    uint32_t    htc_smem_ce_radio_dbg_ril_fatal; //  Sync with CDMA project. For oem-98/99 case, the APPS will set 0x6F656D98/0x6F656D99 at the address 0xFBF016C
#endif /* HTC_INTEGRATE_8974PCN_DBG_020*/

#if 1 //FEATURE_8974_WP8_SUPPORT // HTC_INTEGRATE_8974PCN_WP8_002 // algin with CDMA project
/* only for CDMA VzW windows phone */
    uint32_t    htc_rtn_type;
#endif // HTC_INTEGRATE_8974PCN_WP8_002

#if 1 //FEATURE_8974_WP8_SUPPORT // HTC_INTEGRATE_8974PCN_WP8_003 // algin with CDMA project
/* only for CDMA VzW windows phone */
    uint8_t     VerSW[32];
    uint8_t     VerFM[16];
    uint8_t     VerHW[8];
    uint8_t     verFMLong[24];
    uint8_t     modelName[32];
    uint8_t     VerTemp[16];
#endif // HTC_INTEGRATE_8974PCN_WP8_003

    uint8_t     RCMS_Name[64];
};

#endif /* end of _HTC_RADIO_SMEM_H */
