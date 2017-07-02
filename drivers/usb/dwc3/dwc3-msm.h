#ifndef __DWC3_MSM_H__
#define __DWC3_MSM_H__

#include <linux/usb/msm_hsusb.h>
#include <linux/qpnp/qpnp-adc.h>

#include "core.h"
#include "dwc3_otg.h"

#define DBM_MAX_EPS             4

struct dwc3_msm {
        struct device *dev;
        void __iomem *base;
        struct resource *io_res;
        struct platform_device  *dwc3;
        int dbm_num_eps;
        u8 ep_num_mapping[DBM_MAX_EPS];
        const struct usb_ep_ops *original_ep_ops[DWC3_ENDPOINTS_NUM];
        struct list_head req_complete_list;
        struct clk              *xo_clk;
        struct clk              *ref_clk;
        struct clk              *core_clk;
        struct clk              *iface_clk;
        struct clk              *sleep_clk;
        struct clk              *hsphy_sleep_clk;
        struct clk              *utmi_clk;
        unsigned int            utmi_clk_rate;
        struct clk              *utmi_clk_src;
        struct regulator        *hsusb_3p3;
        struct regulator        *hsusb_1p8;
        struct regulator        *hsusb_vddcx;
        struct regulator        *ssusb_1p8;
        struct regulator        *ssusb_vddcx;
        struct regulator        *dwc3_gdsc;
        struct wake_lock        cable_detect_wlock;

        
        struct regulator        *vbus_otg;
        struct dwc3_ext_xceiv   ext_xceiv;
        bool                    resume_pending;
        atomic_t                pm_suspended;
        atomic_t                in_lpm;
        int                     hs_phy_irq;
        int                     hsphy_init_seq;
        int                     deemphasis_val;
        bool                    lpm_irq_seen;
        struct delayed_work     resume_work;
        struct work_struct      restart_usb_work;
        struct work_struct      usb_block_reset_work;
        bool                    in_restart;
        struct dwc3_charger     charger;
        struct usb_phy          *otg_xceiv;
        struct delayed_work     chg_work;
        enum usb_chg_state      chg_state;
        int                     pmic_id_irq;
        struct work_struct      id_work;
        struct qpnp_adc_tm_btm_param    adc_param;
        struct qpnp_adc_tm_chip *adc_tm_dev;
        struct delayed_work     init_adc_work;
        bool                    id_adc_detect;
        struct qpnp_vadc_chip   *vadc_dev;
        u8                      dcd_retries;
        u32                     bus_perf_client;
        struct msm_bus_scale_pdata      *bus_scale_table;
        struct power_supply     usb_psy;
        struct power_supply     *ext_vbus_psy;
        unsigned int            online;
        unsigned int            host_mode;
        unsigned int            voltage_max;
        unsigned int            current_max;
        unsigned int            vdd_no_vol_level;
        unsigned int            vdd_low_vol_level;
        unsigned int            vdd_high_vol_level;
        unsigned int            tx_fifo_size;
        unsigned int            qdss_tx_fifo_size;
        bool                    vbus_active;
        bool                    ext_inuse;
        enum dwc3_id_state      id_state;
        unsigned long           lpm_flags;
#define MDWC3_PHY_REF_AND_CORECLK_OFF   BIT(0)
#define MDWC3_TCXO_SHUTDOWN             BIT(1)
#define MDWC3_ASYNC_IRQ_WAKE_CAPABILITY BIT(2)

        u32 qscratch_ctl_val;
        dev_t ext_chg_dev;
        struct cdev ext_chg_cdev;
        struct class *ext_chg_class;
        struct device *ext_chg_device;
        bool ext_chg_opened;
        bool ext_chg_active;
        struct completion ext_chg_wait;
        struct qpnp_vadc_chip *vadc_chip;
};


#endif

