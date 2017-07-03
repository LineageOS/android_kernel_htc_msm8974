#ifndef _MSM_RMNET_H_
#define _MSM_RMNET_H_

#define RMNET_MODE_NONE     (0x00)
#define RMNET_MODE_LLP_ETH  (0x01)
#define RMNET_MODE_LLP_IP   (0x02)
#define RMNET_MODE_QOS      (0x04)
#define RMNET_MODE_MASK     (RMNET_MODE_LLP_ETH | \
			     RMNET_MODE_LLP_IP  | \
			     RMNET_MODE_QOS)

#define RMNET_IS_MODE_QOS(mode)  \
	((mode & RMNET_MODE_QOS) == RMNET_MODE_QOS)
#define RMNET_IS_MODE_IP(mode)   \
	((mode & RMNET_MODE_LLP_IP) == RMNET_MODE_LLP_IP)

enum rmnet_ioctl_cmds_e {
	RMNET_IOCTL_SET_LLP_ETHERNET = 0x000089F1, 
	RMNET_IOCTL_SET_LLP_IP       = 0x000089F2, 
	RMNET_IOCTL_GET_LLP          = 0x000089F3, 
	RMNET_IOCTL_SET_QOS_ENABLE   = 0x000089F4, 
	RMNET_IOCTL_SET_QOS_DISABLE  = 0x000089F5, 
	RMNET_IOCTL_GET_QOS          = 0x000089F6, 
	RMNET_IOCTL_GET_OPMODE       = 0x000089F7, 
	RMNET_IOCTL_OPEN             = 0x000089F8, 
	RMNET_IOCTL_CLOSE            = 0x000089F9, 
	RMNET_IOCTL_FLOW_ENABLE	     = 0x000089FA, 
	RMNET_IOCTL_FLOW_DISABLE     = 0x000089FB, 
        RMNET_IOCTL_FLOW_SET_HNDL    = 0x000089FC, 
        RMNET_IOCTL_EXTENDED         = 0x000089FD, 
	RMNET_IOCTL_MAX
};

enum rmnet_ioctl_extended_cmds_e {
        RMNET_IOCTL_GET_SUPPORTED_FEATURES     = 0x0000,   
        RMNET_IOCTL_SET_MRU                    = 0x0001,   
        RMNET_IOCTL_GET_MRU                    = 0x0002,   
        RMNET_IOCTL_GET_EPID                   = 0x0003,   
        RMNET_IOCTL_GET_DRIVER_NAME            = 0x0004,   
        RMNET_IOCTL_ADD_MUX_CHANNEL            = 0x0005,   
        RMNET_IOCTL_SET_EGRESS_DATA_FORMAT     = 0x0006,   
        RMNET_IOCTL_SET_INGRESS_DATA_FORMAT    = 0x0007,   
        RMNET_IOCTL_SET_AGGREGATION_COUNT      = 0x0008,   
        RMNET_IOCTL_GET_AGGREGATION_COUNT      = 0x0009,   
        RMNET_IOCTL_SET_AGGREGATION_SIZE       = 0x000A,   
        RMNET_IOCTL_GET_AGGREGATION_SIZE       = 0x000B,   
        RMNET_IOCTL_FLOW_CONTROL               = 0x000C,   
        RMNET_IOCTL_GET_DFLT_CONTROL_CHANNEL   = 0x000D,   
        RMNET_IOCTL_GET_HWSW_MAP               = 0x000E,   
        RMNET_IOCTL_SET_RX_HEADROOM            = 0x000F,   
        RMNET_IOCTL_GET_EP_PAIR                = 0x0010,   
        RMNET_IOCTL_SET_QOS_VERSION            = 0x0011,   
        RMNET_IOCTL_GET_QOS_VERSION            = 0x0012,   
        RMNET_IOCTL_GET_SUPPORTED_QOS_MODES    = 0x0013,   
        RMNET_IOCTL_SET_SLEEP_STATE            = 0x0014,   
        RMNET_IOCTL_SET_XLAT_DEV_INFO          = 0x0015,   
        RMNET_IOCTL_EXTENDED_MAX               = 0x0016
};

#define RMNET_IOCTL_FEAT_NOTIFY_MUX_CHANNEL              (1<<0)
#define RMNET_IOCTL_FEAT_SET_EGRESS_DATA_FORMAT          (1<<1)
#define RMNET_IOCTL_FEAT_SET_INGRESS_DATA_FORMAT         (1<<2)
#define RMNET_IOCTL_FEAT_SET_AGGREGATION_COUNT           (1<<3)
#define RMNET_IOCTL_FEAT_GET_AGGREGATION_COUNT           (1<<4)
#define RMNET_IOCTL_FEAT_SET_AGGREGATION_SIZE            (1<<5)
#define RMNET_IOCTL_FEAT_GET_AGGREGATION_SIZE            (1<<6)
#define RMNET_IOCTL_FEAT_FLOW_CONTROL                    (1<<7)
#define RMNET_IOCTL_FEAT_GET_DFLT_CONTROL_CHANNEL        (1<<8)
#define RMNET_IOCTL_FEAT_GET_HWSW_MAP                    (1<<9)

#define RMNET_IOCTL_EGRESS_FORMAT_MAP                  (1<<1)
#define RMNET_IOCTL_EGRESS_FORMAT_AGGREGATION          (1<<2)
#define RMNET_IOCTL_EGRESS_FORMAT_MUXING               (1<<3)
#define RMNET_IOCTL_EGRESS_FORMAT_CHECKSUM             (1<<4)

#define RMNET_IOCTL_INGRESS_FORMAT_MAP                 (1<<1)
#define RMNET_IOCTL_INGRESS_FORMAT_DEAGGREGATION       (1<<2)
#define RMNET_IOCTL_INGRESS_FORMAT_DEMUXING            (1<<3)
#define RMNET_IOCTL_INGRESS_FORMAT_CHECKSUM            (1<<4)
#define RMNET_IOCTL_INGRESS_FORMAT_AGG_DATA            (1<<5)

#ifndef IFNAMSIZ
#define IFNAMSIZ 16
#endif

struct rmnet_ioctl_extended_s {
        uint32_t   extended_ioctl;
        union {
                uint32_t data; 

                int8_t if_name[IFNAMSIZ];

                
                struct {
                        uint32_t  mux_id;
                        int8_t    vchannel_name[IFNAMSIZ];
                } rmnet_mux_val;

                
                struct {
                        uint8_t   flow_mode;
                        uint8_t   mux_id;
                } flow_control_prop;

                
                struct {
                        uint32_t   consumer_pipe_num;
                        uint32_t   producer_pipe_num;
                } ipa_ep_pair;

                struct {
                        uint32_t __data; 
                        uint32_t agg_size;
                        uint32_t agg_count;
                } ingress_format;
        } u;
};

struct rmnet_ioctl_data_s {
        union {
                uint32_t        operation_mode;
                uint32_t        tcm_handle;
        } u;
};

#define RMNET_IOCTL_QOS_MODE_6   (1<<0)
#define RMNET_IOCTL_QOS_MODE_8   (1<<1)

#define QMI_QOS_HDR_S  __attribute((__packed__)) qmi_qos_hdr_s
struct QMI_QOS_HDR_S {
	unsigned char    version;
	unsigned char    flags;
	unsigned long    flow_id;
};

struct qmi_qos_hdr8_s {
        struct QMI_QOS_HDR_S   hdr;
        uint8_t                reserved[2];
} __attribute((__packed__));

#endif 
