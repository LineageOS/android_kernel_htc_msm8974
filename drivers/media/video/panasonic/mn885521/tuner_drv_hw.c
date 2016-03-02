/******************************************************************************
 *
 *  file name       : tuner_drv_hw.c
 *  brief note      : HW Control Layer of Tmm Tuner Driver
 *
 *  creation data   : 2011.07.25
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 1720                       $ Revision of Last commit
 *  $Date:: 2013-05-08 22:02:48 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/07/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/

/******************************************************************************/
/* include                                                                    */
/******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include "tuner_drv.h"

#include <linux/mm.h>
#include <linux/vmalloc.h>

/******************************************************************************
 * function
 ******************************************************************************/
int tuner_drv_hw_access( unsigned int uCommand, TUNER_DATA_RW *data,
                         unsigned short param_len );

/******************************************************************************
 * code area
 ******************************************************************************/
/******************************************************************************
 *    function:   tuner_drv_hw_access
 *    brief   :   HW access control of a driver
 *                repeat specified count single register access
 *    date    :   2011.08.02
 *    author  :   K.Kitamura(*)
 *    modifier:   K.Okawa(KXD14)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   uCommand             setting command
 *            :   data                 access data
 *            :   param_len            nummber of access data
 *    output  :   none
 ******************************************************************************/
int tuner_drv_hw_access( unsigned int uCommand, TUNER_DATA_RW *data,
                         unsigned short param_len )
{
    int                ret;
    struct i2c_adapter *adap;
    struct i2c_msg     msgs[2];
    unsigned short     addr;
    unsigned short     flags;
    unsigned char      buf[TUNER_I2C_MSG_DATA_NUM];
    //unsigned char      *buf_all;
    unsigned char      read_data;
    //unsigned short     cnt;
    unsigned char      ena_data;
    unsigned char      write_data;
    unsigned short     loop_cnt;


    /* argument check */
    if( data == NULL )
    {
        DEBUG_PRINT("tuner_drv_hw_access -1- ");

        TRACE();
        return -EINVAL;
    }

    /* get i2c adapter */
    adap = i2c_get_adapter( TUNER_CONFIG_I2C_BUSNUM );
    if( adap == NULL )
    {
	DEBUG_PRINT("tuner_drv_hw_access -2- ");
        TRACE();
        return -EINVAL;
    }

    /* initialize */
    memset( msgs, 0x00, sizeof(struct i2c_msg) * 2 );
    ena_data = 0x00;
    flags = 0;
#if 0
    /* 一括書き込み制御 */
    if( param_len >= TUNER_I2C_WRITE_ALL_NUM )
    {
        /* 領域獲得 */
        buf_all  = ( unsigned char * )vmalloc( param_len + 1 );

        if( buf_all  == NULL )
        {
            TRACE();
            i2c_put_adapter( adap );
            return -EINVAL;
        }

        /* メモリ初期化 */
        memset( buf_all,
                0x00,
                param_len + 1 );

        /* 転送データ作成 */
        msgs[ 0 ].addr  = data[ 0 ].slave_adr;
        msgs[ 0 ].flags = 0;
        msgs[ 0 ].len   = ( unsigned short )( param_len + 1 );
        msgs[ 0 ].buf   = buf_all;

        *buf_all = ( unsigned char )data[ 0 ].adr;

        DEBUG_PRINT("//Write slaveAdr, adr                           || 0x%02x, 0x%2x",
                    data[ 0 ].slave_adr, buf_all[ 0 ]);
        for( loop_cnt = 0; loop_cnt < param_len; loop_cnt++ )
        {
            buf_all[ loop_cnt + 1 ] =  ( unsigned char )data[ loop_cnt ].param;
            DEBUG_PRINT("//Write                                  w_data || 0x%02x",
                        buf_all[ loop_cnt + 1 ]);
        }

        /* データ転送 */
        ret = i2c_transfer( adap, msgs, TUNER_W_MSGNUM );

        /* メモリ開放 */
        vfree( buf_all );

        if( ret < 0 )
        {
            TRACE();
            i2c_put_adapter( adap );
            return -EINVAL;
        }

        i2c_put_adapter( adap );
        return 0;
    }
#endif
    /* access loop */
    for( loop_cnt = 0; loop_cnt < param_len; loop_cnt++ ) {
        /* initialize i2c messages */
        memset( msgs, 0x00, sizeof(struct i2c_msg) * 2 );
        ena_data = 0x00;

        /* command detect switch */
        switch ( uCommand ) {
             case TUNER_IOCTL_VALGET:	/* read access */
                addr   = data[ loop_cnt ].slave_adr;
                flags  = I2C_M_RD;
                buf[ 0 ] = (unsigned char)data[ loop_cnt ].adr;
                buf[ 1 ] = 0;
                break;
            case TUNER_IOCTL_VALSET:	/* write access */
                addr   = data[ loop_cnt ].slave_adr;
                flags  = 0;
                buf[ 0 ] = (unsigned char)data[ loop_cnt ].adr;
                buf[ 1 ] = (unsigned char)data[ loop_cnt ].param;
                break;
            default:
                i2c_put_adapter( adap );
                return -EINVAL;
        }

        if( flags != I2C_M_RD ) {
        	 /* bit modify write */
        	 if( !(( data[ loop_cnt ].sbit == 0 ) && ( data[ loop_cnt ].ebit == 7 ))) {
            	/* specified start/end bit position mode */
                /* TODO: enabit */
                if(( data[ loop_cnt ].sbit == TUNER_SET_ENADATA )
                && ( data[ loop_cnt ].ebit == TUNER_SET_ENADATA ))
                {
                    ena_data = ( unsigned char )data[ loop_cnt ].enabit; 
                }
                else
                {
                    /* calculate enable bit mask */
                	ena_data = (unsigned char)(((1U << (1+data[loop_cnt].ebit-data[loop_cnt].sbit))-1) << data[loop_cnt].sbit);
#if 0
                    for( cnt = 0; cnt < TUNER_CHARNUM_MAX; cnt++ )
                    {
                        if(( cnt >= data[ loop_cnt ].sbit )
                            && ( cnt <= data[ loop_cnt ].ebit ))
                        {
                            ena_data |= ( unsigned char )( 1 << cnt );
                        }
                    }
#endif
                }
                
                if( ena_data != 0xFF )
                {
                    /* read a current value */
                    msgs[ 0 ].addr  = addr;
                    msgs[ 0 ].flags = 0;
                    msgs[ 0 ].len   = TUNER_R_MSGLEN;
                    msgs[ 0 ].buf   = &buf[ 0 ];
                    msgs[ 1 ].addr  = addr;
                    msgs[ 1 ].flags = I2C_M_RD;
                    msgs[ 1 ].len   = TUNER_R_MSGLEN;
                    msgs[ 1 ].buf   = &read_data;

                    ret = i2c_transfer( adap, msgs, TUNER_R_MSGNUM );
                    if( ret < 0 )
                    {
			DEBUG_PRINT("tuner_drv_hw_access -3- ");
                        TRACE();
                        i2c_put_adapter( adap );
                        return -EINVAL;
                    }

                    /* initialize i2c message */
                    memset( msgs, 0x00, sizeof( struct i2c_msg ) * 2 );

                    /* clear bits of write position */
                    read_data  &= ( unsigned char )( ~ena_data );
                    /* construct a write value */
                    write_data = ( unsigned char )( ena_data & data[ loop_cnt ].param );
                    buf[ 1 ] = ( unsigned char )( write_data | read_data );
                }
            }

            //DEBUG_PRINT("ioctl(W) slv:0x%02x adr:0x%02x (single) 0x%02x (RMW:0x%02x WDAT:0x%02x)",
            //                    addr, buf[ 0 ], data[ loop_cnt ].param, read_data,  buf[ 1 ]);

            msgs[ 0 ].addr  = addr;
            msgs[ 0 ].flags = flags;
            msgs[ 0 ].len   = TUNER_W_MSGLEN;
            msgs[ 0 ].buf   = buf;


            ret = i2c_transfer( adap, msgs, TUNER_W_MSGNUM );
            if( ret < 0 )
            {
		DEBUG_PRINT("tuner_drv_hw_access -4- ");
                TRACE();
                i2c_put_adapter( adap );
                return -EINVAL;
            }
        } else {
        	/* write register */
            msgs[ 0 ].addr  = addr;
            msgs[ 0 ].flags = 0;
            msgs[ 0 ].len   = TUNER_R_MSGLEN;
            msgs[ 0 ].buf   = &buf[ 0 ];
            msgs[ 1 ].addr  = addr;
            msgs[ 1 ].flags = flags;
            msgs[ 1 ].len   = TUNER_R_MSGLEN;
            msgs[ 1 ].buf   = &buf[ 1 ];

//            DEBUG_PRINT("ioctl(read(Pre)) slv:0x%02x adr:0x%02x (single)",
//								addr, buf[ 0 ]);
            ret = i2c_transfer( adap, msgs, TUNER_R_MSGNUM );
            if( ret < 0 )
            {
		DEBUG_PRINT("tuner_drv_hw_access, ret = %x -5- ", ret);
                DEBUG_PRINT("ioctl(read(Pre)) slv:0x%02x adr:0x%02x (single)",
                                                              addr, buf[ 0 ]);

                TRACE();
                i2c_put_adapter( adap );
                return -EINVAL;
            }

            /* return read val. */
            data[ loop_cnt ].param = buf[ 1 ];
            //DEBUG_PRINT("ioctl(R) slv:0x%02x adr:0x%02x (single) 0x%02x (RETURN:0x%02x)",
            //                    addr, buf[ 0 ], buf[1], data[ loop_cnt ].param);

        }
    }
    i2c_put_adapter( adap );
    return 0;
}

/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
