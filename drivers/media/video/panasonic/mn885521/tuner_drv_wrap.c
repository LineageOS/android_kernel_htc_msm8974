/******************************************************************************
 *
 *  file name       : tuner_drv_wrap.c
 *  brief note      : The Wrapper Layer for Tmm Tuner Driver
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

/******************************************************************************
 * include
 ******************************************************************************/
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
#include <linux/mm.h>
#include <asm/uaccess.h>
#include "tuner_drv.h"

#ifdef TUNER_CONFIG_IRQ_PC_LINUX
#include "../../../i2c-parport-x/i2c-parport.h"
#endif  /* TUNER_CONFIG_IRQ_PC_LINUX */

/******************************************************************************/
/* function                                                                   */
/******************************************************************************/
int tuner_drv_ctl_power( int data );
int tuner_drv_set_interrupt( void );
void tuner_drv_release_interrupt( void );

/******************************************************************************
 * code area
 ******************************************************************************/
/******************************************************************************
 *    function:   tuner_drv_ctl_power
 *    brief   :   power control of a driver
 *    date    :   2011.08.26
 *    author  :   M.Takahashi(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   data                 setting data
 *    output  :   none
 ******************************************************************************/
int tuner_drv_ctl_power( int data )
{
    /* 電源ON制御 */
    if( data == TUNER_DRV_CTL_POWON )
    {
    }
    /* 電源OFF制御 */
    else
    {
    }

    /* 現状は正常終了を返却 */
    return 0;
}

/******************************************************************************
 *    function:   tuner_drv_set_interrupt
 *    brief   :   interruption registration control of a driver
 *    date    :   2011.08.26
 *    author  :   M.Takahashi(*)
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   pdev
 *    output  :   none
 ******************************************************************************/
int tuner_drv_set_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    int ret;                                     /* 戻り値                    */

    /* 割り込み登録 */
    ret = request_irq( TUNER_CONFIG_INT,         /* 割り込み番号              */
                       tuner_interrupt,          /* コールバック関数          */
                       IRQF_DISABLED,            /* 関数実行中の割込み禁止設定*/
                       "mm_tuner",               /* 表示名称                  */
                       NULL );                   /* デバイスID指定なし        */

    if( ret != 0 )                               /* 登録失敗                  */
    {
        return -1;
    }
#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_set_interrupt( tuner_interrupt );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
    return 0;
}

/******************************************************************************
 *    function:   tuner_drv_release_interrupt
 *    brief   :   interruption registration release control of a driver
 *    date    :   2011.08.26
 *    author  :   M.Takahashi(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_release_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    /* 割り込み登録解除 */
    free_irq( TUNER_CONFIG_INT, NULL );

#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_release_interrupt( NULL );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
}

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
/******************************************************************************
 *    function:   tuner_drv_enable_interrupt
 *    brief   :   interruption registration enable control of a driver
 *    date    :   2011.09.18
 *    author  :   M.Takahashi(*)(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_enable_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    /* 割り込み有効設定 */
    enable_irq( TUNER_INT, NULL );

#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_set_interrupt( tuner_interrupt );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
}

/******************************************************************************
 *    function:   tuner_drv_disable_interrupt
 *    brief   :   interruption registration disable control of a driver
 *    date    :   2011.09.18
 *    author  :   M.Takahashi(*)(*)
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
void tuner_drv_disable_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX 
    /* 割り込み無効設定 */
    disable_irq( TUNER_INT, NULL );

#else  /* TUNER_CONFIG_IRQ_PC_LINUX */
    i2c_release_interrupt( NULL );
#endif /* TUNER_CONFIG_IRQ_PC_LINUX */
}
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */

/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
