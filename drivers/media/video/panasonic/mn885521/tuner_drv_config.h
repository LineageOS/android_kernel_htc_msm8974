/******************************************************************************
 *
 *  file name       : tuner_drv_config.h
 *  brief note      : Driver Config Header
 *
 *  creation data   : 2011.08.28
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 1923                       $ Revision of Last commit
 *  $Date:: 2013-11-06 20:09:04 +0900#$ Date of last commit
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
 * HISTORY      : 2011/08/25    K.Kitamura(*)
 *                001 new creation
 *                2012/10/18	K.Okawa(KXD14)
 *                002 modify for MN88552
 ******************************************************************************/
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/

#ifndef _TUNER_DRV_CONFIG_H
#define _TUNER_DRV_CONFIG_H

/******************************************************************************
 * data
 ******************************************************************************/
#define TUNER_SET_ON                     1       /* setting ON              */
#define TUNER_SET_OFF                    0       /* setting OFF             */

/* device driver file name */
#define TUNER_CONFIG_DRIVER_NAME		"mmtuner_drv"

/* device number */
//#define TUNER_CONFIG_DRV_MAJOR         221 //240      /* MAJOR No.               */
//#define TUNER_CONFIG_DRV_MINOR         0   //200       /* MINOR No.               */

/* compile switch for IRQ */
/* #define TUNER_CONFIG_IRQ_PC_LINUX */

/* IRQ# */
#define TUNER_CONFIG_INT              0x07       /* IRQ No.                 */

/* I2C bus # */
#define TUNER_CONFIG_I2C_BUSNUM       0x03      /* I2C Bus No.             */

/* kernel thread priority  */
#define TUNER_CONFIG_KTH_PRI            95       /* priority (0～99)        */

/* exclusive access control  */
/* #define TUNER_CONFIG_DRV_MULTI              *//* exclusive control enable */

/* interrupt trigger type */
#define TUNER_CONFIG_IRQ_LEVEL  TUNER_SET_ON    /* trigger type = level     */

/* I2C slave address of tuner device register bank */
#define TUNER_SLAVE_ADR_S             0x6C      /* reg. bank: Sub           */
#define TUNER_SLAVE_ADR_M1            0x6D      /* reg. bank: Main#1        */
#define TUNER_SLAVE_ADR_M2            0x6E      /* reg. bank: Main#2        */

#endif/* _TUNER_DRV_CONFIG_H */
/*******************************************************************************
 *              Copyright(c) 2011 Panasonic Co., Ltd.
 ******************************************************************************/
