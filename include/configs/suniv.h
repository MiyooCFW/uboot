/*
 * Configuration settings for new Allwinner F-series (suniv) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * Include common sunxi configuration where most the settings are
 */
#include <configs/sunxi-common.h>

#define CONFIG_SYS_DFU_DATA_BUF_SIZE (512*1024)

#ifdef CONFIG_VIDEO
#define CONFIG_MIYOO
#define CONFIG_VIDEO_LOGO
#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SOURCE
#endif

#endif /* __CONFIG_H */
