/********************************************************************/
/**
 * @file	tl_log.h
 * @brief	log functions
 * @copyright	Nuvoton Technology Corporation Japan.
 */
/********************************************************************/

#ifndef H_TL_LOG
#define H_TL_LOG

/* #undef _CPF */

#ifdef _CPF

#include "CameraLocal.h"

/* Information log */
#define TL_LGI(...)	CAM_INF(__VA_ARGS__)
/* Error log */
#define TL_LGE(...)	CAM_ERR(__VA_ARGS__)

#else	/* _CPF */

#include <stdio.h>

/* Information log */
#define TL_LGI(fmt, ...)
//#define TL_LGI(fmt, ...)	(fprintf(stderr, "[%s:%d] \n" fmt, __func__, __LINE__, ## __VA_ARGS__))
/* Error log */
#define TL_LGE(fmt, ...)	(fprintf(stderr, "[%s:%d] \n" fmt, __func__, __LINE__, ## __VA_ARGS__))

#endif	/* _CPF */


#endif	/* H_TL_LOG */
