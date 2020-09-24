/******************************************************************************/

/**
 *  \file    g4_UtilAssert.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Assert definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_UTIL_ASSERT_H__
#define __G4_UTIL_ASSERT_H__

#include <stdint.h>

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/
#ifdef __PC_LINT__
#define g4_UtilAssert(_expr)    \
{   \
    /*lint -e{506,774}*/  \
    if(!(_expr)) \
    { \
        (void) (_expr); /* trigger lint to generate warning on assert with side effects */ \
//        g4_UtilAssert_Failed(__FILE__, __LINE__, #_expr);   \
    } \
}
#elif defined DEBUG
//# define g4_UtilAssert(_expr) ((_expr) ? (void) 0 : g4_UtilAssert_Failed(__FILE__, __LINE__, #_expr))
#else
# define g4_UtilAssert(_expr) (void)(_expr)
#endif

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


#if defined DEBUG || defined __PC_LINT__
/**
 *  \brief  Assertion failed function, prints information
 */
//void g4_UtilAssert_Failed(char *fn, uint32_t line, char *expr);
#endif

#endif /* defined __G4_UTIL_STDIO_H__ */

