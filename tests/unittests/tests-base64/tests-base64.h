/*
 * Copyright (C) 2015 Martin Landsmann
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup  unittests
 * @{
 *
 * @file        tests-base64.h
 * @brief       Unittests for the ``base64`` module
 *
 * @author      Martin Landsmann <Martin.Landsmann@HAW-Hamburg.de>
 */
#ifndef __TESTS_BASE64_H_
#define __TESTS_BASE64_H_
#include "embUnit/embUnit.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
*  @brief   The entry point of this test suite.
*/
void tests_base64(void);

/**
 * @brief   Generates tests for base64
 *
 * @return  embUnit tests if successful, NULL if not.
 */
Test *tests_base64_tests(void);

#ifdef __cplusplus
}
#endif

#endif /* __TESTS_BASE64_H_ */
/** @} */
