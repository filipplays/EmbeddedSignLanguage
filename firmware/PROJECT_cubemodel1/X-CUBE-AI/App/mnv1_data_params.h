/**
  ******************************************************************************
  * @file    mnv1_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Tue Dec 26 15:23:57 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef MNV1_DATA_PARAMS_H
#define MNV1_DATA_PARAMS_H
#pragma once

#include "ai_platform.h"

/*
#define AI_MNV1_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_mnv1_data_weights_params[1]))
*/

#define AI_MNV1_DATA_CONFIG               (NULL)


#define AI_MNV1_DATA_ACTIVATIONS_SIZES \
  { 82648, }
#define AI_MNV1_DATA_ACTIVATIONS_SIZE     (82648)
#define AI_MNV1_DATA_ACTIVATIONS_COUNT    (1)
#define AI_MNV1_DATA_ACTIVATION_1_SIZE    (82648)



#define AI_MNV1_DATA_WEIGHTS_SIZES \
  { 455776, }
#define AI_MNV1_DATA_WEIGHTS_SIZE         (455776)
#define AI_MNV1_DATA_WEIGHTS_COUNT        (1)
#define AI_MNV1_DATA_WEIGHT_1_SIZE        (455776)



#define AI_MNV1_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_mnv1_activations_table[1])

extern ai_handle g_mnv1_activations_table[1 + 2];



#define AI_MNV1_DATA_WEIGHTS_TABLE_GET() \
  (&g_mnv1_weights_table[1])

extern ai_handle g_mnv1_weights_table[1 + 2];


#endif    /* MNV1_DATA_PARAMS_H */
