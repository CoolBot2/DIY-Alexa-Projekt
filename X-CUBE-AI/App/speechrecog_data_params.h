/**
  ******************************************************************************
  * @file    speechrecog_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-12-29T14:55:38+0100
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef SPEECHRECOG_DATA_PARAMS_H
#define SPEECHRECOG_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_SPEECHRECOG_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_speechrecog_data_weights_params[1]))
*/

#define AI_SPEECHRECOG_DATA_CONFIG               (NULL)


#define AI_SPEECHRECOG_DATA_ACTIVATIONS_SIZES \
  { 22560, }
#define AI_SPEECHRECOG_DATA_ACTIVATIONS_SIZE     (22560)
#define AI_SPEECHRECOG_DATA_ACTIVATIONS_COUNT    (1)
#define AI_SPEECHRECOG_DATA_ACTIVATION_1_SIZE    (22560)



#define AI_SPEECHRECOG_DATA_WEIGHTS_SIZES \
  { 23896, }
#define AI_SPEECHRECOG_DATA_WEIGHTS_SIZE         (23896)
#define AI_SPEECHRECOG_DATA_WEIGHTS_COUNT        (1)
#define AI_SPEECHRECOG_DATA_WEIGHT_1_SIZE        (23896)



#define AI_SPEECHRECOG_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_speechrecog_activations_table[1])

extern ai_handle g_speechrecog_activations_table[1 + 2];



#define AI_SPEECHRECOG_DATA_WEIGHTS_TABLE_GET() \
  (&g_speechrecog_weights_table[1])

extern ai_handle g_speechrecog_weights_table[1 + 2];


#endif    /* SPEECHRECOG_DATA_PARAMS_H */
