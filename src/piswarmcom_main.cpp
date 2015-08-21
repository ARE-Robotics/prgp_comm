/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, University of York Robotics Laboratory (YRL).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file piswarmcom_main.cpp
 *  @brief The main file for prgp_piswarmcom package.
 *  @details The prgp_piswarmcom package is created and tested by Chengqing Liu. The package is updated to
 *  class style based on its initial package by Chengqing Liu on 19 Aug., 2015. The package is responsible
 *  for the bluetooth communication between the home beacon and prgp_piswarmcom package, the bluetooth
 *  communication between the target beacon and prgp_piswarmcom package, and the ros topic communication
 *  between prgp_piswarmcom package and prgp_ardrone package.
 *  @version 1.1
 *  @author Chengqing Liu
 *  @date 19 Aug., 2015
 *  @copyright BSD License.
 */

#include <prgp_piswarmcom/prgp_piswarmcom.h>

/** Main function for the prgp_piwarmcom package.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "prgp_piswarmcom");
  ROS_INFO("Start prgp_piswarmcom Node. Hi from ARE 2014/15");

  PRGPPiSwarmCom PRGPPiSwarmCom;
  PRGPPiSwarmCom.run();

  return 0;
}

