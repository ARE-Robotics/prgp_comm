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
 *  @file prgp_piswarmcom.h
 *  @brief The header file for prgp_piswarmcom package.
 *  @details The prgp_piswarmcom package is created and tested by Chengqing Liu. The package is updated to
 *  class style based on its initial package by Chengqing Liu on 19 Aug., 2015. The package is responsible
 *  for the bluetooth communication between the home beacon and prgp_piswarmcom package, the bluetooth
 *  communication between the target beacon and prgp_piswarmcom package, and the ros topic communication
 *  between prgp_piswarmcom package and prgp_ardrone package.
 *  @version 1.1
 *  @author Chengqing Liu
 *  @date 01 July 2015
 *  @copyright BSD License.
 */

#ifndef PRGP_PISWARMCOM_INCLUDE_PRGP_PISWARMCOM_PRGP_PISWARMCOM_H_
#define PRGP_PISWARMCOM_INCLUDE_PRGP_PISWARMCOM_PRGP_PISWARMCOM_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include "std_msgs/Duration.h"

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

//#define START_DRONE_PKG_IN_CODE  //Open to starting the AR.Drone packages from the code
//#define DEBUG //open the information printing for debugging

/** The main class for the prgp_piswarmcom package.
 */
class PRGPPiSwarmCom
{
private:

  ros::NodeHandle ndh_; /**< ROS node handle */
  ros::WallTime begin_time;
  ros::Duration ndPause; /**< The duration to pause the node. */
  ros::Publisher cmdPiPub; /**< Publisher for sending the command to Pi-Swarm by piswarm_com. */
  ros::Subscriber cmdPiSub; /**< Sublisher for sending the command to Pi-Swarm by piswarm_com. */

  std_msgs::String s_P; /**< Message for publishing to the AR.Drone by piswarm_come. */
  std::string c_P;

  bool targetonFlag;/**< True means sending the command to turn on the target beacon. */
  bool startFlag; /**< True means sending the start command to the Pi-Swarm. */
  bool searchFlag; /**< The value will be true when the Pi-Swarm starts to search the target. */
  bool recruitFlag; /**< The value will be true when the target is found. */
  bool returnFlag; /**< The value will be true when the return command is received from AR.Drone. */
  bool sendToTarget; /**< The value will be true before sending the command to the target beacon. */
  bool sendToHome; /**< The value will be true before sending the command to the home beacon. */
  bool beaconNotified; /**< The value will be true after the beacon received the command. */

  char target_tag; /**< The char variable to store the target tag type. */

  int16_t bt_port; /**< The bluetooth port for the home beacon. */
  int16_t bt_t_port; /**< The bluetooth port for the target beacon. */
  double pi_starttime; /**< Record the time when starting the Pi-Swarm. */
  double pi_returntime; /**< Record the time when returning the Pi-Swarm. */
  double pi_targettime; /**< Record the time when Pi-Swarm reaching the target. */
  double home_ontime; /**< Record the time when turning on the home beacon flashing. */
  double target_ontime; /**< Record the time when turning on the target beacon flashing. */
  double target_offtime; /**< Record the time when turning off the target beacon flashing. */
  bool face_change; /**< For face changing in the project UI. */

public:
  PRGPPiSwarmCom(void);
  ~PRGPPiSwarmCom(void);

  bool run();

  void ardroneCmdRevCb(const std_msgs::StringConstPtr str);
  bool sendCmdToPiSwarm(char msg_w);
  bool revCmdFromPiSwarm();
  bool sendCmdToHomeBeacon(char msg_w);
  bool sendCmdToTargetBeacon(char msg_w);
  bool openBTPort();
  bool prgp_ui();
};

#endif /* PRGP_PISWARMCOM_INCLUDE_PRGP_PISWARMCOM_PRGP_PISWARMCOM_H_ */
