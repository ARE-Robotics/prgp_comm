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
 *  @file prgp_piswarmcom.cpp
 *  @brief The source file for prgp_piswarmcom package.
 *  @details The prgp_piswarmcom package is created and tested by Chengqing Liu.
 *  @version 1.0
 *  @author Chengqing Liu
 *  @date 01 July 2015
 *  @copyright BSD License.
 */

#include <prgp_piswarmcom/prgp_piswarmcom.h>

/** Callback function to receive the command from the ARDrone by topic piswarm_com.
 *  Getting the string from the piswarm_com topic and comparing it with the "b" to
 *  set the returnFlag.
 */
void ardroneCmdRev(const std_msgs::StringConstPtr str)
{
  ROS_INFO_STREAM(*str);
  ROS_INFO("%s\n", str->data.c_str());
  std::string k = str->data.substr(0, 1);
  ROS_INFO("%s\n", k.c_str());
  if ("b" == str->data.c_str())
  {
    returnFlag = true;
  }
}

/** Send command to Pi-Swarm by the bluetooth port.
 *  @param[in] msg_w The input char variable means the symbol to send.
 *  The function will compare the symbol sent and the symbol received to make sure
 *  the command is received.
 */
void sendCmdToPiSwarm(char msg_w)
{
  char msg_r;

  write(bt_port, &msg_w, 1);
  printf("%c\n", msg_w);

  ndPause.sleep(); //Wait for 2 seconds to be published
  msg_r = '\0';
  read(bt_port, &msg_r, 1);
  printf("%c\n", msg_r);

  if (msg_r == msg_w)
  {
    printf("Command is successfully sent to Pi-Swarm\n");
    if (returnFlag == false)
    {
      searchFlag = true;
      startFlag = false;
    }
    else
    {
      returnFlag = false;
      beaconNotified = false;
    }
  }
}

/** Get the command from the Pi-Swarm by the bluetooth port.
 *  The fuction get the command from the home beacon, which receive the command from the
 *  Pi-Swarm through RF communication.
 */
void revCmdFromPiSwarm()
{
  char msg_w = 'q';
  char msg_rr;

  write(bt_port, &msg_w, 1);
  printf("%c\n", msg_w);

  ndPause.sleep(); //Wait for 2 seconds to be published
  msg_rr = '\0';
  read(bt_port, &msg_rr, 1);
  printf("%c\n", msg_rr);

  if (msg_rr == 'r')
  {
    printf("Command is successfully sent to AR.Drone to search black_roundel tag\n");
    beaconFlag = true;
    searchFlag = false;
    target_tag = 'r';
    ndPause.sleep();
    ndPause.sleep();
    ndPause.sleep();
  }
  else if (msg_rr == 'c')
  {
    printf("Command is successfully sent to AR.Drone to search COCARDE tag\n");
    beaconFlag = true;
    searchFlag = false;
    target_tag = 'c';
  }
  else if (msg_rr == 'm')
  {
    printf("Command is successfully sent to AR.Drone to search mix type tag\n");
    beaconFlag = true;
    searchFlag = false;
    target_tag = 'm';
  }
}

/** Send the command to the home beacon by the bluetooth port.
 *  The function send the command to the home beacon and compare the symbol
 *  sent and the symbol received to make sure the command is received.
 */
void sendCmdToHomeBeacon(char msg_w)
{
  char msg_r;

  write(bt_port, &msg_w, 1);
  printf("%c\n", msg_w);

  ndPause.sleep(); //Wait for 2 seconds to be published
  msg_r = '\0';
  read(bt_port, &msg_r, 1);
  printf("%c\n", msg_r);

  if (msg_r == msg_w)
  {
    printf("Command is successfully sent to Home beacon\n");
    sendToHome = false;
    beaconNotified = true;
  }
}

/** Send the command to the target beacon by the bluetooth port.
 *  The function send the command to the target beacon and compare the symbol
 *  sent and the symbol received to make sure the command is received.
 */
void sendCmdToTargetBeacon(char msg_w)
{
  char msg_r;

  write(bt_t_port, &msg_w, 1);
  printf("%c\n", msg_w);

  ndPause.sleep(); //Wait for 2 seconds to be published
  msg_r = '\0';
  read(bt_t_port, &msg_r, 1);
  printf("%c\n", msg_r);

  if (msg_r == msg_w)
  {
    printf("Command is successfully sent to Target beacon\n");
    sendToTarget = false;
    sendToHome = true;
  }
}

/** Check the bluetooth port availability
 *  Check the bluetooth port from PC to home beacon and the port from PC to target beacon
 *  . If the port is not available, try to open it again.
 */
void portChecking()
{
  if (bt_port == -1)
  {
    ROS_INFO("Error opening the bt_port"); //Inform user on the terminal
    bt_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
  }
  else
  {
    ROS_INFO("bt_port is open"); //Inform user on the terminal
  }
  printf("%d\n", bt_port);

  if (bt_t_port == -1)
  {
    ROS_INFO("Error opening the bt_t_port"); //Inform user on the terminal
    bt_t_port = open("/dev/rfcomm1", O_RDWR | O_NOCTTY | O_NDELAY);
  }
  else
  {
    ROS_INFO("bt_t_port is open"); //Inform user on the terminal
  }
  printf("%d\n", bt_t_port);
}

/** Main function and running loop for the prgp_piwarmcom package.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "prgp_piswarmcom");
  ros::NodeHandle ndh_; /**< ROS node handle */
  ndPause = ros::Duration(1, 0);

  cmdPiPub = ndh_.advertise<std_msgs::String>("piswarm_com", 1);
  cmdPiSub = ndh_.subscribe("piswarm_com", 1, ardroneCmdRev);

  bt_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
  bt_t_port = open("/dev/rfcomm1", O_RDWR | O_NOCTTY | O_NDELAY);

  //set_interface_attribs (bt_h_port, B115200, 0);

  while (ros::ok)
  {
    portChecking();
    if (startFlag == true)
    {
      sendCmdToPiSwarm('s');
      ROS_INFO("PRGP: Send command to start the Pi-Swarm by serial communication");
    }

    if (searchFlag == true) //Pi-Swarm search the target
    {
      revCmdFromPiSwarm();
    }
    if (beaconFlag == true)
    {
      sprintf(&c_P[0], "%c", target_tag);
      //c_P = target_tag;
      s_P.data = c_P.c_str();
      cmdPiPub.publish(s_P);
      target_tag = '\0';
      if (sendToTarget == true)
      {
        sendCmdToTargetBeacon('x'); // to Target beacon
      }
      if (sendToHome == true)
      {
        sendCmdToHomeBeacon('o'); // to Home beacon
      }
    }
    if ((returnFlag == true) && (beaconNotified == true))
    {
      sendCmdToPiSwarm('b');
    }

    ros::spinOnce();
  }
  close(bt_port);
  close(bt_t_port);

  return 0;
}

