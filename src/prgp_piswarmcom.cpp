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
 *  set the returnFlag. std::string k = str->data.substr(0, 1) can be used.
 */
void ardroneCmdRevCb(const std_msgs::StringConstPtr str)
{
  ROS_INFO_STREAM(*str);
  ROS_INFO("%s", str->data.c_str());

  if ("b" == str->data)
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
  char msg_r = '\0';
  int nbytes;
  printf("Start to send Command to the Pi-Swarm\n");

  ioctl(bt_port, FIONREAD, &nbytes);
  printf("Bytes in queue for reading:%d\n", nbytes);
  if (nbytes > 0)
  {
    if (read(bt_port, &msg_r, 1))
    {
      printf("Succefssfully reading from the port\n");
    }
  }
  if (msg_r == msg_w)
  {

  }
  else
  {
    if (write(bt_port, &msg_w, 1))
    {
      printf("Succefssfully writing to the port\n");
    }
  }

  printf("Writing:%c\n", msg_w);
  printf("Reading:%c\n", msg_r);

  if (msg_r == msg_w)
  {
    printf("Command is successfully sent to Pi-Swarm\n");
    if (returnFlag == false)
    {
      searchFlag = true;
      startFlag = false;
      tcflush(bt_port, TCIFLUSH);
    }
    else
    {
      returnFlag = false;
      beaconNotified = false;

      tcflush(bt_port, TCIFLUSH);
      close(bt_port);
      close(bt_t_port);
      printf("Close the port\n");
    }

  }

}

/** Get the command from the Pi-Swarm by the bluetooth port.
 *  The fuction get the command from the home beacon, which receive the command from the
 *  Pi-Swarm through RF communication.
 */
void revCmdFromPiSwarm()
{
  char msg_r = '\0';
  char msg_w = '\0';
  int nbytes;
  uint8_t i = 0;

  printf("Start to receive Command form the Pi-Swarm\n");

  ioctl(bt_port, FIONREAD, &nbytes);
  printf("Bytes in queue for reading:%d\n", nbytes);
  if (nbytes > 0)
  {
    if (read(bt_port, &msg_r, 1))
    {
      printf("Succefssfully reading from the port\n");
    }
  }
  if ((msg_r == 'r') || (msg_r == 'c') || (msg_r == 'm'))
  {
    while (i < 5)
    {
      if (write(bt_port, &msg_r, 1))
      {
        printf("Succefssfully writing to the port\n");
        msg_w = msg_r;
      }
      i++;
    }

  }

  printf("Reading:%c\n", msg_r);
  printf("Writing:%c\n", msg_w);

  if (msg_r == 'r')
  {
    printf("Command is successfully sent to AR.Drone to search black_roundel tag\n");
    recruitFlag = true;
    searchFlag = false;
    target_tag = 'r';
    tcflush(bt_port, TCIFLUSH);
  }
  else if (msg_r == 'c')
  {
    printf("Command is successfully sent to AR.Drone to search COCARDE tag\n");
    recruitFlag = true;
    searchFlag = false;
    target_tag = 'c';
    tcflush(bt_port, TCIFLUSH);
  }
  else if (msg_r == 'm')
  {
    printf("Command is successfully sent to AR.Drone to search mix type tag\n");
    recruitFlag = true;
    searchFlag = false;
    target_tag = 'm';
    tcflush(bt_port, TCIFLUSH);
  }

}

/** Send the command to the home beacon by the bluetooth port.
 *  @param[in] msg_w The input char variable means the symbol to send.
 *  The function send the command to the home beacon and compare the symbol
 *  sent and the symbol received to make sure the command is received.
 */
void sendCmdToHomeBeacon(char msg_w)
{
  char msg_r = '\0';
  int nbytes;
  printf("Start to send Command to the Home beacon\n");

  ioctl(bt_port, FIONREAD, &nbytes);
  printf("Bytes in queue for reading:%d\n", nbytes);
  if (nbytes > 0)
  {
    if (read(bt_port, &msg_r, 1))
    {
      printf("Succefssfully reading from the port\n");
    }
  }
  if (msg_r == msg_w)
  {

  }
  else
  {
    if (write(bt_port, &msg_w, 1))
    {
      printf("Succefssfully writing to the port\n");
    }
  }

  printf("Writing:%c\n", msg_w);
  printf("Reading:%c\n", msg_r);

  if (msg_r == msg_w)
  {
    printf("Command is successfully sent to Home beacon\n");
    sendToHome = false;
    beaconNotified = true;
    tcflush(bt_port, TCIFLUSH);
  }
}

/** Send the command to the target beacon by the bluetooth port.
 *  @param[in] msg_w The input char variable means the symbol to send.
 *  The function send the command to the target beacon and compare the symbol
 *  sent and the symbol received to make sure the command is received.
 */
void sendCmdToTargetBeacon(char msg_w)
{
  char msg_r = '\0';
  int nbytes;
  printf("Start to send Command to the Target beacon\n");

  ioctl(bt_t_port, FIONREAD, &nbytes);
  printf("Bytes in queue for reading:%d\n", nbytes);
  if (nbytes > 0)
  {
    if (read(bt_t_port, &msg_r, 1))
    {
      printf("Succefssfully reading from the port\n");
    }
  }
  if (msg_r == msg_w)
  {

  }
  else
  {
    if (write(bt_t_port, &msg_w, 1))
    {
      printf("Succefssfully writing to the port\n");
    }
  }

  printf("Writing:%c\n", msg_w);
  printf("Reading:%c\n", msg_r);

  if (msg_r == msg_w)
  {
    printf("Command is successfully sent to Target beacon\n");
    if (startFlag == true)
    {
      targetonFlag = false;
    }
    else
    {
      sendToTarget = false;
      sendToHome = true;
    }
    tcflush(bt_t_port, TCIFLUSH);
  }
}

/** Open the bluetooth port and set the port attributes.
 *  Open the bluetooth port from PC to home beacon and the port from PC to target beacon
 *  . Set the port attributes, such as the baud rate.
 */
void openBTPort()
{
  struct termios Opt;
  struct termios Opt_t;

  bt_port = open("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_NDELAY);
  //fcntl(bt_port, F_SETFL, 0);
  tcgetattr(bt_port, &Opt);
  cfsetispeed(&Opt, B115200);
  cfsetospeed(&Opt, B115200);
  //Opt.c_cflag |= (CLOCAL | CREAD);

  tcflush(bt_port, TCIOFLUSH);
  tcsetattr(bt_port, TCSANOW, &Opt);

  if (bt_port == -1)
  {
    ROS_INFO("Error opening the bt_port");
  }
  else
  {
    ROS_INFO("bt_port is open");

  }

  bt_t_port = open("/dev/rfcomm1", O_RDWR | O_NOCTTY | O_NDELAY);
  //fcntl(bt_t_port, F_SETFL, 0);
  tcgetattr(bt_t_port, &Opt_t);
  cfsetispeed(&Opt_t, B115200);
  cfsetospeed(&Opt_t, B115200);
  //Opt_t.c_cflag |= (CLOCAL | CREAD);

  tcflush(bt_t_port, TCIOFLUSH);
  tcsetattr(bt_t_port, TCSANOW, &Opt_t);

  if (bt_t_port == -1)
  {
    ROS_INFO("Error opening the bt_t_port");
  }
  else
  {
    ROS_INFO("bt_t_port is open");

  }
}

/** Main function and running loop for the prgp_piwarmcom package.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "prgp_piswarmcom");
  ros::NodeHandle ndh_; /**< ROS node handle */
  char msg = 'f';
  uint8_t l = 0;
  ndPause = ros::Duration(1, 0);

  cmdPiPub = ndh_.advertise<std_msgs::String>("piswarm_com", 1);
  cmdPiSub = ndh_.subscribe("piswarm_com", 1, ardroneCmdRevCb);
  openBTPort();

  //Reset the flag for mbed.
  while(l < 5)
  {
    if (write(bt_port, &msg, 1))
    {
      printf("Writing to reset the flag for home mbed\n");
    }
    if (write(bt_t_port, &msg, 1))
    {
      printf("Writing to reset the flag for target mbed\n");
    }
    l++;
  }

  while (ros::ok)
  {

    if (startFlag == true)
    {
      //targetonFlag = false;
      if (targetonFlag == true)
      {
        //Turn on the target beacon.
        sendCmdToTargetBeacon('o');
      }
      else
      {
        //Start the Pi-Swarm
        sendCmdToPiSwarm('s');
      }
    }

    if (searchFlag == true) //Pi-Swarm search the target
    {
      revCmdFromPiSwarm();
    }
    if (recruitFlag == true)
    {
#ifdef START_DRONE_PKG_IN_CODE
      std::cout << "STARTDRONE" << std::endl;
      sleep(15);
#endif
      switch (target_tag)
      {
        case 'r':
          c_P = "r";
          break;
        case 'c':
          c_P = "c";
          break;
        case 'm':
          c_P = "m";
          break;
        default:
          break;
      }
      s_P.data = c_P.c_str();
      cmdPiPub.publish(s_P);
      target_tag = '\0';
      sendToTarget = true;
      recruitFlag = false;

    }

    if (sendToTarget == true)
    {
      sendCmdToTargetBeacon('x'); // to Target beacon
    }

    if (sendToHome == true)
    {
      sendCmdToHomeBeacon('o'); // to Home beacon
    }

    if ((returnFlag == true) && (beaconNotified == true))
    {
      sendCmdToPiSwarm('b');
    }

    usleep(500000);
    ros::spinOnce();
  }

  return 0;
}

