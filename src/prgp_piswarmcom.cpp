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

#include <prgp_piswarmcom/prgp_piswarmcom.h>

/** Initialise the variables and paramaters.
 *  Initialise the ROS time, ROS Duration, Publishers, Subscribers, Flags and so on.
 */
PRGPPiSwarmCom::PRGPPiSwarmCom()
{
  ndPause = ros::Duration(1, 0);
  cmdPiPub = ndh_.advertise<std_msgs::String>("piswarm_com", 1);
  cmdPiSub = ndh_.subscribe("piswarm_com", 1, &PRGPPiSwarmCom::ardroneCmdRevCb, this);

  targetonFlag = true;
  startFlag = true;
  searchFlag = false;
  recruitFlag = false;
  returnFlag = false;
  sendToTarget = false;
  sendToHome = false;
  beaconNotified = false;

  target_tag = '\0';
  bt_port = -1;
  bt_t_port = -1;
  pi_starttime = 0;
  pi_returntime = 0;
  pi_targettime = 0;
  home_ontime = 0;
  target_ontime = 0;
  target_offtime = 0;
  face_change = true;
}

/** Class destructor.
 */
PRGPPiSwarmCom::~PRGPPiSwarmCom()
{

}

/** Callback function to receive the command from the ARDrone by topic piswarm_com.
 *  Getting the string from the piswarm_com topic and comparing it with the "b" to
 *  set the returnFlag. std::string k = str->data.substr(0, 1) can be used.
 */
void PRGPPiSwarmCom::ardroneCmdRevCb(const std_msgs::StringConstPtr str)
{
#ifdef DEBUG
  ROS_INFO_STREAM(*str);
  ROS_INFO("%s", str->data.c_str());
#endif

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
bool PRGPPiSwarmCom::sendCmdToPiSwarm(char msg_w)
{
  char msg_r = '\0';
  int nbytes;
#ifdef DEBUG
  printf("Start to send Command to the Pi-Swarm\n");
#endif
  ioctl(bt_port, FIONREAD, &nbytes);
#ifdef DEBUG
  printf("Bytes in queue for reading:%d\n", nbytes);
#endif
  if (nbytes > 0)
  {
    if (read(bt_port, &msg_r, 1))
    {
#ifdef DEBUG
      printf("Succefssfully reading from the port\n");
#endif
    }
  }
  if (msg_r == msg_w)
  {

  }
  else
  {
    if (write(bt_port, &msg_w, 1))
    {
#ifdef DEBUG
      printf("Succefssfully writing to the port\n");
#endif
    }
  }
#ifdef DEBUG
  printf("Writing:%c\n", msg_w);
  printf("Reading:%c\n", msg_r);
#endif

  if (msg_r == msg_w)
  {
#ifdef DEBUG
    printf("Command is successfully sent to Pi-Swarm\n");
#endif
    if (returnFlag == false)
    {
      searchFlag = true;
      startFlag = false;
      pi_starttime = ros::WallTime::now().toSec() - begin_time.toSec();
      tcflush(bt_port, TCIFLUSH);
      return true; //add this for unit testing
    }
    else
    {
      returnFlag = false;
      beaconNotified = false;
      pi_returntime = ros::WallTime::now().toSec() - begin_time.toSec();
      tcflush(bt_port, TCIFLUSH);
      close(bt_port);
      close(bt_t_port);
      printf("Close the port\n");
      return true; //add this for unit testing
    }

  }
  else
  {
    return false; //add this for unit testing
  }

}

/** Get the command from the Pi-Swarm by the bluetooth port.
 *  The fuction get the command from the home beacon, which receive the command from the
 *  Pi-Swarm through RF communication.
 */
bool PRGPPiSwarmCom::revCmdFromPiSwarm()
{
  char msg_r = '\0';
  char msg_w = '\0';
  int nbytes;
  uint8_t i = 0;
#ifdef DEBUG
  printf("Start to receive Command form the Pi-Swarm\n");
#endif

  ioctl(bt_port, FIONREAD, &nbytes);
#ifdef DEBUG
  printf("Bytes in queue for reading:%d\n", nbytes);
#endif

  if (nbytes > 0)
  {
    if (read(bt_port, &msg_r, 1))
    {
#ifdef DEBUG
      printf("Succefssfully reading from the port\n");
#endif
    }
  }
  if ((msg_r == 'r') || (msg_r == 'c') || (msg_r == 'm'))
  {
    while (i < 5)
    {
      if (write(bt_port, &msg_r, 1))
      {
#ifdef DEUG
        printf("Succefssfully writing to the port\n");
#endif
        msg_w = msg_r;
      }
      i++;
    }

  }
#ifdef DEBUG
  printf("Reading:%c\n", msg_r);
  printf("Writing:%c\n", msg_w);
#endif

  if (msg_r == 'r')
  {
#ifdef DEBUG
    printf("Command is successfully sent to AR.Drone to search black_roundel tag\n");
#endif
    recruitFlag = true;
    searchFlag = false;
    target_tag = 'r';
    pi_targettime = ros::WallTime::now().toSec() - begin_time.toSec();
    tcflush(bt_port, TCIFLUSH);
  }
  else if (msg_r == 'c')
  {
#ifdef DEBUG
    printf("Command is successfully sent to AR.Drone to search COCARDE tag\n");
#endif
    recruitFlag = true;
    searchFlag = false;
    target_tag = 'c';
    pi_targettime = ros::WallTime::now().toSec() - begin_time.toSec();
    tcflush(bt_port, TCIFLUSH);
  }
  else if (msg_r == 'm')
  {
#ifdef DEBUG
    printf("Command is successfully sent to AR.Drone to search mix type tag\n");
#endif
    recruitFlag = true;
    searchFlag = false;
    target_tag = 'm';
    pi_targettime = ros::WallTime::now().toSec() - begin_time.toSec();
    tcflush(bt_port, TCIFLUSH);
  }
  return recruitFlag; //add this for unit testing
}

/** Send the command to the home beacon by the bluetooth port.
 *  @param[in] msg_w The input char variable means the symbol to send.
 *  The function send the command to the home beacon and compare the symbol
 *  sent and the symbol received to make sure the command is received.
 */
bool PRGPPiSwarmCom::sendCmdToHomeBeacon(char msg_w)
{
  char msg_r = '\0';
  int nbytes;
#ifdef DEBUG
  printf("Start to send Command to the Home beacon\n");
#endif

  ioctl(bt_port, FIONREAD, &nbytes);
#ifdef DEBUG
  printf("Bytes in queue for reading:%d\n", nbytes);
#endif
  if (nbytes > 0)
  {
    if (read(bt_port, &msg_r, 1))
    {
#ifdef DEBUG
      printf("Succefssfully reading from the port\n");
#endif
    }
  }
  if (msg_r == msg_w)
  {

  }
  else
  {
    if (write(bt_port, &msg_w, 1))
    {
#ifdef DEBUG
      printf("Succefssfully writing to the port\n");
#endif
    }
  }
#ifdef DEBUG
  printf("Writing:%c\n", msg_w);
  printf("Reading:%c\n", msg_r);
#endif

  if (msg_r == msg_w)
  {
#ifdef DEBUG
    printf("Command is successfully sent to Home beacon\n");
#endif
    sendToHome = false;
    beaconNotified = true;
    home_ontime = ros::WallTime::now().toSec() - begin_time.toSec();
    tcflush(bt_port, TCIFLUSH);
  }
  return beaconNotified; //add this for unit testing
}

/** Send the command to the target beacon by the bluetooth port.
 *  @param[in] msg_w The input char variable means the symbol to send.
 *  The function send the command to the target beacon and compare the symbol
 *  sent and the symbol received to make sure the command is received.
 */
bool PRGPPiSwarmCom::sendCmdToTargetBeacon(char msg_w)
{
  char msg_r = '\0';
  int nbytes;
#ifdef DEBUG
  printf("Start to send Command to the Target beacon\n");
#endif

  ioctl(bt_t_port, FIONREAD, &nbytes);
#ifdef DEBUG
  printf("Bytes in queue for reading:%d\n", nbytes);
#endif
  if (nbytes > 0)
  {
    if (read(bt_t_port, &msg_r, 1))
    {
#ifdef DEBUG
      printf("Succefssfully reading from the port\n");
#endif
    }
  }
  if (msg_r == msg_w)
  {

  }
  else
  {
    if (write(bt_t_port, &msg_w, 1))
    {
#ifdef DEBUG
      printf("Succefssfully writing to the port\n");
#endif
    }
  }
#ifdef DEBUG
  printf("Writing:%c\n", msg_w);
  printf("Reading:%c\n", msg_r);
#endif

  if (msg_r == msg_w)
  {
#ifdef DEBUG
    printf("Command is successfully sent to Target beacon\n");
#endif
    if (startFlag == true)
    {
      targetonFlag = false;
      target_ontime = ros::WallTime::now().toSec() - begin_time.toSec();
      return true; //add this for unit testing
    }
    else
    {
      sendToTarget = false;
      sendToHome = true;
      target_offtime = ros::WallTime::now().toSec() - begin_time.toSec();
      return true; //add this for unit testing
    }
    tcflush(bt_t_port, TCIFLUSH);
  }
  else
  {
    return false; //add this for unit testing
  }
}

/** Open the bluetooth port and set the port attributes.
 *  Open the bluetooth port from PC to home beacon and the port from PC to target beacon
 *  . Set the port attributes, such as the baud rate.
 */
bool PRGPPiSwarmCom::openBTPort()
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
    printf("Error opening the bt_port\n");
  }
  else
  {
    printf("bt_port is open\n");

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
    printf("Error opening the bt_t_port\n");
  }
  else
  {
    printf("bt_t_port is open\n");

  }

  if ((bt_port == 1) && (bt_t_port == 1))
  {
    return true; //add this for unit testing
  }
  else
  {
    return false; //add this for unit testing
  }
}

/** The function to implement the UI for the project.
 *  The function draws the UI for the project. It displays the project information and results.
 */
bool PRGPPiSwarmCom::prgp_ui()
{
  double clock_time = 0;
  uint32_t min = 0;
  uint32_t sec = 0;
  uint32_t msec = 0;

  printf("\033[2J"); //clear the screen.
  clock_time = ros::WallTime::now().toSec() - begin_time.toSec();

  min = int(clock_time / 60.0);
  sec = int(fmod(clock_time, 60));
  msec = int((fmod(clock_time, 60) - sec) * 10);

  printf("-----------------------------------------\n");
  if (face_change == true)
  {
    printf("|   *\\(^_^)/*    Welcome!   *\\(^_^)/*   |\n");
    face_change = false;
  }
  else
  {
    printf("|   *\\(o_o)/*    Welcome!   *\\(o_o)/*   |\n");
    face_change = true;
  }
  printf("-----------------------------------------\n");
  printf("|**********University of York***********|\n");
  printf("|**************Robot Lab****************|\n");
  printf("|*********ARE PRGP Project 2015*********|\n");
  printf("|      Robert Evans, Homero Silva       |\n");
  printf("|     Shengsong Yang, Chengqing Liu     |\n");
  printf("|**************Time count***************|\n");
  printf("|               %02d:%02d.%d                 |\n", min, sec, msec);
  printf("|**************Time count***************|\n");
  printf("|                                       |\n");
  printf("|************Project results************|\n");
  if (target_ontime > 0)
  {
    printf("|1. target_ontime:     %02d:%02d.%d          |\n", int(target_ontime / 60.0), int(fmod(target_ontime, 60)),
           int((fmod(target_ontime, 60) - int(fmod(target_ontime, 60))) * 10));
  }
  else
  {
    printf("|1. target_ontime:     %02d:%02d.%d          |\n", min, sec, msec);
  }
  if (pi_starttime > 0)
  {
    printf("|2. pi_starttime:      %02d:%02d.%d          |\n", int(pi_starttime / 60.0), int(fmod(pi_starttime, 60)),
           int((fmod(pi_starttime, 60) - int(fmod(pi_starttime, 60))) * 10));
  }
  else
  {
    printf("|2. pi_starttime:      %02d:%02d.%d          |\n", min, sec, msec);
  }
  if (pi_targettime > 0)
  {
    printf("|3. pi_targettime:     %02d:%02d.%d          |\n", int(pi_targettime / 60.0), int(fmod(pi_targettime, 60)),
           int((fmod(pi_targettime, 60) - int(fmod(pi_targettime, 60))) * 10));

  }
  else
  {
    printf("|3. pi_targettime:     %02d:%02d.%d          |\n", min, sec, msec);
  }
  if (target_offtime > 0)
  {
    printf("|4. target_offtime:    %02d:%02d.%d          |\n", int(target_offtime / 60.0),
           int(fmod(target_offtime, 60)), int((fmod(target_offtime, 60) - int(fmod(target_offtime, 60))) * 10));
  }
  else
  {
    printf("|4. target_offtime:    %02d:%02d.%d          |\n", min, sec, msec);
  }
  if (home_ontime > 0)
  {
    printf("|5. home_ontime:       %02d:%02d.%d          |\n", int(home_ontime / 60.0), int(fmod(home_ontime, 60)),
           int((fmod(home_ontime, 60) - int(fmod(home_ontime, 60))) * 10));
  }
  else
  {
    printf("|5. home_ontime:       %02d:%02d.%d          |\n", min, sec, msec);
  }
  if (pi_returntime > 0)
  {
    printf("|5. pi_returntime:     %02d:%02d.%d          |\n", int(pi_returntime / 60.0), int(fmod(pi_returntime, 60)),
           int((fmod(pi_returntime, 60) - int(fmod(pi_returntime, 60))) * 10));
  }
  else
  {
    printf("|6. pi_returntime:     %02d:%02d.%d          |\n", min, sec, msec);
  }
  printf("|************Project results************|\n");

  printf("-----------------------------------------\n");

  return true;
}
/** Main running loop for the prgp_piwarmcom package.
 *  This function is to open the BT port, reset the flags in the Home and target beacon with the command,
 *  send the command to turn on the target beacon, send the command to start the Pi-Swarm via home beacon,
 *  get the command from Pi-Swarm via home beacon and publish command to prgp_ardrone package via the topic.
 *  Then the function is to turn off the target beacon and turn on the home beacon. After getting the command
 *  from the prgp_ardrone package via the topic, the function is to send the command to return the Pi-Swarm.
 */
bool PRGPPiSwarmCom::run()
{
  char msg = 'f';
  uint8_t l = 0;

  openBTPort();
  sleep(1);

  //Reset the flag for mbed.
  while (l < 5)
  {
    if (write(bt_port, &msg, 1))
    {
#ifdef DEBUG
      printf("Writing to reset the flag for home mbed\n");
#endif
    }
    if (write(bt_t_port, &msg, 1))
    {
#ifdef DEBUG
      printf("Writing to reset the flag for target mbed\n");
#endif
    }
    l++;
  }

  begin_time = ros::WallTime::now();

  while (ros::ok)
  {
    prgp_ui();

    if (startFlag == true)
    {

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
      //The prpg_launch.sh is not quite reliable to identify this. So this is commented.
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
  return true; //add this for unit testing
}

