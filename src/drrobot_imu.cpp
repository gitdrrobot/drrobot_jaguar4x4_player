/*!
 *  drrobot_imu
 *  Copyright (c) 2013, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_imu is a driver for imu device on the Jaguar robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, Jaguar robot, you need load drrobotplayer_Jaguar***.yaml use command "rosparam load drrobotplayer_jaguar*.yaml"
@verbatim
$ drrobot_imu
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b drrobot_imu_raw: will publish imu raw message. Please referee the message file.
<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotIMUIP (string) : robot IMU Network module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number 
 */
#include <stdexcept>
#include <termios.h>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <drrobot_jaguar4x4_player/IMUInfo.h>
using namespace std;


#define MAXBUFLEN 8096
#define COMM_LOST_TH	200
#undef DEBUG_ERROR
class DrRobotIMURawNode
{
public:

 /*! \struct  MotorSensorData
   *  for motor sensor data
   */
  struct IMUSensorData
  {
    int seqNum;
    int accelRawData[3];         //!< accel sensor raw data, 0- x axis, 1- y axis 2- z axis
    int gyroRawData[3];         //!< gyro sensor raw data, 0- x axis, 1- y axis 2- z axis
    int magRawData[3];         //!< magnetic sensor raw data, 0- x axis, 1- y axis 2- z axis
  };

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;

    ros::Publisher imu_raw_pub_;
    std::string robot_prefix_;

    DrRobotIMURawNode()
    {
        ros::NodeHandle private_nh("~");

        _robotIP = "192.168.0.61";
        private_nh.getParam("IMUIP",_robotIP);
        ROS_INFO("I get IMU_IP: [%s]", _robotIP.c_str());


        _commPortNum = 10001;
 	private_nh.getParam("IMUPortNum",_commPortNum);
        ROS_INFO("I get IMU_IP: [%d]", _commPortNum);

        //create publishers for sensor data information
        imu_raw_pub_ = node_.advertise<drrobot_jaguar4x4_player::IMUInfo>("drrobot_imu", 1);
	//prepare network communication
	memset(&_imuServer,0,sizeof(_imuServer));
	_imuServer.sin_family = AF_INET;
	_imuServer.sin_addr.s_addr = inet_addr(_robotIP.c_str());
	_imuServer.sin_port = htons(_commPortNum);

	if ( (_sockfd = socket(AF_INET,SOCK_STREAM,0)) < 0)
	{
		ROS_INFO("Failed to creat socket");	
	}	
        _comCnt = 0;
	_stopComm = true;
	_tv.tv_sec = 0;
	_tv.tv_usec = 200;             //200us ?
	strcpy(_revMsg, "");
	
    }

    ~DrRobotIMURawNode()
    {
    }


void handleComData(const char *data, const int nLen)
{
	int endPos = 0;
	int revLen = strlen(data);
	int tokCnt = 0;
	char * readPtr;
	char sepChar[1];
	sepChar[0] = ',';
	if (strlen(_revMsg) + nLen < MAXBUFLEN)
	{
		strcat(_revMsg,data);
	}
	else
	{
		//clear revMsg
		strcpy(_revMsg,"");
		strcat(_revMsg,data);
	}

	
Proc_Start:
	endPos = 0;
	revLen = strlen(_revMsg);
	for (int i = 0; i < revLen - 2; i ++)
	{
		if ((_revMsg[i] == '\n') && (_revMsg[i + 1] == '\r')) 
		{
			endPos = i + 1;
			break;
		}
	}

	if (endPos > 0)
	{
		// means find a line of message, process here
		strncpy(_processMsg,_revMsg,endPos);
#ifdef DEBUG_ERROR
		printf("process packet length is %d bytes long\n", strlen(_processMsg));
		printf("process packet is %s \n", _processMsg);
#endif
		tokCnt = 0;
		if (_processMsg[0] == 0x24)	//here is the head
		{
			readPtr = strtok(_processMsg,sepChar);
			_imuSensorData.seqNum = atoi(readPtr+ 1);
#ifdef DEBUG_ERROR
			printf("process packet Sequence Number is %d \n", _imuSensorData.seqNum);
#endif
			while( (readPtr = strtok(NULL,sepChar)) != NULL)
			{
			

				if (tokCnt == 0){
					// accel x
					_imuSensorData.accelRawData[0] = atoi(readPtr);
				}
				else if (tokCnt == 1){
					// accel y
					_imuSensorData.accelRawData[1] = atoi(readPtr);
				}
				else if (tokCnt == 2){
					// accel z
					_imuSensorData.accelRawData[2] = atoi(readPtr);
				}
				else if (tokCnt == 3){
					// gyro x
					_imuSensorData.gyroRawData[0] = atoi(readPtr);
				}
				else if (tokCnt == 4){
					// gyro y
					_imuSensorData.gyroRawData[1] = atoi(readPtr);
				}
				else if (tokCnt == 5){
					// gyro z
					_imuSensorData.gyroRawData[2] = atoi(readPtr);
				}
				else if (tokCnt == 6){
					// mag x
					_imuSensorData.magRawData[0] = atoi(readPtr);
				}

				else if (tokCnt == 7){
					// mag y
					_imuSensorData.magRawData[1] = atoi(readPtr);
				}
				else if (tokCnt == 8){
					// mag z
					_imuSensorData.magRawData[2] = atoi(readPtr);
				}
			 tokCnt ++;
			if (tokCnt > 8) 
				{
					doUpdate();
					break;
				}
			
			}
		}	

		// if endPos is not the last one, trim the revMsg to prepare next message process
		if (endPos < strlen(_revMsg))
		{
			int j = 0;
			for(j = 0; j < strlen(_revMsg) - endPos - 1; j ++)
			{
				_revMsg[j] = _revMsg[endPos + 1 + j];
			}
			_revMsg[j] = 0x0;
		}
#ifdef DEBUG_ERROR
		printf("end position is %d \n", endPos);
		printf("PPP message is %s \n", _revMsg);
#endif
		goto Proc_Start;
	}
	else
	{
		// do nothing
	}
	return;
}
   //communication thread here
void commWorkingThread(){
  while(!_stopComm)
  {
      FD_ZERO(&_readfds);
      FD_SET(_sockfd, &_readfds);
      select(_sockfd + 1, &_readfds, NULL, NULL, &_tv);
      if (FD_ISSET(_sockfd,&_readfds))
      {
	strcpy(_recBuf,"");
        if ( (_numbytes = recv(_sockfd, _recBuf, MAXBUFLEN - 1,0))  < 0)
        {
                perror("recv");
                return;
        }
  #ifdef DEBUG_ERROR
	_recBuf[_numbytes] =0x0;
        printf("listener: packet is %d bytes long\n", _numbytes);
	printf("listener: packet is %s \n", _recBuf);
  #endif
        _comCnt = 0;
        handleComData(_recBuf,_numbytes);
      }
      else
      {
        _comCnt++;

        usleep(1000);              //1ms
        if (_comCnt > COMM_LOST_TH)
        {
          printf("Communication is lost, need close all. Port: %d \n", _commPortNum);
          _stopComm = true;
          return;
        }
      }
  }
  return;
}



    int start()
    {
	//setup network connection
      int res = -1;
        res = connect(_sockfd,(struct sockaddr *) &_imuServer,sizeof(_imuServer));

	if (res == 0)
	{
		_stopComm = false;
		ROS_INFO("open port number at: [%d]", _commPortNum);
		_pCommThread = boost::thread(&DrRobotIMURawNode::commWorkingThread,this);

	}
	else
	{
		
		_stopComm = true;
		ROS_INFO("could not open network connection to [%d]",  _commPortNum);
		//ROS_INFO("error code [%d]",  res);
	}

        return (res);
    }

    int stop()
    {
        close(_sockfd);
	_pCommThread.join();
        usleep(1000000);
        return(0);
    }


    void doUpdate()
    {
	drrobot_jaguar4x4_player::IMUInfo imuInfo;
	 imuInfo.header.stamp = ros::Time::now();
         imuInfo.header.frame_id = string("drrobot_IMU_");
	 imuInfo.header.frame_id += boost::lexical_cast<std::string>(_imuSensorData.seqNum);
	 
	 imuInfo.accelRawData[0] = _imuSensorData.accelRawData[0];
	 imuInfo.accelRawData[1] = _imuSensorData.accelRawData[1];
	 imuInfo.accelRawData[2] = _imuSensorData.accelRawData[2];

	 imuInfo.gyroRawData[0] = _imuSensorData.gyroRawData[0];
	 imuInfo.gyroRawData[1] = _imuSensorData.gyroRawData[1];
	 imuInfo.gyroRawData[2] = _imuSensorData.gyroRawData[2];

	 imuInfo.magRawData[0] = _imuSensorData.magRawData[0];
	 imuInfo.magRawData[1] = _imuSensorData.magRawData[1];
	 imuInfo.magRawData[2] = _imuSensorData.magRawData[2];

	ROS_INFO("Publish IMU Raw Data [%s,%d,%d,%d,%d,%d,%d,%d,%d,%d]", string(imuInfo.header.frame_id).c_str(), imuInfo.accelRawData[0],imuInfo.accelRawData[1],imuInfo.accelRawData[2],imuInfo.gyroRawData[0],imuInfo.gyroRawData[1],imuInfo.gyroRawData[2],imuInfo.magRawData[0],imuInfo.magRawData[1],imuInfo.magRawData[2]);
          imu_raw_pub_.publish(imuInfo);
    }

private:

    struct IMUSensorData _imuSensorData;
    std::string _robotIP;
    int  _commPortNum;
    int _comCnt;
    struct sockaddr_in _imuServer;
    int _sockfd;
    boost::thread _pCommThread;
    bool _stopComm;
    fd_set _readfds;
    struct timeval _tv;
    int _numbytes;
    char _recBuf[MAXBUFLEN];
    char _revMsg[MAXBUFLEN];
    char _processMsg[MAXBUFLEN];
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drrobot_imu");

    DrRobotIMURawNode drrobotIMURaw;
    ros::NodeHandle n;
    // Start up the robot
    if (drrobotIMURaw.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

    ros::Rate loop_rate(10);      //10Hz

    while (n.ok())
    {
    //  drrobotIMURaw.doUpdate();
      ros::spinOnce();
     loop_rate.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop 
    drrobotIMURaw.stop();

    return(0);
}

