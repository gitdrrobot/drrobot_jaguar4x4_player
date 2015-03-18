/*!
 *  drrobotMsgTester
 *  Copyright (c) 2011, Dr Robot Inc
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
  drrobotMsgTester is for demonstration and testing sensor message published by drrobot_player.
  It will received all the sensor information and print out on console.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
          then run drrobot_player first.
@verbatim
$ drrobotMsgTester
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
-@b drrobot_motor: will receive MotionInfoArray Message. Please referee the message file.
-@b drrobot_powerinfo: will receive PowerInfo Message. Please referee the message file.
-@b drrobot_ir: will receive RangeArray Message for IR sensor, and transform AD value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_sonar: will receive RangeArray Message for ultrasonic sensor, and transform value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_standardsensor: will receive StandardardSensor Message. Please referee the message file.
-@b drrobot_customsensor: will receive CustomSensor Message. Please referee the message file. Not available for standard I90/Sentinel3/Hawk/H20/X80SV robot
<hr>
*/
#include <ros/ros.h>
#include <drrobot_jaguar4x4_player/MotorInfo.h>
#include <drrobot_jaguar4x4_player/MotorInfoArray.h>
#include <drrobot_jaguar4x4_player/RangeArray.h>
#include <drrobot_jaguar4x4_player/Range.h>
#include <drrobot_jaguar4x4_player/PowerInfo.h>
#include <drrobot_jaguar4x4_player/StandardSensor.h>
#include <drrobot_jaguar4x4_player/CustomSensor.h>
#include <drrobot_jaguar4x4_player/IMUInfo.h>
#include <std_msgs/Header.h>



/*! \brief
 * here is temperature sensor data table for translating AD value to temperature
*/
 double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
 double tempTable[25] = {-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};

/*! \brief
 *      This call back function is for MotorInfoArray message.
 *  @param[in] msg  is a pointer of MotorInfoArray message, the array size should be 6
 *  @return null
 */
void motorSensorCallback(const drrobot_jaguar4x4_player::MotorInfoArray::ConstPtr& msg)
{
  int msgSize = msg->motorInfos.capacity();

  if (msgSize == 6)
  {
      ROS_INFO("Motor Encoder Pos: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_pos, msg->motorInfos[1].encoder_pos, msg->motorInfos[2].encoder_pos
               , msg->motorInfos[3].encoder_pos, msg->motorInfos[4].encoder_pos, msg->motorInfos[5].encoder_pos);
      ROS_INFO("Motor Encoder Vel: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_vel, msg->motorInfos[1].encoder_vel, msg->motorInfos[2].encoder_vel
                 , msg->motorInfos[3].encoder_vel, msg->motorInfos[4].encoder_vel, msg->motorInfos[5].encoder_vel);
      ROS_INFO("Motor Encoder Dir: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_dir, msg->motorInfos[1].encoder_dir, msg->motorInfos[2].encoder_dir
                 , msg->motorInfos[3].encoder_dir, msg->motorInfos[4].encoder_dir, msg->motorInfos[5].encoder_dir);
      ROS_INFO("Motor Motor Current: [%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]", msg->motorInfos[0].motor_current, msg->motorInfos[1].motor_current, msg->motorInfos[2].motor_current
                   , msg->motorInfos[3].motor_current, msg->motorInfos[4].motor_current, msg->motorInfos[5].motor_current);
      ROS_INFO("Motor Motor_PWM: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].motor_pwm, msg->motorInfos[1].motor_pwm, msg->motorInfos[2].motor_pwm
                 , msg->motorInfos[3].motor_pwm, msg->motorInfos[4].motor_pwm, msg->motorInfos[5].motor_pwm);
  }
}

/*! \brief
 *      This call back function is for RangArray message of IR range sensor
 *  @param[in] msg  is a pointer of RangeArray message, the array size should be 10
 *  @return null
 */
void irSensorCallback(const drrobot_jaguar4x4_player::RangeArray::ConstPtr& msg)
{
  int msgSize = msg->ranges.capacity();
  if (msgSize == 10)
  {
    ROS_INFO("IR Distance:[%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]", msg->ranges[0].range,msg->ranges[1].range,msg->ranges[2].range,msg->ranges[3].range,
             msg->ranges[4].range,msg->ranges[5].range,msg->ranges[6].range,msg->ranges[7].range,msg->ranges[8].range,msg->ranges[9].range);
  }

}

/*! \brief
 *      This call back function is for RangArray message of Ultrasonic range sensor
 *  @param[in] msg  is a pointer of RangeArray message, the array size should be 6
 *  @return null
 */
void usSensorCallback(const drrobot_jaguar4x4_player::RangeArray::ConstPtr& msg)
{
  int msgSize = msg->ranges.capacity();
  if (msgSize == 6)
  {
    ROS_INFO("US Distance:[%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]", msg->ranges[0].range,msg->ranges[1].range,msg->ranges[2].range,msg->ranges[3].range,
             msg->ranges[4].range,msg->ranges[5].range);
  }

}


/*! \brief
 *      This call back function is for StandardSensor message
 *  @param[in] msg  is a pointer of StandardSensor message
 *  @return null
 */
void standardSensorCallback(const drrobot_jaguar4x4_player::StandardSensor::ConstPtr& msg)
{

  ROS_INFO("Human Sensor:[%d, %d, %d, %d]", msg->humanSensorData[0],msg->humanSensorData[1],msg->humanSensorData[2],msg->humanSensorData[3]);
  ROS_INFO("Board Power Voltage: [%2.2f V]", msg->boardPowerVol);
  ROS_INFO("Motor Power Voltage: [%2.2f V]", msg->motorPowerVol);
  ROS_INFO("Servo Power Voltage: [%2.2f V]", msg->servoPowerVol);
/*!
 *  Standard I90 series robot does not have below sensor, only available on X80SVP
 */
  ROS_INFO("Temperature Sensor: [%d]", msg->thermoSensorData);
  ROS_INFO("Board Over Heat Sensor: [%d, %d]", msg->overHeatSensorData[0], msg->overHeatSensorData[1]);
  ROS_INFO("Tilting Sensor:[%d, %d]", msg->tiltingSensorData[0],msg->tiltingSensorData[1]);


}

/*! \brief
 *      This call back function is for PowerInfo message, only available on I90/Sentinel3/Hawk/H20 robot
 *  @param[in] msg  is a pointer of PowerInfo message
 *  @return null
 */
void powerSensorCallback(const drrobot_jaguar4x4_player::PowerInfo::ConstPtr& msg)
{


  ROS_INFO("Battery 1 Voltage: [%2.2f V]", msg->bat1_vol);
  ROS_INFO("Battery 2 Voltage: [%2.2f V]", msg->bat2_vol);
  ROS_INFO("DCIN Power Voltage: [%2.2f V]", msg->dcin_vol);

  ROS_INFO("Battery 1 Temperature Sensor: [%f]", msg->bat1_temp);
  ROS_INFO("Battery 2 Temperature Sensor: [%f]", msg->bat2_temp);

  ROS_INFO("Power Status: [ %d ]", msg->power_status);
  ROS_INFO("Power Path: [ %d ]", msg->power_path);
  ROS_INFO("Charge Path: [ %d ]", msg->charge_path);

}
/*! \brief
 *      This function is for translating AD value to temperature degree according sensor data
 *  @param[in] adValue  is raw AD value
 *  @return temperature in degree
 */


double trans2Temperature(int adValue)
{
 double tempM = 0;
 double k = (double)adValue/4095;
 double resValue = 0;
 
 if (k != 1)
 {
	resValue = 10000 * k/(1-k);
 }
 else
 {
	resValue = resTable[0];
 }
 int index = -1;
 if (resValue >= resTable[0])
 {
	tempM = -20;
 }
 else if(resValue <= resTable[24])
 {
	tempM = 100;
 }
 else
 { 
	for(int i = 0; i < 24; i++)
	{
		if (resValue < resTable[i] && (resValue > resTable[i + 1]) )
		{
			index = i;
			break;
		}
	}
	if (index >= 0)
	{
		tempM = tempTable[index] + (resValue - resTable[index])/(resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
	}
        else
	{
		tempM = 0;
	}
 }
 return tempM;

}

/*! \brief
 *      This call back function is for custom sensor message, 
 *  @param[in] msg  is a pointer of custom sensor message
 * please make sure what kind of sensor on the custom AD channel
 * for Jaguar Robot, the custom AD channel[4~7] is for motor temperature sensor
 * For Jaguar 4X4 , AD[4] is for left front motor, AD[5] is for right front motor, AD[6] is for left rear motor, AD[7] is for right rear motor
 *  @return null
 */
void customSensorCallback(const drrobot_jaguar4x4_player::CustomSensor::ConstPtr& msg)
{

/*
  ROS_INFO("Custom AD Channel:[%d, %d, %d, %d, %d, %d, %d, %d]", msg->customADData[0],msg->customADData[1],msg->customADData[2],msg->customADData[3],msg->customADData[4],msg->customADData[5],msg->customADData[6],msg->customADData[7]);
  ROS_INFO(" Custom IO: %d", msg->customIO);
*/
  ROS_INFO("Left Front Motor Temperature:[%3.2f]", trans2Temperature(msg->customADData[4]));
  ROS_INFO("Right Front Motor Temperature:[%3.2f]", trans2Temperature(msg->customADData[5]));
  ROS_INFO("Left Rear Motor Temperature:[%3.2f]", trans2Temperature(msg->customADData[6]));
  ROS_INFO("Right Rear Motor Temperature:[%3.2f]", trans2Temperature(msg->customADData[7]));
}

/*! \brief
 *      This call back function is for imu sensor message, 
 *  @param[in] msg  is a pointer of imu sensor message
 *  @return null
 */
void imuSensorCallback(const drrobot_jaguar4x4_player::IMUInfo::ConstPtr& msg)
{

  ROS_INFO(" AccelX:[%d]", msg->accelRawData[0]);
  ROS_INFO(" AccelY:[%d]", msg->accelRawData[1]);
  ROS_INFO(" AccelZ:[%d]", msg->accelRawData[2]);
  ROS_INFO(" GyroX:[%d]", msg->gyroRawData[0]);
  ROS_INFO(" GyroY:[%d]", msg->gyroRawData[1]);
  ROS_INFO(" GyroZ:[%d]", msg->gyroRawData[2]);
  ROS_INFO(" MagX:[%d]", msg->magRawData[0]);
  ROS_INFO(" MagY:[%d]", msg->magRawData[1]);
  ROS_INFO(" MagZ:[%d]", msg->magRawData[2]);

}


int main(int argc, char **argv)
{
  /*!
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "drrobot_msgTester");

  /*!
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /*!
   *  here will subscribe the sensor messages which are published by drrobot_player
   *  The second parameter to the subscribe() function is set as '1' because we are only
   *  interested at latest sensor information
   */
  ros::Subscriber motorSensorSub = n.subscribe("drrobot_motor", 1, motorSensorCallback);
  ros::Subscriber irSensorSub = n.subscribe("drrobot_ir", 1, irSensorCallback);
  ros::Subscriber usSensorSub = n.subscribe("drrobot_sonar", 1, usSensorCallback);
  ros::Subscriber standardSensorSub = n.subscribe("drrobot_standardsensor", 1, standardSensorCallback);
  ros::Subscriber powerSensorSub = n.subscribe("drrobot_powerinfo", 1, powerSensorCallback);
  ros::Subscriber customSensorSub = n.subscribe("drrobot_customsensor", 1, customSensorCallback);
  ros::Subscriber imuSensorSub = n.subscribe("drrobot_imu", 1, imuSensorCallback);
  /*!
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

