/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date Jan 2015
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/drrobot_jaguar_v6/qnode.hpp"
#include "../include/drrobot_jaguar_v6/drrobotprotocol.hpp"
#include <jaguar/MotorInfo.h>
#include <jaguar/MotorInfoArray.h>
#include <jaguar/BaseMotorCmd.h>
#include <jaguar/FlipMotorCmd.h>
#include <jaguar/GPSInfo.h>
#include <jaguar/IMUInfo.h>
#include <jaguar/MotorBoardInfo.h>
#include <jaguar/MotorBoardInfoArray.h>
#include <chapter6_tutorials/Distance.h>
#include <math.h>
// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drrobot_jaguar_v6 {


/*****************************************************************************
** Implementation
*****************************************************************************/
int vel =1000;
QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"drrobot_jaguar_v6");
    if ( ! ros::master::check() ) {
        return false;
    }

    msgCnt = 0;
    stairsDistance = false;
    previousFF = 0;
    previousFR = 0;

    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
	// Add your ros communications here.
    motorInfo_pub_ = n.advertise<jaguar::MotorInfoArray>("drrobot_jaguar_v6_motor_sensor", 1);
    motorboardInfoArray_pub_= n.advertise<jaguar::MotorBoardInfoArray>("drrobot_jaguar_v6_motorboard_sensor",1);
    gpsInfo_pub_ = n.advertise<jaguar::GPSInfo>("drrobot_jaguar_v6_gps_sensor",1);
    imuInfo_pub_ = n.advertise<jaguar::IMUInfo>("drrobot_jaguar_v6_imu_sensor",1);
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    //motor_cmd_sub_ = n.subscribe<jaguar::BaseMotorCmd>("drrobot_jaguar_v6_basemotor_cmd", 1, boost::bind(&QNode::wheelCmdReceived, this, _1));
    //flipmotor_cmd_sub_ = n.subscribe<jaguar::FlipMotorCmd>("drrobot_jaguar_v6_flipmotor_cmd", 1, boost::bind(&QNode::flipCmdReceived, this, _1));
    motor_cmd_sub_ = n.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1, boost::bind(&QNode::wheelCmdReceived, this, _1));
    flipmotor_cmd_sub_ = n.subscribe<geometry_msgs::TwistStamped>("cmd_flipper",1, boost::bind(&QNode::flipCmdReceived, this, _1));
    joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&QNode::ButtonCmdReceived, this, _1));

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	
	return true;
}

void QNode::publisherMotorData(MotorData motorData[],int len)
{
    jaguar::MotorInfoArray motorInfoArray;
        motorInfoArray.motorInfo.resize(len);
        for (uint32_t i = 0 ; i < len; ++i)
        {

                motorInfoArray.motorInfo[i].header.stamp = ros::Time::now();
                motorInfoArray.motorInfo[i].header.frame_id = "/drrobot_jaguar_v6_motor";
                motorInfoArray.motorInfo[i].encoderPos = motorData[i].encoderPos;
                motorInfoArray.motorInfo[i].encoderVel = motorData[i].encoderSpeed;
                motorInfoArray.motorInfo[i].motorPower = motorData[i].motorPower;
                motorInfoArray.motorInfo[i].motorTemp = motorData[i].motorTemp;	// motor temperature reading
                motorInfoArray.motorInfo[i].motorCurrent = motorData[i].motorAmp;  //motor current feedback reading
                motorInfoArray.motorInfo[i].encoderDiff = 0;//not used now
        }

   // ROS_INFO("publish motor info array");
        motorInfo_pub_.publish(motorInfoArray);
}

void QNode::publisherMotorBoardInfoArray(MotorBoardData motorBoardData[],int len)
{
    jaguar::MotorBoardInfoArray motorBoardInfoArray;
     motorBoardInfoArray.motorBoardInfo.resize(len);
    for (uint32_t i = 0; i < len; i++){

        motorBoardInfoArray.motorBoardInfo[i].header.stamp = ros::Time::now();
        motorBoardInfoArray.motorBoardInfo[i].header.frame_id = "/drrobot_jaguar_v6_motorboard";
        motorBoardInfoArray.motorBoardInfo[i].status = motorBoardData[i].driverState;
        motorBoardInfoArray.motorBoardInfo[i].temp1 = 0;	  // temperature 1, internal chip temperature, no used now
        motorBoardInfoArray.motorBoardInfo[i].temp2 = motorBoardData[i].ch1Temp;	  // driver channel 1 temperature
        motorBoardInfoArray.motorBoardInfo[i].temp3 = motorBoardData[i].ch2Temp;	  // driver channel 2 temperature
        motorBoardInfoArray.motorBoardInfo[i].volMain = motorBoardData[i].drvVoltage;  //main power voltage, default is battery voltage
        motorBoardInfoArray.motorBoardInfo[i].vol12V = motorBoardData[i].motVoltage;
        motorBoardInfoArray.motorBoardInfo[i].vol5V = motorBoardData[i].reg5Voltage;	  // 5V power
        motorBoardInfoArray.motorBoardInfo[i].dinput = 0;	  // digital input, not used now
        motorBoardInfoArray.motorBoardInfo[i].doutput = 0;	  // digital output, not used now
        motorBoardInfoArray.motorBoardInfo[i].ack = 0; 	  //not used now 0- right command received(receive "+") -1 wrong("-")

    }
    motorboardInfoArray_pub_.publish(motorBoardInfoArray);

}

void QNode::publisherGPSInfo(GPSData gpsData)
{
    jaguar::GPSInfo gpsInfo;
    gpsInfo.header.stamp = ros::Time::now();
    gpsInfo.header.frame_id = "/drrobot_jaguar_v6_gps";
    gpsInfo.status = gpsData.gpsState;
    gpsInfo.gpsTimeStamp = gpsData.gpsTimeStamp;
    gpsInfo.latitude = gpsData.gpsLat;  //
    gpsInfo.longitude = gpsData.gpsLong;
    gpsInfo.vog = gpsData.gpsVog;
    gpsInfo.cog = gpsData.gpsCog;
    gpsInfo_pub_.publish(gpsInfo);
}

void QNode::publisherIMUData(IMUData imuData)
{
    jaguar::IMUInfo imuInfo;
    imuInfo.header.stamp = ros::Time::now();
    imuInfo.header.frame_id = "/drrobot_jaguar_v6_imu";
    imuInfo.seq = imuData.seqNo;  //0 ~ 255
    imuInfo.yaw = imuData.estYaw;	//radian
    imuInfo.pitch = -atan2(imuData.accelRaw[0],sqrt(imuData.accelRaw[1]*imuData.accelRaw[1]+imuData.accelRaw[2]*imuData.accelRaw[2])); //0; //not used now
    imuInfo.roll = ((atan2(-imuData.accelRaw[1],sqrt(imuData.accelRaw[0]*imuData.accelRaw[0]+imuData.accelRaw[2]*imuData.accelRaw[2]))*180)/M_PI); //0; //not used now
    imuInfo.gyro_x = imuData.gyroRaw[0];
    imuInfo.gyro_y = imuData.gyroRaw[1];
    imuInfo.gyro_z = imuData.gyroRaw[2];
    imuInfo.accel_x = imuData.accelRaw[0];
    imuInfo.accel_y = imuData.accelRaw[1];
    imuInfo.accel_z = imuData.accelRaw[2];
    imuInfo.comp_x = imuData.compassRaw[0];
    imuInfo.comp_y = imuData.compassRaw[1];
    imuInfo.comp_z = imuData.compassRaw[2];
    imuInfo_pub_.publish(imuInfo);
}

void QNode::publisherFlipperData(FlipperData flipperData)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.name[0] ="left_rear_track_j";
    joint_state.position[0] = flipperData.Rearangle;
    joint_state.name[1] ="right_rear_track_j";
    joint_state.position[1] = flipperData.Rearangle;
    joint_state.name[2] ="left_front_track_j";
    joint_state.position[2] = - flipperData.Frontangle;
    joint_state.name[3] ="right_front_track_j";
    joint_state.position[3] = - flipperData.Frontangle;

    joint_pub.publish(joint_state);
}

void QNode::run() {
        ros::Rate loop_rate(10);
	int count = 0;
        while ( ros::ok() )
        {
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
	}
        std::cout << "Jaguar node shutdown" << std::endl;
        emit rosShutdown(); // used to signal a shutdown (useful to roslaunch)
}

/*void QNode::wheelCmdReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg)
{
    int cmdValue1 =  cmd->leftCmd;
    int cmdValue2 = cmd->rightCmd;
    std::cout << "Ros received motor command." << std::endl;
    emit wheelCmdUpdated(cmdValue1,cmdValue2);
}*/

void QNode::wheelCmdReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg)
{
    int cmdValue1 = 0;
    int cmdValue2 = 0;
    //std::cout << "Ros received motor command." << std::endl;
    /**if(cmd_msg->twist.linear.y > 2.){                                       //forward
        cmdValue1 = - static_cast<int>(cmd_msg->twist.linear.x);
        cmdValue2 = static_cast<int>(cmd_msg->twist.linear.x);
    }
    else if(cmd_msg->twist.linear.y < -2.){                                     //backward
        cmdValue1 = static_cast<int>(cmd_msg->twist.linear.x);
        cmdValue2 = - static_cast<int>(cmd_msg->twist.linear.x);
    }
    else if (cmd_msg->twist.angular.z > 0.2 && cmd_msg->twist.linear.y == 0.) { //left
        cmdValue1 = static_cast<int>(cmd_msg->twist.linear.x);
        cmdValue2 = static_cast<int>(cmd_msg->twist.linear.x);
    }
    else if (cmd_msg->twist.angular.z < -0.2 && cmd_msg->twist.linear.y == 0.) { //right
        cmdValue1 = - static_cast<int>(cmd_msg->twist.linear.x);
        cmdValue2 = - static_cast<int>(cmd_msg->twist.linear.x);
    }
    else{
        cmdValue1 = 0;
        cmdValue2 = 0;
    }**/
    cmdValue1 = - static_cast<int>(cmd_msg->twist.linear.x * vel) - static_cast<int>(cmd_msg->twist.angular.z * vel/3);
    cmdValue2 = static_cast<int>(cmd_msg->twist.linear.x * vel) - static_cast<int>(cmd_msg->twist.angular.z * vel/3);

    //ROS_INFO("cmd1: %d",cmdValue1);
    //ROS_INFO("cmd2: %d",cmdValue2);
    ROS_INFO("Speed: %d",static_cast<int>(cmd_msg->twist.linear.x));

    emit wheelCmdUpdated(cmdValue1,cmdValue2);
}



/*void QNode::flipCmdReceived(const drrobot_jaguar_v6::FlipMotorCmd::ConstPtr& cmd)
{
    std::cout << "Ros received flip motor command." << std::endl;
    emit flipCmdUpdated(cmd->leftFrontCmd,cmd->rightFrontCmd,cmd->leftRearCmd,cmd->rightRearCmd);

}
*/

void QNode::flipCmdReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg)
{
    //std::cout << "Ros received flip motor command." << std::endl;
    //ROS_INFO("previousFF: %d",static_cast<int>(cmd_msg->twist.linear.x*RADTOPULSE));
    //ROS_INFO("previousFR: %d",static_cast<int>(cmd_msg->twist.angular.x*RADTOPULSE));

    //ROS_INFO("previousFF: %d",static_cast<int>(cmd_msg->twist.linear.x));
    //ROS_INFO("previousFR: %d",static_cast<int>(cmd_msg->twist.angular.x));

    //int cmdFF = static_cast<int>(cmd_msg->twist.linear.x*RADTOPULSE) - previousFF;
    //int cmdFR = static_cast<int>(cmd_msg->twist.angular.x*RADTOPULSE) - previousFR;
    //previousFF = static_cast<int>(cmd_msg->twist.linear.x*RADTOPULSE);
    //previousFR = static_cast<int>(cmd_msg->twist.angular.x*RADTOPULSE);
    ROS_INFO("ENTRADO: %d", msgCnt);

    double cmdFF = static_cast<double>(cmd_msg->twist.linear.x *180/M_PI);
    double cmdFR = static_cast<double>(cmd_msg->twist.angular.x *180/M_PI);
    previousFF = cmdFF;
    previousFR = cmdFR;
    ROS_INFO("ENTRADO: %d, FF: %d, FR:%d", msgCnt, previousFF, previousFR);
    msgCnt ++;

}

void QNode::ButtonCmdReceived(const sensor_msgs::JoyConstPtr &joy)
{
    if(joy->buttons[1]){
        ROS_INFO("PULSADO 1");
        emit ReleaseEstopCmdUpdated();
    }
    else if(not(joy->buttons[1])){
        ROS_INFO(" NO PULSADO 1");
        emit EstopCmdUpdated();
    }
    if(joy->buttons[10]){
        emit FlipperCmdUpdated(10, 10);
        std::cout << "Reset Init" << std::endl;
    }
    if(joy->buttons[9]){
        emit SetInitCmdUpdated();
        std::cout << "Set Init" << std::endl;
    }
    if(joy->buttons[0] || joy->buttons[2] || joy->buttons[15] || joy->buttons[16]){
        ROS_INFO("UDATED: %d, %d", previousFF, previousFR);
        emit FlipperCmdUpdated(previousFF,previousFR);
    }
    if(joy->buttons[11]){
        std::cout << "Stairs auto motion" << std::endl;
        stairsDistance = true;
    }

}

void QNode::stairsApproachPhase(const chapter6_tutorials::Distance::ConstPtr& distance)
{
    if(stairsDistance){
        std::cout << "Stairs Approach" << std::endl;
        if(distance->distance1 > 900 && distance->distance2 > 900){
            ROS_INFO("Distance : %d", distance->distance1);
            emit StairsApproach(-150,150);
        }
        else {
            stairsDistance = false;
//            if(abs(distance->distance1 - distance->distance2) > 5){
//                ROS_INFO("Distance 1 : %d", distance->distance1);
//                ROS_INFO("Distance 2 : %d", distance->distance2);
//                if(distance->distance1 > distance->distance2){
//                    emit StairsApproach(-200,-200);
//                    ROS_INFO("Distance 1 > Distance 2");
//                }
//                else if(distance->distance1 < distance->distance2){
//                    emit StairsApproach(200,200);
//                    ROS_INFO("Distance 1 < Distance 2");
//                }
//            }
//            else {
                emit EstopCmdUpdated();
                ROS_INFO("Approach finished : %d", distance->distance1);
                //emit Flipper30UpCmdUpdated();
                ROS_INFO("Flipper Inclination");
                emit StairsTouch();
                return;
//            }

        }
    }
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
        case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	emit loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace drrobot_jaguar_v6
