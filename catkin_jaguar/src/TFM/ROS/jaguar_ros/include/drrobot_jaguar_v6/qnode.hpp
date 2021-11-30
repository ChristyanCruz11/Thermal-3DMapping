/**
 * @file /include/drobot_jaguar_v6/qnode.hpp
 *
 * @brief Communications central!

 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef drrobot_jaguar_v6_QNODE_HPP_
#define drrobot_Jaguar_v6_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <jaguar/BaseMotorCmd.h>
#include <jaguar/FlipMotorCmd.h>
#include "drrobotprotocol.hpp"
#include <chapter6_tutorials/Distance.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drrobot_jaguar_v6 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

 
    void publisherMotorData(MotorData motorData[],int len);
    void publisherMotorBoardInfoArray(MotorBoardData motorBoardData[],int len);
    void publisherFlipperData(FlipperData flipperData);
    void publisherGPSInfo(GPSData gpsData);
    void publisherIMUData(IMUData imuData);
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

signals:
	void loggingUpdated();
    	void rosShutdown();
    void wheelCmdUpdated(int,int);
    void flipCmdUpdated(int,int);
    void EstopCmdUpdated();
    void ReleaseEstopCmdUpdated();
    void SetInitCmdUpdated();
    void ResetInitCmdUpdated();
    void Flipper30UpCmdUpdated();
    void FlipperCmdUpdated(double,double);
    void Flipper60DownCmdUpdated();
    void StairsApproach(int,int);
    void StairsTouch();
    //void FlipInclination(double);

private:
	int init_argc;
	char** init_argv;


    ros::Publisher motorInfo_pub_;
    ros::Publisher motorboardInfoArray_pub_;
    ros::Publisher gpsInfo_pub_;
    ros::Publisher imuInfo_pub_;
    ros::Publisher joint_pub;

    ros::Subscriber motor_cmd_sub_;
    ros::Subscriber flipmotor_cmd_sub_;
    ros::Subscriber joy_subscriber;
    ros::Subscriber stairs_distance_subscriber;

    int msgCnt;
    QStringListModel logging_model;
    bool stairsDistance;
    int previousFF;
    int previousFR;

    //void wheelCmdReceived(const drrobot_jaguar_v6::BaseMotorCmd::ConstPtr& cmd);
    //void flipCmdReceived(const drrobot_jaguar_v6::FlipMotorCmd::ConstPtr& cmd);
    void wheelCmdReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg);
    void flipCmdReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd_msg);
    void ButtonCmdReceived(const sensor_msgs::JoyConstPtr &joy);
    void stairsApproachPhase(const chapter6_tutorials::Distance::ConstPtr& distance);
};

}  // namespace drrobot_jaguar_v6

#endif /* drrobot_jaguar_v6_QNODE_HPP_ */
