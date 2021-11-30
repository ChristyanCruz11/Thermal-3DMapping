#ifndef DRROBOTPROTOCOL_H
#define DRROBOTPROTOCOL_H
//some robot setting
#define WHEEL_R	0.12
#define WHEEL_DIS 0.5
#define MOTOR_CNT	300
#define MOTOR_DIR	-1
#define PI		3.14159
#define KNNOT2MS  0.514444444
#define LEFTFRONTFLIP_CMD   30
#define RIGHTFRONTFLIP_CMD   30
#define LEFTREARFLIP_CMD   30
#define RIGHTREARFLIP_CMD   30
#define FLIPARM_CIRCLE_CNT  7600
#define RADTOPULSE  891 // 891 motor pulsations for 1 radian

struct  MotorData
{
    int motorPower;
    int encoderSpeed;
    int encoderPos;
    int encoderDir;
    double motorAmp;        //current
    double motorTemp;       //temperature
    double angle;
    bool iniFlag;
    int preEncoder;
    int restPos;
};

struct MotorBoardData
{
    double drvVoltage;
    double motVoltage;
    double reg5Voltage;
    double ch1Temp;
    double ch2Temp;
    int driverState;
};

struct IMUData
{
    int seqNo;
    double estYaw;
    int gyroRaw[3];
    int compassRaw[3];
    int accelRaw[3];
};

struct GPSData
{
    int gpsState;
    double gpsLat;
    double gpsLong;
    double gpsCog;
    double gpsVog;
    double gpsTimeStamp;

};

struct FlipperData
{
    double Frontangle;
    double Rearangle;

};

#endif // DRROBOTPROTOCOL_H
