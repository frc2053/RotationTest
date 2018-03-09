#ifndef _ROBOT_H
#define _ROBOT_H

#include "TimedRobot.h"
#include "Timer.h"
#include <fstream>
#include <sstream>

class Robot : public frc::TimedRobot {
public:
	virtual void RobotInit();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();
	void SetLeftSpeed(double tempSpeed);
	void SetRightSpeed(double tempSpeed);
private:
	int state;
	double rots;
	double initAngle;
	double trackwidth_front;
	double trackwidth_back;
	double wd;
	double tpr;
};

#endif
