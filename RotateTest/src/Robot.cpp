#include "Robot.h"
#include "RobotMap.h"
#include <iostream>
#include <math.h>

void Robot::RobotInit() {
	std::cout << "Robot is starting!" << std::endl;
	RobotMap::init();
}

void Robot::DisabledInit() {
	std::cout << "Disabled!" << std::endl;
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	std::cout << "Autonomous Init!" << std::endl;
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	std::cout << "Teleop Init!" << std::endl;
	initAngle = RobotMap::robotImu->GetAngle();
	wd = .0635;
	tpr = 26214;
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();

	rots = (fabs(RobotMap::robotImu->GetAngle()) - fabs(initAngle)) / 360.0;
	if(state == 1) {
		SetLeftSpeed(.5);
		SetRightSpeed(-.5);
		if(rots > 10) {
			state = 2;
		}
	}
	else if(state == 2) {
		SetLeftSpeed(0);
		SetRightSpeed(0);

		double dist_native = (fabs(RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0)) + fabs(RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0))) / 2.0;
		double dist = (dist_native / tpr) * M_PI * wd;

		trackwidth = dist / (rots * M_PI);

		std::cout << "EMPIRICAL TRACKWIDTH: " << trackwidth << " METERS" << std::endl;
		state = 3;
	}
}

void Robot::TestPeriodic() {

}

void Robot::SetLeftSpeed(double tempSpeed)
{
	RobotMap::swerveSubsystemFLDriveTalon->Set(ControlMode::PercentOutput, tempSpeed);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ControlMode::PercentOutput, tempSpeed);
}

void Robot::SetRightSpeed(double tempSpeed) {
	RobotMap::swerveSubsystemFLDriveTalon->Set(ControlMode::PercentOutput, tempSpeed);
	RobotMap::swerveSubsystemBLDriveTalon->Set(ControlMode::PercentOutput, tempSpeed);
}

START_ROBOT_CLASS(Robot);
