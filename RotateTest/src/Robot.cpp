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
	state=1;
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();

	rots = (fabs(RobotMap::robotImu->GetAngle()) - fabs(initAngle)) / 360.0;
	if(state == 1) {
		SetLeftSpeed(1);
		SetRightSpeed(-1);
		if(rots > 10) {
			state = 2;
		}
	}
	else if(state == 2) {
		SetLeftSpeed(0);
		SetRightSpeed(0);

		double dist_native_front = (fabs(RobotMap::swerveSubsystemFLDriveTalon->GetSelectedSensorPosition(0)) + fabs(RobotMap::swerveSubsystemFRDriveTalon->GetSelectedSensorPosition(0))) / 2.0;
		double dist_native_back = (fabs(RobotMap::swerveSubsystemBLDriveTalon->GetSelectedSensorPosition(0)) + fabs(RobotMap::swerveSubsystemBRDriveTalon->GetSelectedSensorPosition(0))) / 2.0;
		double dist_front = (dist_native_front / tpr) * M_PI * wd;
		double dist_back = (dist_native_back / tpr) * M_PI * wd;

		trackwidth_front = dist_front / (rots * M_PI);
		trackwidth_back = dist_back / (rots * M_PI);

		std::cout << "EMPIRICAL TRACKWIDTH FRONT: " << trackwidth_front << " METERS" << std::endl;
		std::cout << "EMPIRICAL TRACKWIDTH BACK: " << trackwidth_back << " METERS" << std::endl;
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
	RobotMap::swerveSubsystemFRDriveTalon->Set(ControlMode::PercentOutput, tempSpeed);
	RobotMap::swerveSubsystemBRDriveTalon->Set(ControlMode::PercentOutput, tempSpeed);
}

START_ROBOT_CLASS(Robot);
