#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"

class RobotMap {
public:
	static void init();

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLDriveTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRDriveTalon;

	static std::shared_ptr<can::TalonSRX> swerveSubsystemFLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemFRRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBLRotTalon;
	static std::shared_ptr<can::TalonSRX> swerveSubsystemBRRotTalon;
	static std::shared_ptr<frc::PowerDistributionPanel> pdp;
	static std::shared_ptr<AHRS> robotImu;
};
#endif
