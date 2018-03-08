#include "RobotMap.h"

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLDriveTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRDriveTalon;

std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFLRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemFRRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBLRotTalon;
std::shared_ptr<can::TalonSRX> RobotMap::swerveSubsystemBRRotTalon;

std::shared_ptr<frc::PowerDistributionPanel> RobotMap::pdp;
std::shared_ptr<AHRS> RobotMap::robotImu;

void RobotMap::init() {
	swerveSubsystemFLDriveTalon.reset(new can::TalonSRX(2));
	swerveSubsystemFRDriveTalon.reset(new can::TalonSRX(3));
	swerveSubsystemBLDriveTalon.reset(new can::TalonSRX(4));
	swerveSubsystemBRDriveTalon.reset(new can::TalonSRX(5));

	swerveSubsystemFLRotTalon.reset(new can::TalonSRX(6));
	swerveSubsystemFRRotTalon.reset(new can::TalonSRX(7));
	swerveSubsystemBLRotTalon.reset(new can::TalonSRX(8));
	swerveSubsystemBRRotTalon.reset(new can::TalonSRX(9));

	pdp.reset(new frc::PowerDistributionPanel());

	robotImu.reset(new AHRS(frc::SPI::Port::kMXP));

	//DRIVE TALONS
	swerveSubsystemFLDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	swerveSubsystemFRDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	swerveSubsystemBLDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
	swerveSubsystemBRDriveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);

	swerveSubsystemFLDriveTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemFRDriveTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemBLDriveTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemBRDriveTalon->SetSelectedSensorPosition(0, 0, 10);

	swerveSubsystemFLDriveTalon->SetSensorPhase(false);
	swerveSubsystemFRDriveTalon->SetSensorPhase(false);
	swerveSubsystemBLDriveTalon->SetSensorPhase(false);
	swerveSubsystemBRDriveTalon->SetSensorPhase(false);

	swerveSubsystemFLDriveTalon->SetInverted(false);
	swerveSubsystemFRDriveTalon->SetInverted(false);
	swerveSubsystemBLDriveTalon->SetInverted(false);
	swerveSubsystemBRDriveTalon->SetInverted(false);

	swerveSubsystemFLDriveTalon->ConfigVoltageCompSaturation(12, 10);
	swerveSubsystemFRDriveTalon->ConfigVoltageCompSaturation(12, 10);
	swerveSubsystemBLDriveTalon->ConfigVoltageCompSaturation(12, 10);
	swerveSubsystemBRDriveTalon->ConfigVoltageCompSaturation(12, 10);

	swerveSubsystemFLDriveTalon->EnableVoltageCompensation(true);
	swerveSubsystemFRDriveTalon->EnableVoltageCompensation(true);
	swerveSubsystemBLDriveTalon->EnableVoltageCompensation(true);
	swerveSubsystemBRDriveTalon->EnableVoltageCompensation(true);

	//ROTATION TALONS
	swerveSubsystemFLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemFRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemBLRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	swerveSubsystemBRRotTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

	swerveSubsystemFLRotTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemFRRotTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemBLRotTalon->SetSelectedSensorPosition(0, 0, 10);
	swerveSubsystemBRRotTalon->SetSelectedSensorPosition(0, 0, 10);

	swerveSubsystemFLRotTalon->SetSensorPhase(false);
	swerveSubsystemFRRotTalon->SetSensorPhase(false);
	swerveSubsystemBLRotTalon->SetSensorPhase(false);
	swerveSubsystemBRRotTalon->SetSensorPhase(false);

	swerveSubsystemFLRotTalon->SetInverted(true);
	swerveSubsystemFRRotTalon->SetInverted(true);
	swerveSubsystemBLRotTalon->SetInverted(true);
	swerveSubsystemBRRotTalon->SetInverted(true);

	swerveSubsystemFLRotTalon->ConfigVoltageCompSaturation(12, 10);
	swerveSubsystemFRRotTalon->ConfigVoltageCompSaturation(12, 10);
	swerveSubsystemBLRotTalon->ConfigVoltageCompSaturation(12, 10);
	swerveSubsystemBRRotTalon->ConfigVoltageCompSaturation(12, 10);

	swerveSubsystemFLRotTalon->EnableVoltageCompensation(true);
	swerveSubsystemFRRotTalon->EnableVoltageCompensation(true);
	swerveSubsystemBLRotTalon->EnableVoltageCompensation(true);
	swerveSubsystemBRRotTalon->EnableVoltageCompensation(true);
}
