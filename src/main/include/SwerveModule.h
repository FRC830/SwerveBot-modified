// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class SwerveModule {
 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr double kDriveGearRatio = 6.6;
  static constexpr double kTurningGearRatio = 12.8;

  static constexpr auto kModuleMaxAngularVelocity =
      wpi::math::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::math::pi * 2_rad_per_s / 1_s;  // radians per second^2
   std::string m_driveMotorName;
   std::string m_turningMotorName;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, std::string driveMotorName, std::string turningMotorName);
  frc::SwerveModuleState GetState();
  void SetDesiredState(const frc::SwerveModuleState& state);


  // Motor Values
  rev::CANSparkMax m_driveMotor;
  rev::CANEncoder m_driveMotorEncoder = m_driveMotor.GetEncoder();
  double m_driveMotorVelocity = m_driveMotorEncoder.GetVelocity();
  double m_driveMotorRotations = m_driveMotorEncoder.GetPosition();
  ctre::phoenix::sensors::CANCoder m_driveMotorCANCoder {55}; //arbitrary CANCoder id
  double m_driveMotorAbsolutePosition = m_driveMotorCANCoder.GetAbsolutePosition();

  rev::CANSparkMax m_turningMotor;
  rev::CANEncoder m_turningMotorEncoder = m_turningMotor.GetEncoder();
  double m_turningMotorVelocity = m_turningMotorEncoder.GetVelocity();
  double m_turningMotorRotations = m_turningMotorEncoder.GetPosition();
  ctre::phoenix::sensors::CANCoder m_turningMotorCANCoder {30}; //arbitrary id
  double m_turningMotorAbsolutePosition = m_turningMotorCANCoder.GetAbsolutePosition();

  frc2::PIDController m_drivePIDController{1.0, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};

  void OutputValues(){ //Outputs all the SmartDashboard widgets when called
      frc::SmartDashboard::PutNumber(m_driveMotorName + " Velocity: ", m_driveMotorVelocity);
      frc::SmartDashboard::PutNumber(m_driveMotorName + " Rotations: ", m_driveMotorRotations);
      frc::SmartDashboard::PutNumber(m_driveMotorName + " Absolute Position: ", m_driveMotorAbsolutePosition);
      frc::SmartDashboard::PutNumber(m_turningMotorName + " Velocity: ", m_turningMotorVelocity);
      frc::SmartDashboard::PutNumber(m_turningMotorName + " Rotations: ", m_turningMotorRotations);
      frc::SmartDashboard::PutNumber(m_turningMotorName + " Absolute Position: ", m_turningMotorAbsolutePosition);
  }
};
