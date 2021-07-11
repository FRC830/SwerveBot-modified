// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless) {

  // Set the conversion factor for the drive encoder to the distance (in meters)
  // travelled for each full "tick" of the drive encoder.
  // By default, GetVelocity() returns RPM (motor rev / min), so to convert to m/sec:
  //
  // motor rev     1 min           1 wheel rev          2*pi*kWheelRadius m     m
  // ---------  *  ------ * ------------------------- * -------------------  = ---
  //    min        60 sec   kDriveGearRatio motor rev       1 wheel rev        sec
  m_driveMotor.GetEncoder().SetVelocityConversionFactor(
      (1.0 / 60.0) * (1 / kDriveGearRatio) * (2 * wpi::math::pi * kWheelRadius));

  // Set the conversion factor for the turning encoder to the angle (in radians)
  // that the module turns for each full "tick" of the turning encoder.
  // By default, GetPosition() returns motor revolutions, so to convert to radians:
  //
  //                     1 wheel rev           2*pi radians
  // motor rev * --------------------------- * ------------ = radians
  //             kTurningGearRatio motor rev   1 wheel rev
  m_turningMotor.GetEncoder().SetPositionConversionFactor(
      2 * wpi::math::pi / kTurningGearRatio);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::math::pi),
                                               units::radian_t(wpi::math::pi));
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveMotor.GetEncoder().GetVelocity()},
          frc::Rotation2d(units::radian_t(m_turningMotor.GetEncoder().GetPosition()))};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) {
  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveMotor.GetEncoder().GetVelocity(), state.speed.to<double>());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(m_turningMotor.GetEncoder().GetPosition()), state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
