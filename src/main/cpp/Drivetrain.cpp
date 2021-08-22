// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include "frc/geometry/Rotation2d.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {

  frc::SmartDashboard::PutNumber("gyro angle", m_gyro->GetAngle());
  frc::SmartDashboard::PutNumber("gyro yaw", m_gyro->GetYaw());
  frc::SmartDashboard::PutNumber("gyro pitch", m_gyro->GetPitch());
  frc::SmartDashboard::PutNumber("gyro roll", m_gyro->GetRoll());
  frc::Rotation2d gyroRotation = units::degree_t{-m_gyro->GetAngle()};
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, gyroRotation)
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;


  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  //Temporary Place to Output Values
  m_frontLeft.OutputValues();
  m_frontRight.OutputValues();
  m_backLeft.OutputValues();
  m_backRight.OutputValues();
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro->GetRotation2d(), m_frontLeft.GetState(),
                    m_frontRight.GetState(), m_backLeft.GetState(),
                    m_backRight.GetState());
}
