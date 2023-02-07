// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <math.h>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

#include "swerve/src/include/SwerveTrain.h"
#include "controller/Controller.h"

frc::Joystick* playerOne;
frc::XboxController* playerTwo;

void Robot::RobotInit() {
playerOne = new frc::Joystick(R_controllerPortPlayerOne);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
    SwerveTrain::GetInstance().ResetHold();
    // TODO: NEEDS TO CHANGE
    SwerveTrain::GetInstance().HardwareZero();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);
}

void Robot::TeleopPeriodic() {
  if (playerOne->GetRawButton(9) && playerOne->GetRawButton(10)) {

        SwerveTrain::GetInstance().HardwareZero();
    }
    if (playerOne->GetRawButton(4)) {

        NavX::GetInstance().Calibrate();
        NavX::GetInstance().resetYaw();
    }
    if (playerOne->GetRawButton(7)) {
        
        SwerveTrain::GetInstance().AssumeZeroPosition();
    }
    else {

        double x = playerOne->GetX();
        double y = playerOne->GetY();
        double z = playerOne->GetZ();
        Controller::forceControllerXYZToZeroInDeadzone(x, y, z);

        if (playerOne->GetRawButton(2)) {

            frc::SmartDashboard::PutNumber("z", z);
        }
        else {

            z *= R_controllerZMultiplier;
        }

        SwerveTrain::GetInstance().Drive(
            -x,
            -y,
            z,
            playerOne->GetRawButton(5),
            playerOne->GetRawButton(3),
            -(((playerOne->GetThrottle() + 1.0) / 2.0) - 1.0)
        );
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  SwerveTrain::GetInstance().SetSwerveBrake(false);
  SwerveTrain::GetInstance().SetDriveBrake(false);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
