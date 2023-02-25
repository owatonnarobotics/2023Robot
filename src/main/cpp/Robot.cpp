// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Claw.h"
#include "Arm.h"
#include <math.h>

#include <fmt/core.h> 

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include "limelight/Limelight.h"
#include "swerve/src/include/SwerveTrain.h"
#include "controller/Controller.h"
#include "commonauto/AutoSequence.h"
#include "commonauto/steps/WaitSeconds.h"
#include "commonauto/steps/TimeDriveHold.h"
#include "commonauto/steps/TurnToAbsoluteAngle.h"
#include "commonauto/steps/Stop.h"
#include "commonauto/steps/ResetNavXYaw.h"
#include "commonauto/steps/CalibrateNavXThenReset.h"
#include "Auto/SetArm.h"
#include "Auto/SetArm2.h"
#include "Auto/SetClaw.h"
#include "Auto/SetClaw2.h"
#include "Auto/SwitchPneumatics.h"


frc::Joystick* playerOne;
frc::XboxController* playerTwo;
AutoSequence* bigSequence;
frc::SendableChooser<std::string>* autoChooser;

void Robot::RobotInit() {
playerOne = new frc::Joystick(R_controllerPortPlayerOne);
playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);
frc::SmartDashboard::PutNumber("Distance of the Arm Acuator", .35);
frc::SmartDashboard::PutNumber("Distance of the Claw Acuator", .15);
frc::CameraServer::StartAutomaticCapture();

autoChooser = new frc::SendableChooser<std::string>;
autoChooser->AddOption("one cone", "one cone");
autoChooser->AddOption("cube and platform", "cp");
autoChooser->AddOption("two ball sides", "2bs");
autoChooser->SetDefaultOption("tri-ball", "tb");
frc::SmartDashboard::PutData(autoChooser);

bigSequence = new AutoSequence(false);
bigSequence->EnableLogging();
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
    SwerveTrain::GetInstance().HardwareZero();
    bigSequence->Reset();

    std::string selectedAuto = autoChooser->GetSelected();
    
    if (selectedAuto == "one cone") {
    bigSequence->AddStep(new CalibrateNavXThenReset);
    bigSequence->AddStep(new WaitSeconds(1));
    bigSequence->AddStep(new ResetNavXYaw);


    bigSequence->AddStep(new SetArm2(.5));
    bigSequence->AddStep(new SetClaw2(.3));
    //bigSequence->AddStep(new WaitSeconds(.75));
    //bigSequence->AddStep(new TimeDriveHold(0, .25, 2));
    bigSequence->AddStep(new SwitchPneumatics());
    bigSequence->AddStep(new WaitSeconds(.5));
    bigSequence->AddStep(new TimeDriveHold(0, -.5, 3.5));
    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);

    bigSequence->AddStep(new WaitSeconds(20));
    }
    else if(selectedAuto == "cp"){
      bigSequence->AddStep(new CalibrateNavXThenReset);
      bigSequence->AddStep(new WaitSeconds(1));
      bigSequence->AddStep(new ResetNavXYaw);


      bigSequence->AddStep(new SetArm2(.45));
      bigSequence->AddStep(new SetClaw2(.3));
      //bigSequence->AddStep(new WaitSeconds(.75));
      //bigSequence->AddStep(new TimeDriveHold(0, .25, 2));
      bigSequence->AddStep(new SwitchPneumatics());
      bigSequence->AddStep(new WaitSeconds(.5));
      bigSequence->AddStep(new SetClaw(1));
      bigSequence->AddStep(new SetArm(1));
      bigSequence->AddStep(new TimeDriveHold(0, -.75, 2.5));
    }
    bigSequence->AddStep(new Stop);
    bigSequence->Init();


}

void Robot::AutonomousPeriodic() {
  /* if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  } */
  bigSequence->Execute();
  
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

  if(playerTwo->GetAButtonPressed()){
    Arm::GetInstance().ArmSetPosition(frc::SmartDashboard::GetNumber("Distance of the Arm Acuator", .35));
  }
  else if(playerTwo->GetLeftTriggerAxis() > .25){
    Arm::GetInstance().ArmSpeed(playerTwo->GetLeftTriggerAxis()); // arm up
  }
  else if(playerTwo->GetRightTriggerAxis() > .25){
    Arm::GetInstance().ArmSpeed(-playerTwo->GetRightTriggerAxis()); // arm down
  }
  else{
    Arm::GetInstance().ArmSpeed(0);
  }

  if(playerTwo->GetBackButton()){
    Arm::GetInstance().ArmSetPosition(frc::SmartDashboard::GetNumber("Distance of the Claw Acuator", .15));
  }
  else if(playerTwo->GetRightBumper()){

    Claw::GetInstance().ClawTiltSpeed(-1); // claw tilt up
  }
  else if(playerTwo->GetLeftBumper()){
    Claw::GetInstance().ClawTiltSpeed(1); // claw tilt down
  }
  else{
    Claw::GetInstance().ClawTiltSpeed(0);
  }
  
  if(playerTwo->GetYButtonPressed()){
    Claw::GetInstance().SwitchPneumatics();
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
