#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "Consts.h"
#include <frc/Solenoid.h>
#include <frc/AnalogPotentiometer.h>

class Claw {

    public:
        static Claw& GetInstance() {
            static Claw* instance = new Claw(R_ClawSolenoid1, R_ClawActuatorCANID, R_ClawPotentiometer);
            return *instance;
        }

        void ClawTiltSpeed(double speedToSet) {
            m_ClawActuator->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }
        void ClawTiltPosition(double distanceToSet){
            while(m_ClawPotentiometer->Get() <= distanceToSet){
                m_ClawActuator->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
            }
            m_ClawActuator->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, .1);
        }
        void SwitchPneumatics(){
            m_ClawSolenoid1->Toggle();
        }
    
    private:
        Claw(const int R_ClawSolenoid1, const int R_ClawActuatorCANID, const int R_ClawPotentiometer) {
            m_ClawActuator = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClawActuatorCANID);
            m_ClawSolenoid1 = new frc::Solenoid {frc::PneumaticsModuleType::CTREPCM, R_ClawSolenoid1};
            m_ClawPotentiometer = new frc::AnalogPotentiometer(R_ClawPotentiometer, 1.0, 0.0 );
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClawActuator;
        frc::Solenoid *m_ClawSolenoid1;
        frc::AnalogPotentiometer *m_ClawPotentiometer;
};