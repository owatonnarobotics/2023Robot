#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "Consts.h"
#include <frc/AnalogPotentiometer.h>

class Arm {

    public:
        static Arm& GetInstance() {
            static Arm* instance = new Arm(R_ArmActuatorCANID1, R_ArmActuatorCANID2, R_ArmPotentiometer);
            return *instance;
        }

        void ArmSpeed(double speedToSet) {
            m_ArmActuator1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
            m_ArmActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }
        void ArmSetPosition(double positionToSet){
            while(m_ArmPotentiometer->Get() <= positionToSet){
                m_ArmActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
            }
            m_ArmActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        }
        void ArmSetPositionAuto(double positionToSet){
            while(m_ArmPotentiometer->Get() <= positionToSet){
                m_ArmActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
            }
            m_ArmActuator2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, .15);
        }

    private:
        Arm(const int R_ArmActuatorCANID1, const int R_ArmActuatorCANID2, const int R_ArmPotentiometer) {
            m_ArmActuator1 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ArmActuatorCANID1);
            m_ArmActuator2 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ArmActuatorCANID2);
            m_ArmPotentiometer = new frc::AnalogPotentiometer(R_ArmPotentiometer, 1.0, 0.0 );
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ArmActuator1;
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ArmActuator2;
        frc::AnalogPotentiometer *m_ArmPotentiometer;
};