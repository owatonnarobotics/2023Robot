#pragma once
#include "Arm.h"
#include "commonauto/AutoStep.h"

class SetArm2 : public AutoStep {

    public:
        SetArm2(const double speed) : AutoStep("SetArm2") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Arm::GetInstance().ArmSetPositionAuto(m_speed);
            return true;
        }

    private:
        double m_speed;
};