#pragma once

#include "commonauto/AutoStep.h"

#include "Arm.h"

class SetArm : public AutoStep {

    public:
        SetArm(const double speed) : AutoStep("SetArm") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Arm::GetInstance().ArmSpeed(m_speed);
            return true;
        }

    private:
        double m_speed;
};