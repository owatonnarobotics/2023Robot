#pragma once

#include "commonauto/AutoStep.h"

#include "Claw.h"

class SetClaw : public AutoStep {

    public:
        SetClaw(const double speed) : AutoStep("SetClaw") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Claw::GetInstance().ClawTiltSpeed(m_speed);
            return true;
        }

    private:
        double m_speed;
};