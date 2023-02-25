#pragma once

#include "commonauto/AutoStep.h"

#include "Claw.h"

class SetClaw2 : public AutoStep {

    public:
        SetClaw2(const double speed) : AutoStep("SetClaw2") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Claw::GetInstance().ClawTiltPosition(m_speed);
            return true;
        }

    private:
        double m_speed;
};