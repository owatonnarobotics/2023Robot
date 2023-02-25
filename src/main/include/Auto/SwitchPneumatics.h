#pragma once

#include "commonauto/AutoStep.h"

#include "Claw.h"

class SwitchPneumatics : public AutoStep {

    public:
        SwitchPneumatics() : AutoStep("SwitchPneumatics") {

        
        }

        void Init() {}

        bool Execute() {

            Claw::GetInstance().SwitchPneumatics();
            return true;
        }

    private:

};