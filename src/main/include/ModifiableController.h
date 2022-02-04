#pragma once
#include <math.h>
#include <string>
#include <frc/XboxController.h>
class ModifiableController: public frc::XboxController {
    public:
        bool deadzoneActive = false;
        double deadzoneThreshhold = 0.1;
        double inputSensitivity = 1;
        bool sensitivityActive = false;


        ModifiableController(int controllerIndex)
            :XboxController(controllerIndex)
        {

        }
        double GetLeftY(std::string effect = "") {return applyEffects(XboxController::GetLeftY(), effect);}
        double GetRightY(std::string effect = "") {return applyEffects(XboxController::GetRightY(), effect);}
        double GetLeftX(std::string effect = "") {return applyEffects(XboxController::GetLeftX(), effect);}
        double GetRightX(std::string effect = "") {return applyEffects(XboxController::GetRightX(), effect);}
        double GetLeftTriggerAxis(std::string effect = "") {return applyEffects(XboxController::GetLeftTriggerAxis(), effect);}
        double GetRightTriggerAxis(std::string effect = "") {return applyEffects(XboxController::GetRightTriggerAxis(), effect);}
        
        void setDeadzone(bool target = true){
            deadzoneActive=target;
        }

        void setDeadzoneLimit(double target){
            deadzoneThreshhold=target;
        }

        void setSensitivity(bool target = true){
            sensitivityActive=target;
        }

        void setSensitivityLevel(double target){
            inputSensitivity=target;
        }

        double applyEffects(double originalOutput, std::string specialEffects){
            double adjustedOutput = originalOutput;

            bool deadzoneOutput = ( (specialEffects != "og") && (specialEffects != "noDZ") );
            bool sensitivityOutput = ( (specialEffects != "og") && (specialEffects != "noS") );
            if(deadzoneActive && deadzoneOutput){
                adjustedOutput = Deadzone(adjustedOutput);
            }
            if(sensitivityActive && sensitivityOutput){
                adjustedOutput = adjustedOutput*inputSensitivity;
            }
            return adjustedOutput;
        };



    double Deadzone(double amm){
        //deadzoneLimit is arbitrary
        //make it smartdashboard
        if (abs(amm) < deadzoneThreshhold){
        amm = 0;
        }
        
        return amm;
    }
};