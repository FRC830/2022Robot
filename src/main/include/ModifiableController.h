#pragma once
#include <math.h>
#include <string>
#include <frc/XboxController.h>
class ModifiableController: public frc::XboxController {
    public:
        bool deadzoneActive = false;
        double deadzoneThreshhold = 0.1;



        ModifiableController(int motorIndex)
            :XboxController(motorIndex)
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
        double applyEffects(double originalOutput, std::string specialEffects){
            if(specialEffects == ""){
                if(deadzoneActive){
                    return Deadzone(originalOutput);
                }
                return originalOutput;
                 
            }
            return originalOutput;
        }

    double Deadzone(double amm){
    //deadzoneLimit is arbitrary
    //make it smartdashboard
    if (abs(amm) < deadzoneThreshhold){
      amm = 0;
    }
    
    return amm;
}
};