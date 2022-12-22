#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;

class DefaultClimbCommand : public frc2::CommandHelper<frc2::CommandBase, DefaultClimbCommand> {
   public:
    explicit DefaultClimbCommand(Climber& climber);

    void Initialize() override;

   private:
    Climber& _climber;
};