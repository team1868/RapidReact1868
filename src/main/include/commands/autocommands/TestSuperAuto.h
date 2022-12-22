#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class Indexer;
class Intake;
class Shooter;
class SensorBoard;

class TestSuperAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, TestSuperAuto> {
   public:
    TestSuperAuto(Shooter& shooter, Indexer& indexer, Intake& intake, SensorBoard& sensorControl);
};
