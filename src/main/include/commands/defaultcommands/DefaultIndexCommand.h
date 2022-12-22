#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class ControlBoard;
class SensorBoard;

class DefaultIndexCommand : public frc2::CommandHelper<frc2::CommandBase, DefaultIndexCommand> {
   public:
    enum IndexingState { kIndexingUp, kFull, kIdle, kReIndexing };

    enum IndexWheelState { kWheelIdle, kForward, kBackward };

    explicit DefaultIndexCommand(Indexer& indexer,
                                 SensorBoard& sensorControl,
                                 ControlBoard& humanControl);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

   private:
    Indexer& _indexer;
    SensorBoard& _sensorControl;
    ControlBoard& _humanControl;

    double _curTime;
    double _startReIndexTime = 0.0, _startIndexTime = 0.0, _reIndexTimeout = 3.0,
           _indexTimeout = 3.0, _startIndexWheelTime = 0.0, _indexWheelTimeout = 0.5;

    IndexingState _curIndexState, _nextIndexState;
    IndexWheelState _curIndexWheelState, _nextIndexWheelState;

    frc::DriverStation::Alliance _alliance = frc::DriverStation::kInvalid;
};
