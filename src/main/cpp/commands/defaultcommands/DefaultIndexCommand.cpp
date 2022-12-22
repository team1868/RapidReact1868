#include "commands/defaultcommands/DefaultIndexCommand.h"

#include <iostream>

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "utils/INDEXER_CONSTANTS.h"

// default indexer command
DefaultIndexCommand::DefaultIndexCommand(Indexer& indexer,
                                         SensorBoard& sensorControl,
                                         ControlBoard& humanControl)
    : _indexer{indexer}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&indexer});

    bool top    = _indexer.GetTopLightSensor();
    bool bottom = _indexer.GetBottomLightSensor();

    // timeouts
    _curTime = _sensorControl.GetCurTime();

    // figure out state first
    _nextIndexWheelState = kWheelIdle;
    _curIndexWheelState  = kWheelIdle;
    _nextIndexState      = kIdle;
    _curIndexState       = kIdle;
    if (top && bottom) {
        _curIndexState = kFull;
    } else if (!bottom && top) {
        _curIndexState = kReIndexing;
    } else if (bottom && !top) {
        _curIndexState = kIdle;
    }
}

void DefaultIndexCommand::Initialize()
{
    _indexer.SetIndexRollersPower(0.0);
    _indexer.SetElevatorPower(0.0);
    _alliance = _sensorControl.GetAlliance();
}

void DefaultIndexCommand::Execute()
{
    bool top         = _indexer.GetTopLightSensor();
    bool bottom      = _indexer.GetBottomLightSensor();
    bool indexSensor = _indexer.IndexIsBlocked();
    _curTime         = _sensorControl.GetCurTime();

    _nextIndexWheelState = kWheelIdle;
    _nextIndexState      = kIdle;

    // handle index wheel first
    if (_curIndexWheelState == kWheelIdle) {
        // check for next state
        if (!indexSensor) {
            double indexRollerPower = RESET_OUTPUT;
            if (_sensorControl.IsIntaking()) {
                indexRollerPower = _indexer.ColorSensorExists() ? 0.4 : INDEX_ROLLERS_OUTPUT;
            }
            _indexer.SetIndexRollersPower(indexRollerPower);
        } else if (_sensorControl.IsAuto() || _humanControl.GetDesired(Buttons::kDefenseButton)) {
            _startIndexWheelTime = _curTime;
            // Just index everything
            _nextIndexWheelState = kForward;
        } else {
            _startIndexWheelTime = _curTime;
            double rbRatio       = _indexer.GetRedBlueRatio();
            _nextIndexWheelState =
                ((_alliance == frc::DriverStation::kBlue && rbRatio > _indexer.GetColorRatio()) ||
                 (_alliance == frc::DriverStation::kRed && rbRatio < _indexer.GetColorRatio()))
                    ? kBackward
                    : kForward;
        }
    } else if (_curIndexWheelState == kForward &&
               _curTime - _startIndexWheelTime < _indexWheelTimeout) {
        // not timed out
        _indexer.SetIndexRollersPower(0.7);
        _nextIndexWheelState = kForward;

    } else if (_curIndexWheelState == kBackward &&
               _curTime - _startIndexWheelTime < _indexWheelTimeout) {
        // not timed out
        _indexer.SetIndexRollersPower(REV_INDEX_ROLLERS_OUTPUT);
        _nextIndexWheelState = kBackward;
    } else {
        _indexer.SetIndexRollersPower(RESET_OUTPUT);
    }

    // handling ball elevator
    if (_curIndexState == kIdle || _curIndexState == kFull) {  // can change state
        _indexer.SetElevatorPower(RESET_OUTPUT);
        if (top && bottom) {
            _nextIndexState = kFull;
        } else if (_nextIndexWheelState == kForward && !(top && bottom)) {
            // something at index and not full
            _nextIndexState = kIndexingUp;
            _startIndexTime = _curTime;
        } else if (top && !bottom) {
            // ball at top but needs to be lowered to bottom
            _nextIndexState   = kReIndexing;
            _startReIndexTime = _curTime;
        }
    } else if (_curIndexState == kReIndexing) {
        if (indexSensor && !top) {
            // something just arrived and can move up, switch to indexing up
            _nextIndexState = kIndexingUp;
            _startIndexTime = _curTime;
        } else if (!bottom && _curTime - _startReIndexTime < _reIndexTimeout) {
            // nothing at bottom, keep on going down
            _indexer.SetElevatorPower(-0.3);
            _nextIndexState = kReIndexing;
        } else {
            // timed out or something at bottom
            _indexer.SetElevatorPower(RESET_OUTPUT);
        }
    } else if (_curIndexState == kIndexingUp) {
        if (!top && _curTime - _startIndexTime < _indexTimeout) {
            // nothing at top and not timed out, continue
            _indexer.SetElevatorPower(0.3);
            _nextIndexState = kIndexingUp;
        } else {
            // top or timed out, stop
            _indexer.SetElevatorPower(RESET_OUTPUT);
        }
    }

    _curIndexWheelState = _nextIndexWheelState;
    _curIndexState      = _nextIndexState;

    if (top && bottom /* && _sensorControl.GetTurretLEDOverride()*/) {
        _sensorControl.SetIndexLEDDesired(1);
    } else if (top || bottom /* && _sensorControl.GetTurretLEDOverride()*/) {
        _sensorControl.SetIndexLEDDesired(0);
    } else {
        _sensorControl.SetIndexLEDDesired(-1);
    }
}

void DefaultIndexCommand::End(bool interrupted)
{
    _indexer.SetIndexRollersPower(RESET_OUTPUT);
    _indexer.SetElevatorPower(RESET_OUTPUT);
}
