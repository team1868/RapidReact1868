#include "subsystems/Indexer.h"

#include <float.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <iostream>

Indexer::Indexer()
{
#ifdef JO
    _indexRollerMotor.SetInverted(true);
    _elevatorMotor.SetInverted(false);
#else
    _elevatorMotor.SetInverted(false);
    _elevatorMotor.SetNeutralMode(NeutralMode::Brake);
    _indexRollerMotor.SetInverted(true);
    _indexRollerMotor.SetNeutralMode(NeutralMode::Brake);

    _colorSensorExists = _colorSensor.IsConnected();
    _curProximity      = 0;

    // currently color sensor 1
    switch (COLOR_SENSOR_NUMBER) {
        case (1):
            _minProximity  = 350;   // low values are far
            _maxProximity  = 2500;  // close values are close
            _blueTolerance = 0.3;
            _redTolerance  = 0.4;
            break;
        case (2):
            _minProximity  = 200;
            _maxProximity  = 1200;
            _blueTolerance = 0.3;
            _redTolerance  = 0.3;
            break;
        case (3):
            _minProximity  = 300;
            _maxProximity  = 2000;
            _blueTolerance = 0.3;
            _redTolerance  = 0.3;
            break;
        case (4):
            _minProximity  = 200;
            _maxProximity  = 1100;
            _blueTolerance = 0.3;
            _redTolerance  = 0.3;
            break;
    }
#endif

    ConfigureShuffleboard();
    std::cout << "end of indexer" << std::endl;
}
void Indexer::Periodic() { UpdateShuffleboard(); }

void Indexer::SetIndexRollersPower(double power) { _indexRollerMotor.Set(power); }
double Indexer::GetIndexRollersPower() { return _indexRollerMotor.Get(); }

void Indexer::SetElevatorPower(double power) { _elevatorMotor.Set(power); }
double Indexer::GetElevatorPower() { return _elevatorMotor.Get(); }

bool Indexer::GetTopLightSensor() { return !_topLightSensor.Get(); }
bool Indexer::GetBottomLightSensor() { return !_bottomLightSensor.Get(); }

bool Indexer::ColorSensorExists()
{
    bool connected = _colorSensor.IsConnected();
    _colorSensorDeadEntry.SetBoolean(connected);
    return connected;
}

double Indexer::GetColorRatio() { return _colorRatioEntry.GetDouble(0.91); }

double Indexer::GetRedBlueRatio()
{
    return _detectedColor.red / std::clamp<double>(fabs(_detectedColor.blue), 0.0000001, DBL_MAX);
}

// Matches color, returns string of color that color sensor senses
Indexer::BALL_COLOR Indexer::GetMatchColor()
{
#if defined(PRACTICEBOT) || defined(COMPBOT)
    if (_detectedColor.blue > _blueTolerance) {
        return BALL_COLOR::BLUE;
    } else if (_detectedColor.red > _redTolerance) {
        return BALL_COLOR::RED;
    }
#endif
    return BALL_COLOR::UNKNOWN;
}

bool Indexer::IndexIsBlocked()
{
#ifdef JO
    return _colorSensor.Get();
#else
    return (_curProximity >= _minProximity /* && _curProximity <= _maxProximity */);
#endif
}

void Indexer::UpdateColorSensors()
{
#if defined(PRACTICEBOT) || defined(COMPBOT)
    if (_colorSensorExists) {
        _detectedColor = _colorSensor.GetColor();
        _curProximity  = _colorSensor.GetProximity();
    } else {
        _detectedColor = frc::Color(0, 0, 0);
        _curProximity  = 0.0;
    }

    // print out current rgb values + proximity to shuffleboard
    _actualREntry.SetDouble(_detectedColor.red);
    _actualGEntry.SetDouble(_detectedColor.green);
    _actualBEntry.SetDouble(_detectedColor.blue);
    _proximityEntry.SetDouble(_curProximity);

    if (_resetEntry.GetBoolean(false)) {
        _blueEntry.SetBoolean(false);
        _redEntry.SetBoolean(false);
        _resetEntry.SetBoolean(false);
    }

    // use this to provide a threshold!
    if (_curProximity >= _minProximity && _curProximity <= _maxProximity) {
        _proximityBoolEntry.SetBoolean(true);
        switch (GetMatchColor()) {
            case (BALL_COLOR::BLUE): _blueEntry.SetBoolean(true); break;
            case (BALL_COLOR::RED): _redEntry.SetBoolean(true); break;
            default: break;
        }
    } else {
        _proximityBoolEntry.SetBoolean(false);
    }
#endif
}

void Indexer::ConfigureShuffleboard()
{
    auto& superstructureTab     = frc::Shuffleboard::GetTab("Superstructure");
    auto& tuneColorSensorLayout = superstructureTab.GetLayout("Tune Color Sensor", "List Layout");
    auto& setupTab              = frc::Shuffleboard::GetTab("Setup");

    _colorRatioEntry    = tuneColorSensorLayout.Add("Tune Color Ratio", 0.91).GetEntry();
    _curColorRatioEntry = tuneColorSensorLayout.Add("Cur Color Ratio", 1.0).GetEntry();

    _topLightSensorEntry    = setupTab.Add("Top Light Sensor", false).GetEntry();
    _bottomLightSensorEntry = setupTab.Add("Bottom Light Sensor", false).GetEntry();
    _colorSensorDeadEntry   = frc::Shuffleboard::GetTab("Drive Team Control")
                                .Add("Color Sensor Working", _colorSensor.IsConnected())
                                .WithSize(2, 1)
                                .WithPosition(0, 1)
                                .GetEntry();

#if defined(PRACTICEBOT) || defined(COMPBOT)

    auto& actualValuesLayout = superstructureTab.GetLayout("Actual RGB + Proximity", "List Layout");
    _actualREntry            = actualValuesLayout.Add("Actual R", 0.0).GetEntry();
    _actualGEntry            = actualValuesLayout.Add("Actual G", 0.0).GetEntry();
    _actualBEntry            = actualValuesLayout.Add("Actual B", 0.0).GetEntry();
    _proximityEntry          = actualValuesLayout.Add("Proximity", 0.0).GetEntry();
    _proximityBoolEntry      = actualValuesLayout.Add("Proximity SEEN", false).GetEntry();

    _blueEntry  = superstructureTab.Add("Blue", false).GetEntry();
    _redEntry   = superstructureTab.Add("Red", false).GetEntry();
    _resetEntry = superstructureTab.Add("Reset values", false)
                      .WithWidget(frc::BuiltInWidgets::kToggleButton)
                      .GetEntry();
#endif
}
void Indexer::UpdateShuffleboard()
{
    UpdateColorSensors();

    _curColorRatioEntry.SetDouble(GetRedBlueRatio());
    _topLightSensorEntry.SetBoolean(GetTopLightSensor());
    _bottomLightSensorEntry.SetBoolean(GetBottomLightSensor());
}
