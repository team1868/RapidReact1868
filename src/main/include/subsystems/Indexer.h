#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/DigitalInput.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>
#include <rev/ColorSensorV3.h>

#include "utils/INDEXER_CONSTANTS.h"
#include "utils/PORTS.h"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

class Indexer : public frc2::SubsystemBase {
   public:
    Indexer();
    void Periodic() override;

    enum BALL_COLOR { UNKNOWN, BLUE, RED };

    bool IndexIsBlocked();
    bool ColorSensorExists();
    double GetColorRatio();
    double GetRedBlueRatio();
    bool GetTopLightSensor();
    bool GetBottomLightSensor();

    void SetIndexRollersPower(double power);
    double GetIndexRollersPower();
    void SetElevatorPower(double power);
    double GetElevatorPower();

   private:
    void ConfigureShuffleboard();
    void UpdateShuffleboard();
    void UpdateColorSensors();
    BALL_COLOR GetMatchColor();

#ifdef JO
    WPI_VictorSPX _indexRollerMotor{ELEVATOR_FEEDER_MOTOR_ID};
    WPI_VictorSPX _elevatorMotor{ELEVATOR_MOTOR_ID};

    frc::DigitalInput _colorSensor;

    // optical (decide between IR break beam or photoelectric)
    frc::DigitalInput _bottomLightSensor{BOTTOM_ELEVATOR_LIGHT_SENSOR_PORT};
    frc::DigitalInput _topLightSensor{TOP_ELEVATOR_LIGHT_SENSOR_PORT};
#elif defined(PRACTICEBOT) || defined(COMPBOT)
    WPI_VictorSPX _indexRollerMotor{ELEVATOR_MOTOR_ID};
    WPI_VictorSPX _elevatorMotor{INDEX_ROLLER_MOTOR_ID};

    rev::ColorSensorV3 _colorSensor{I2CPORT};
    nt::NetworkTableEntry _blueEntry, _redEntry, _resetEntry;
    nt::NetworkTableEntry _actualREntry, _actualGEntry, _actualBEntry;
    nt::NetworkTableEntry _proximityBoolEntry, _proximityEntry;

    frc::DigitalInput _bottomLightSensor{BOTTOM_LIGHT_SENSOR_PORT};
    frc::DigitalInput _topLightSensor{TOP_LIGHT_SENSOR_PORT};
#endif

    frc::Color _detectedColor;
    bool _colorSensorExists = false;
    double _blueTolerance, _redTolerance;
    int _minProximity, _curProximity, _maxProximity;
    nt::NetworkTableEntry _colorRatioEntry, _curColorRatioEntry, _colorSensorDeadEntry;
    nt::NetworkTableEntry _topLightSensorEntry, _bottomLightSensorEntry;
};
