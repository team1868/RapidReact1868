#include "microsystems/Logger.h"

#include <ctime>

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/DriveTrain.h"
#include "utils/PORTS.h"

std::ofstream Logger::logData;
std::ofstream Logger::logAction;
#define USE_NAVX = true;

/**
 * Log state: records the physical state of the robot and human control
 */
void Logger::LogState(DriveTrain& drivetrain,
                      SensorBoard& sensorControl,
                      ControlBoard& humanControl)
{
    if (!logData.is_open()) {
        logData.open(GetTimeStamp().append("datalog.txt"), std::ofstream::out | std::ofstream::app);

        stringBuffer << "Time, "
                     // "Left Encoder, Right Encoder, Left Wheel Speed,"
                     // << "Right Wheel Speed, Yaw, Roll, Pitch, Voltage, Total Current, "
                     // << "Left Drive A Current, Left Drive B Current, Right Drive A Current, Right
                     // Drive B Current, "
                     // << "Flywheel One Current, Flywheel Two Current, Climber One Current, Climber
                     // Two Current, "
                     // << "Intake Rollers Current, Intake Wrist Current, "
                     // << "Funnel Index Current, Elevator Bottom Current, Elevator Top Current, "
                     << "Voltage, "
                     << "Total Current, "
                     << "Pose X, "
                     << "Pose Y"
                     // << "RoboRIO Current, Total Power, Total Energy, Pressure, "
                     // << "Left Joy X, Left Joy Y, "
                     // << "Right Joy X, Right Joy Y, Reverse (currently not in), Arcade, "
                     // << "Quick Turn Desired"
                     << "\r\n";
        // fix labels
    }

    stringBuffer << sensorControl.GetTime() << ", " <<
        // drivetrain.GetLeftEncoderValue() << ", " <<
        // drivetrain.GetRightEncoderValue() << ", " <<
        // sensorControl.GetNavXYaw() << ", " <<
        // sensorControl.GetNavXRoll() << ", " <<
        // sensorControl.GetNavXPitch() << ", " <<
        drivetrain.GetVoltage() << ", " <<

        // drivetrain.GetCompressorCurrent() << ", " <<
        // drivetrain.GetRIOCurrent() << ", " <<
        drivetrain.GetTotalCurrent() << ", " << ((double)drivetrain.GetPose().X()) << ", "
                 << ((double)drivetrain.GetPose().Y());
    // drivetrain.GetTotalEnergy() << ", " <<
    // drivetrain.GetPressureSwitchValue() << ", " <<
    // humanControl.GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kX) << ", " <<
    // humanControl.GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY) << ", " <<
    // humanControl.GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX) << ", " <<
    // humanControl.GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kY);

    logAction << stringBuffer.rdbuf();
    logAction.flush();
    stringBuffer.clear();
    stringBuffer.str("");
}
/* format:
 * robotmodel state / ControlBoard state
 *
 * ie:
 *
 * timer / left motor / right motor / gear shift / pdp voltage / leftjoy x / leftjoy y
 * 			/ rightjoy x / rightjoy y / reverse desired / gearshift desired /
 */

/**
 * Log action: records higher-level processes
 * @param fileName a reference to an std::string
 * @param line an integer
 * @param stateName a reference to an std::string
 * @param state a reference to an std::string
 */
void Logger::LogAction(DriveTrain& drivetrain,
                       SensorBoard& sensorControl,
                       const std::string& fileName,
                       int line,
                       const std::string& stateName,
                       const std::string& state)
{
    if (!logAction.is_open()) {
        logAction.open(GetTimeStamp().append("actionlog.txt"),
                       std::ofstream::out | std::ofstream::app);
    }

    stringBuffer << sensorControl.GetTime() << ", " << fileName << ", " << line << ", " << stateName
                 << ", " << state << "\r\n";
    logAction << stringBuffer.rdbuf();
    logAction.flush();
    stringBuffer.clear();
    stringBuffer.str("");
}

/* overloaded methods without time stamp */
void Logger::LogAction(const std::string& fileName,
                       int line,
                       const std::string& stateName,
                       bool state)
{
    if (!logAction.is_open()) {
        logAction.open(GetTimeStamp().append("actionlog.txt"),
                       std::ofstream::out | std::ofstream::app);
    }

    stringBuffer << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
    logAction << stringBuffer.rdbuf();
    logAction.flush();
    stringBuffer.clear();
    stringBuffer.str("");
}

void Logger::LogAction(const std::string& fileName,
                       int line,
                       const std::string& stateName,
                       double state)
{
    if (!logAction.is_open()) {
        logAction.open(GetTimeStamp().append("actionlog.txt"),
                       std::ofstream::out | std::ofstream::app);
    }

    stringBuffer << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
    logAction << stringBuffer.rdbuf();
    logAction.flush();
    stringBuffer.clear();
    stringBuffer.str("");
}

void Logger::LogAction(const std::string& fileName,
                       int line,
                       const std::string& stateName,
                       const std::string& state)
{
    if (!logAction.is_open()) {
        logAction.open(GetTimeStamp().append("actionlog.txt"),
                       std::ofstream::out | std::ofstream::app);
    }

    // steps for debugging
    // single line direct to logAction output file, see if the overrun can be recreated
    // logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
    stringBuffer << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
    // we should only flush ocassionally and the string buffer does not need to be written each time
    // In general only flushing/writing when specific actions occur rather than every iteration is
    // best Eliminating flush actions is far more important
    logAction << stringBuffer.rdbuf();
    logAction.flush();
    stringBuffer.clear();
    stringBuffer.str("");
}

/**
 * Closes the log
 */
void Logger::CloseLogs()
{
    logData.close();
    logAction.close();
}

/**
 * Gets time stamp
 * @return the time as a string
 */
std::string Logger::GetTimeStamp()
{
    /*	struct timespec tp;
    clock_gettime(CLOCK_REALTIME,&tp);
    double realTime = (double)tp.tv_sec + (double)((double)tp.tv_nsec*1e-9);
*/
    time_t rawtime = time(0);
    struct tm* timeinfo;  // get current time
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);  // converts time_t to tm as local time
    strftime(
        buffer, 80, "/home/lvuser/%F_%H_%M_actionlog.txt", timeinfo);  // fileName contains %F_%H_%M

    return buffer;
}