#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

class DriveTrain;
class Indexer;
class Intake;
class Shooter;
class Turret;
class SensorBoard;
class DefaultIndexCommand;
class DefaultTurretNewCommand;

class TwoBaller : public frc2::CommandHelper<frc2::SequentialCommandGroup, TwoBaller> {
   public:
    TwoBaller(DriveTrain& drivetrain,
              Intake& intake,
              Shooter& shooter,
              SensorBoard& sensorControl,
              Turret& turret,
              Indexer& indexer,
              frc2::InstantCommand& stopDrivetrainCommand,
              DefaultIndexCommand& defaultIndexCommand);
};

class TwoBallDefense : public frc2::CommandHelper<frc2::SequentialCommandGroup, TwoBallDefense> {
   public:
    TwoBallDefense(DriveTrain& drivetrain,
                   Intake& intake,
                   Shooter& shooter,
                   SensorBoard& sensorControl,
                   Turret& turret,
                   Indexer& indexer,
                   frc2::InstantCommand& stopDrivetrainCommand,
                   DefaultIndexCommand& defaultIndexCommand);
};

class ThreeBallHP : public frc2::CommandHelper<frc2::SequentialCommandGroup, ThreeBallHP> {
   public:
    ThreeBallHP(DriveTrain& drivetrain,
                Intake& intake,
                Shooter& shooter,
                SensorBoard& sensorControl,
                Turret& turret,
                Indexer& indexer,
                frc2::InstantCommand& stopDrivetrainCommand,
                DefaultIndexCommand& defaultIndexCommand,
                DefaultTurretNewCommand& defaultTurretNewCommand);
};

class ThreeBallKMM : public frc2::CommandHelper<frc2::SequentialCommandGroup, ThreeBallKMM> {
   public:
    ThreeBallKMM(DriveTrain& drivetrain,
                 Intake& intake,
                 Shooter& shooter,
                 SensorBoard& sensorControl,
                 Turret& turret,
                 Indexer& indexer,
                 frc2::InstantCommand& stopDrivetrainCommand);
};

class ThreeBallTriangle
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, ThreeBallTriangle> {
   public:
    ThreeBallTriangle(DriveTrain& drivetrain,
                      Intake& intake,
                      Shooter& shooter,
                      SensorBoard& sensorControl,
                      Turret& turret,
                      Indexer& indexer,
                      frc2::InstantCommand& stopDrivetrainCommand);
};

class FourBall : public frc2::CommandHelper<frc2::SequentialCommandGroup, FourBall> {
   public:
    FourBall(DriveTrain& drivetrain,
             Intake& intake,
             Shooter& shooter,
             SensorBoard& sensorControl,
             Turret& turret,
             Indexer& indexer,
             frc2::InstantCommand& stopDrivetrainCommand,
             DefaultIndexCommand& defaultIndexCommand,
             DefaultTurretNewCommand& defaultTurretNewCommand);
};

class FiveBallTrans : public frc2::CommandHelper<frc2::SequentialCommandGroup, FiveBallTrans> {
   public:
    FiveBallTrans(DriveTrain& drivetrain,
                  Intake& intake,
                  Shooter& shooter,
                  SensorBoard& sensorControl,
                  Turret& turret,
                  Indexer& indexer,
                  frc2::InstantCommand& stopDrivetrainCommand,
                  DefaultIndexCommand& defaultIndexCommand,
                  DefaultTurretNewCommand& defaultTurretNewCommand);
};
