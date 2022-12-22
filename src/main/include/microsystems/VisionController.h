#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

#include "utils/VISION_CONSTANTS.h"

class VisionController {
   public:
    VisionController();

    void Update();
    units::foot_t GetPhotonDistance();
    units::degree_t GetPhotonAngle();
    // frc::Pose2d GetVisionPose();

   private:
    photonlib::PhotonCamera _photonVision{"gloworm"};
    photonlib::PhotonPipelineResult _photonResult;
    photonlib::PhotonTrackedTarget _target;
    frc::Transform2d _photonPose;
    units::meter_t _photonDistance = 0_m;
    units::degree_t _photonPitch   = 0.0_deg;
    units::degree_t _photonYaw     = 0.0_deg;

    // assume camera is centered on robot
    frc::Transform2d _cameraToRobot{{0.0_m, 0.0_m, {}}, {0.0_m, 0.0_m, {}}};
    frc::Pose2d _targetPose{16.4846_m, 8.1026_m, {}};

    frc::Pose2d _photonCalcPose;
    bool _hasTarget;
};