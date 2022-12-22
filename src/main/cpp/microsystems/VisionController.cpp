#include "microsystems/VisionController.h"

#include <units/angle.h>
#include <units/length.h>

VisionController::VisionController()
{
    _photonResult = _photonVision.GetLatestResult();
    _hasTarget    = _photonResult.HasTargets();
}

// frc::Pose2d VisionController::GetVisionPose()
// {
//     return photonlib::PhotonUtils::EstimateFieldToRobot(CAMERA_HEIGHT_M,
//                                                         TARGET_HEIGHT_M,
//                                                         CAMERA_PITCH_M,
//                                                         _photonPitch,
//                                                         -_photonYaw,
//                                                         _navX.GetRotation2d(),
//                                                         _targetPose,
//                                                         _cameraToRobot);
// }

void VisionController::Update()
{
    _photonResult = _photonVision.GetLatestResult();
    _hasTarget    = _photonResult.HasTargets();

    if (_hasTarget) {
        _target         = _photonResult.GetBestTarget();
        _photonPose     = _target.GetCameraRelativePose();
        _photonPitch    = degree_t{_target.GetPitch()};
        _photonYaw      = degree_t{_target.GetYaw()};
        _photonDistance = photonlib::PhotonUtils::CalculateDistanceToTarget(
            CAMERA_HEIGHT_M, TARGET_HEIGHT_M, CAMERA_PITCH_M, _photonPitch);
        // _photonCalcPose = GetVisionPose();
    }
}

units::degree_t VisionController::GetPhotonAngle() { return _photonYaw; }

units::foot_t VisionController::GetPhotonDistance()
{
    // strict typing converts _photonDistance meters -> feet
    return _photonDistance;
}
