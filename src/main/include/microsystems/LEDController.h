#pragma once

#include <frc/AddressableLED.h>
#include <array>

#include "utils/PORTS.h"

class LEDController {
   public:
    LEDController();

    void UpdateLED(int desiredColor, bool autoClimbing, bool LEDoverride, bool indexLEDdesired);
    void UpdateDisabledLED(int desiredColor);

    void RainbowLED();
    void AlignedLED();
    void PreppingLED();
    void ShootingLED();
    void AutoClimbLED();
    void AutoManualClimbLED();
    void IntakingLED();
    void ClimbDoneLED();
    void ElevatorUndoLED();
    void FullSystemLED();
    void HalfSystemLED();
    void AlignedShootingLED();

   private:
    int _firstPixelHue  = 0;
    int _alignedCounter = 0, _LEDcounter = 0, _LEDsecondaryCounter = 0;
    frc::AddressableLED _LEDstrip{LED_PORT};
    std::array<frc::AddressableLED::LEDData, LED_LENGTH> _ledBuffer;
};