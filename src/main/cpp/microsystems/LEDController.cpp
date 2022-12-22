#include "microsystems/LEDController.h"

LEDController::LEDController()
{
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV(198 / 2, 41 * 255 / 100, int(97 * 2.55));
    }
    _LEDstrip.SetLength(LED_LENGTH);
    _LEDstrip.SetData(_ledBuffer);
    _LEDstrip.Start();
}

void LEDController::UpdateDisabledLED(int desiredColor)
{
    switch (desiredColor) {
        case (1): PreppingLED(); break;
        case (2): ShootingLED(); break;
        case (3): IntakingLED(); break;
        case (4): ClimbDoneLED(); break;
        case (5): AutoManualClimbLED(); break;
        default: RainbowLED();
    }
}

void LEDController::UpdateLED(int desiredColor,
                              bool autoClimbing,
                              bool LEDoverride,
                              bool indexLEDdesired)
{
    if (autoClimbing) {
        switch (desiredColor) {
            case (0): ClimbDoneLED(); break;
            case (1): AutoManualClimbLED(); break;
            case (2): IntakingLED(); break;
            case (3): PreppingLED(); break;
            case (4): ShootingLED(); break;
            case (5): HalfSystemLED(); break;
            case (6): FullSystemLED(); break;
            default: RainbowLED(); break;
        }
        return;
    }

    if (desiredColor == 9 && !LEDoverride) {
        _LEDcounter = 0;
        AlignedLED();
    } else if (desiredColor == 5) {  // zero turret
        _LEDcounter = 0;
        AutoManualClimbLED();
    } else if (indexLEDdesired == 1) {  // full
        FullSystemLED();
    } else if (indexLEDdesired == 0) {  // one ball
        HalfSystemLED();
    } else {
        _LEDcounter = 0;
        switch (desiredColor) {
            case (1): PreppingLED(); break;
            case (2): ShootingLED(); break;
            case (3): IntakingLED(); break;
            case (7): ClimbDoneLED(); break;
            default: RainbowLED();
        }
    }

    _LEDstrip.SetData(_ledBuffer);
}

void LEDController::AlignedLED()
{  // lime
    if (_alignedCounter < 10) {
        for (int i = 0; i < LED_LENGTH; i++) {
            _ledBuffer[i].SetHSV(116 / 2, 255, 70);
        }
    } else if (_alignedCounter < 20) {
        for (int i = 0; i < LED_LENGTH; i++) {
            _ledBuffer[i].SetHSV(116 / 2, 255, 0);
        }
    } else {
        _alignedCounter = -1;
    }
    _alignedCounter++;
}

void LEDController::AlignedShootingLED()
{  // lime
    if (_alignedCounter < 10) {
        for (int i = 0; i < LED_LENGTH; i++) {
            _ledBuffer[i].SetHSV(18, 255, 70);
        }
    } else if (_alignedCounter < 20) {
        for (int i = 0; i < LED_LENGTH; i++) {
            _ledBuffer[i].SetHSV(18, 255, 0);
        }
    } else {
        _alignedCounter = -1;
    }
    _alignedCounter++;
}

void LEDController::RainbowLED()
{
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV((_firstPixelHue + (i * 180 / LED_LENGTH)) % 180, 200, 70);
    }
    _firstPixelHue = (_firstPixelHue + 3) % 180;
}

void LEDController::FullSystemLED()
{
    if (++_LEDsecondaryCounter > 5) {
        _LEDcounter++;
        _LEDsecondaryCounter = 0;
    }
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV((i + _LEDcounter) % 6 < 3 ? 0 : 180, 214, 70);
    }
}

void LEDController::HalfSystemLED()
{
    if (++_LEDsecondaryCounter > 5) {
        _LEDcounter++;
        _LEDsecondaryCounter = 0;
    }
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV(((i + _LEDcounter) % 6 < 3) ? 56 : 272, 214, 70);
    }
}

void LEDController::PreppingLED()
{  // red
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV((_firstPixelHue + (i * 180 / LED_LENGTH)) % 8, 200, 70);
    }

    _firstPixelHue = (_firstPixelHue + 1) % 8;
}

void LEDController::ShootingLED()
{  // orange-yellow
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV((_firstPixelHue + (i * 180 / LED_LENGTH)) % 10 + 182, 200, 70);
    }
    _firstPixelHue = (_firstPixelHue + 1) % 10;
}

void LEDController::AutoClimbLED()
{  // light blue
    for (int i = 0; i < LED_LENGTH; i++) {
        auto const pixelHue = (_firstPixelHue + (i * 180 / LED_LENGTH)) % 6 + 115;
        _ledBuffer[i].SetHSV(pixelHue, 200, 70);
    }
    _firstPixelHue = (_firstPixelHue + 1) % 6;
}

void LEDController::AutoManualClimbLED()
{  // blueish purple
    for (int i = 0; i < LED_LENGTH; i++) {
        auto const pixelHue = (_firstPixelHue + (i * 180 / LED_LENGTH)) % 10 + 115;
        _ledBuffer[i].SetHSV(pixelHue, 200, 70);
    }
    _firstPixelHue = (_firstPixelHue + 1) % 10;
}

void LEDController::IntakingLED()
{  // purple
    for (int i = 0; i < LED_LENGTH; i++) {
        auto const pixelHue = (_firstPixelHue + (i * 180 / LED_LENGTH)) % 12 + 135;
        _ledBuffer[i].SetHSV(pixelHue, 200, 70);
    }
    _firstPixelHue += 1;
    _firstPixelHue %= 12;
}

void LEDController::ClimbDoneLED()
{  // sc blue
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV(198 / 2, int(50 * 255 / 100), 70);
    }
}

void LEDController::ElevatorUndoLED()
{
    for (int i = 0; i < LED_LENGTH; i++) {
        _ledBuffer[i].SetHSV(0, 0, 70);
    }
}
