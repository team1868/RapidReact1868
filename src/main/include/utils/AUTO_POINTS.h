
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpi/numbers>

// two ball
frc::Pose2d const TWO_BALL_POS_1{3.62_m, 0.0_m, 0.0_deg};

frc::Pose2d const TWO_BALL_DEF_POS_1{3.62_m, 0.0_m, 0.0_deg};
// 6 ft away from starting pos, -35.384 degrees, end w/ -73.021 degrees angle
frc::Pose2d const TWO_BALL_DEF_POS_2{6.0 * 0.8153_m, 6.0 * -0.579_m, -73.021_deg};
// intermediate point
frc::Translation2d const TWO_BALL_DEF_POS_2_WAYPOINT{6.0 * 0.8153_m + 3.0_m,
                                                     6.0 * -0.579_m + 1.74_m};
// end looking at hangar
frc::Pose2d const TWO_BALL_DEF_POS_3{6.0_m, 0.0_m, 43.0_deg};

// four ball
frc::Pose2d const FOUR_BALL_POS_1{2.80_m, 0.0_m, 0.0_deg};
frc::Pose2d const FOUR_BALL_POS_2{8.7 * 0.9583_m, 8.7 * -0.2857_m, 0.0_deg};
frc::Pose2d const FOUR_BALL_POS_3{5.0 * 0.9583_m, 5.0 * -0.2857_m, 0.0_deg};

frc::Pose2d const FIVE_BALL_POS_1{0.3_m + 13.07_m * cos(73.88 * wpi::numbers::pi / 180.0),
                                  13.07_m * sin(73.88 * wpi::numbers::pi / 180.0),
                                  73.88_deg};
