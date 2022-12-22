#pragma once

namespace AutoConstants {
constexpr auto kMaxSpeed            = 3_mps;
constexpr auto kMaxAcceleration     = 2_mps_sq;
constexpr auto kMaxFastAcceleration = 4_mps_sq;

constexpr auto kRamseteB    = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

constexpr double kAutoDriveDistanceInches  = 10;
constexpr double kAutoBackupDistanceInches = 10;
constexpr double kAutoDriveSpeed           = 0.2;
}  // namespace AutoConstants