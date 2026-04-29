#include "activeGuidanceMath.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;                        // [rad]
constexpr double kRad2Deg = 180.0 / kPi;                              // [deg/rad]
constexpr double kTinyNorm = 1.0e-12;                                 // [-]
constexpr double kRollSweepStepDeg = 2.0;                             // [deg]
constexpr double kRollNearOptMarginDeg = 0.25;                        // [deg]
constexpr double kRollPhasePerStepRad = kRollSweepStepDeg * kPi / 180.0; // [rad]
constexpr int kRollSweepCount = 180;                                  // [-]

inline double clampUnitRange(double value)
{
    // Guard for floating-point drift before acos().
    return std::max(-1.0, std::min(1.0, value));
}

inline double wrapDeltaDeg(double aDeg, double bDeg)
{
    // Smallest circular separation between two roll angles in degrees.
    double delta = std::fmod(aDeg - bDeg + 180.0, 360.0);
    if (delta < 0.0) {
        delta += 360.0;
    }
    return std::fabs(delta - 180.0);
}

inline void copy3(const double in[3], double out[3])
{
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

inline double dot3(const double vecA[3], const double vecB[3])
{
    return vecA[0] * vecB[0] + vecA[1] * vecB[1] + vecA[2] * vecB[2];
}

inline double norm3(const double vec[3])
{
    return std::sqrt(dot3(vec, vec));
}

inline void cross3(const double vecA[3], const double vecB[3], double out[3])
{
    out[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
    out[1] = vecA[2] * vecB[0] - vecA[0] * vecB[2];
    out[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

void buildYzFrameAboutX(const double x_B[3],
                        const double earthHat[3],
                        double outY_B[3],
                        double outZ_B[3])
{
    cross3(x_B, earthHat, outZ_B);
    if (!ActiveGuidanceMath::safeUnit(outZ_B, outZ_B)) {
        double fallbackY[3] = {0.0, 1.0, 0.0};
        cross3(x_B, fallbackY, outZ_B);
        if (!ActiveGuidanceMath::safeUnit(outZ_B, outZ_B)) {
            double fallbackZ[3] = {0.0, 0.0, 1.0};
            cross3(x_B, fallbackZ, outZ_B);
            ActiveGuidanceMath::safeUnit(outZ_B, outZ_B);
        }
    }

    cross3(outZ_B, x_B, outY_B);
    ActiveGuidanceMath::safeUnit(outY_B, outY_B);
}

}  // namespace

namespace ActiveGuidanceMath {

double earthLimbHalfAngleRad(double earthRadiusKm, double orbitAltitudeKm)
{
    return std::asin(earthRadiusKm / (earthRadiusKm + orbitAltitudeKm));
}

bool safeUnit(const double vec[3], double out[3])
{
    const double mag = norm3(vec);
    if (mag < kTinyNorm) {
        return false;
    }

    out[0] = vec[0] / mag;
    out[1] = vec[1] / mag;
    out[2] = vec[2] / mag;
    return true;
}

double angleDegBetween(const double vecA[3], const double vecB[3])
{
    const double dot = clampUnitRange(dot3(vecA, vecB));
    return std::acos(dot) * kRad2Deg;
}

void computeCompromiseX(const double earthHat[3],
                        const double sunHat[3],
                        double rhoRad,
                        double outX_B[3])
{
    double antiSun[3] = {-sunHat[0], -sunHat[1], -sunHat[2]};
    const double antiSunProjection = dot3(antiSun, earthHat);

    double u2[3] = {
        antiSun[0] - antiSunProjection * earthHat[0],
        antiSun[1] - antiSunProjection * earthHat[1],
        antiSun[2] - antiSunProjection * earthHat[2]
    };

    if (!safeUnit(u2, u2)) {
        double fallback[3] = {1.0, 0.0, 0.0};
        if (std::fabs(earthHat[0]) >= 0.9) {
            fallback[0] = 0.0;
            fallback[1] = 1.0;
        }

        const double fallbackProjection = dot3(fallback, earthHat);
        u2[0] = fallback[0] - fallbackProjection * earthHat[0];
        u2[1] = fallback[1] - fallbackProjection * earthHat[1];
        u2[2] = fallback[2] - fallbackProjection * earthHat[2];
        safeUnit(u2, u2);
    }

    outX_B[0] = std::cos(rhoRad) * earthHat[0] + std::sin(rhoRad) * u2[0];
    outX_B[1] = std::cos(rhoRad) * earthHat[1] + std::sin(rhoRad) * u2[1];
    outX_B[2] = std::cos(rhoRad) * earthHat[2] + std::sin(rhoRad) * u2[2];
    safeUnit(outX_B, outX_B);
}

RollSolveResult solveRollForLostClearance(const double x_B[3],
                                          const double earthHat[3],
                                          const double sunHat[3],
                                          const std::vector<const double*>& extraKeepoutHats,
                                          bool hasPrevRoll,
                                          double prevRollDeg)
{
    struct Candidate {
        double rollDeg;
        double scoreDeg;
        double y_B[3];
        double z_B[3];
    };

    double yTemp[3] = {0.0, 0.0, 0.0};
    double zTemp[3] = {0.0, 0.0, 0.0};
    buildYzFrameAboutX(x_B, earthHat, yTemp, zTemp);

    Candidate candidates[kRollSweepCount];
    double maxScoreDeg = -std::numeric_limits<double>::infinity();

    for (int idx = 0; idx < kRollSweepCount; ++idx) {
        const double rollDeg = idx * kRollSweepStepDeg;              // [deg]
        const double rollRad = idx * kRollPhasePerStepRad;           // [rad]

        double yTest[3] = {
            std::cos(rollRad) * yTemp[0] + std::sin(rollRad) * zTemp[0],
            std::cos(rollRad) * yTemp[1] + std::sin(rollRad) * zTemp[1],
            std::cos(rollRad) * yTemp[2] + std::sin(rollRad) * zTemp[2]
        };

        double zTest[3] = {0.0, 0.0, 0.0};
        cross3(x_B, yTest, zTest);
        safeUnit(zTest, zTest);

        double scoreDeg = std::min(
            angleDegBetween(zTest, earthHat),
            angleDegBetween(zTest, sunHat)
        );

        for (const double* keepoutHat : extraKeepoutHats) {
            scoreDeg = std::min(scoreDeg, angleDegBetween(zTest, keepoutHat));
        }

        candidates[idx].rollDeg = rollDeg;
        candidates[idx].scoreDeg = scoreDeg;
        copy3(yTest, candidates[idx].y_B);
        copy3(zTest, candidates[idx].z_B);

        if (scoreDeg > maxScoreDeg) {
            maxScoreDeg = scoreDeg;
        }
    }

    int bestIdx = -1;
    double bestRank = std::numeric_limits<double>::infinity();

    for (int idx = 0; idx < kRollSweepCount; ++idx) {
        if (candidates[idx].scoreDeg < maxScoreDeg - kRollNearOptMarginDeg) {
            continue;
        }

        if (!hasPrevRoll) {
            const double rank = -candidates[idx].scoreDeg;
            if (rank < bestRank) {
                bestRank = rank;
                bestIdx = idx;
            }
        } else {
            const double rank = wrapDeltaDeg(candidates[idx].rollDeg, prevRollDeg);
            if (rank < bestRank) {
                bestRank = rank;
                bestIdx = idx;
            }
        }
    }

    if (bestIdx < 0) {
        bestIdx = 0;
    }

    RollSolveResult result = {};
    result.rollDeg = candidates[bestIdx].rollDeg;
    result.scoreDeg = maxScoreDeg;
    copy3(candidates[bestIdx].y_B, result.y_B);
    copy3(candidates[bestIdx].z_B, result.z_B);
    return result;
}

bool buildFrameForMinusZTarget(const double minusZTargetHat[3],
                               const double xHintHat[3],
                               double outX_B[3],
                               double outY_B[3],
                               double outZ_B[3])
{
    double minusZHat[3] = {0.0, 0.0, 0.0};
    if (!safeUnit(minusZTargetHat, minusZHat)) {
        return false;
    }

    // Antenna boresight is -Z, so body +Z points opposite the target direction.
    outZ_B[0] = -minusZHat[0];
    outZ_B[1] = -minusZHat[1];
    outZ_B[2] = -minusZHat[2];

    const double xHintProjection = dot3(xHintHat, outZ_B);
    outX_B[0] = xHintHat[0] - xHintProjection * outZ_B[0];
    outX_B[1] = xHintHat[1] - xHintProjection * outZ_B[1];
    outX_B[2] = xHintHat[2] - xHintProjection * outZ_B[2];

    if (!safeUnit(outX_B, outX_B)) {
        double fallback[3] = {1.0, 0.0, 0.0};
        if (std::fabs(outZ_B[0]) >= 0.9) {
            fallback[0] = 0.0;
            fallback[1] = 1.0;
        }

        const double fallbackProjection = dot3(fallback, outZ_B);
        outX_B[0] = fallback[0] - fallbackProjection * outZ_B[0];
        outX_B[1] = fallback[1] - fallbackProjection * outZ_B[1];
        outX_B[2] = fallback[2] - fallbackProjection * outZ_B[2];
        if (!safeUnit(outX_B, outX_B)) {
            return false;
        }
    }

    cross3(outZ_B, outX_B, outY_B);
    if (!safeUnit(outY_B, outY_B)) {
        return false;
    }

    return true;
}

bool buildFrameForPlusXTarget(const double plusXTargetHat[3],
                               const double zHintHat[3],
                               double outX_B[3],
                               double outY_B[3],
                               double outZ_B[3])
{
    double plusXHat[3] = {0.0, 0.0, 0.0};
    if (!safeUnit(plusXTargetHat, plusXHat)) {
        return false;
    }

    // Keep +X aligned with the station LOS so DOWNLINK preserves FOUND Earth-pointing behavior.
    outX_B[0] = plusXHat[0];
    outX_B[1] = plusXHat[1];
    outX_B[2] = plusXHat[2];

    const double zHintProjection = dot3(zHintHat, outX_B);
    outZ_B[0] = zHintHat[0] - zHintProjection * outX_B[0];
    outZ_B[1] = zHintHat[1] - zHintProjection * outX_B[1];
    outZ_B[2] = zHintHat[2] - zHintProjection * outX_B[2];

    if (!safeUnit(outZ_B, outZ_B)) {
        double fallback[3] = {0.0, 0.0, 1.0};
        if (std::fabs(outX_B[2]) >= 0.9) {
            fallback[1] = 1.0;
            fallback[2] = 0.0;
        }

        const double fallbackProjection = dot3(fallback, outX_B);
        outZ_B[0] = fallback[0] - fallbackProjection * outX_B[0];
        outZ_B[1] = fallback[1] - fallbackProjection * outX_B[1];
        outZ_B[2] = fallback[2] - fallbackProjection * outX_B[2];
        if (!safeUnit(outZ_B, outZ_B)) {
            return false;
        }
    }

    cross3(outZ_B, outX_B, outY_B);
    if (!safeUnit(outY_B, outY_B)) {
        return false;
    }

    return true;
}

}  // namespace ActiveGuidanceMath
