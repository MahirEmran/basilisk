/*
 ISC License

 Copyright (c) 2026, huskysat-camera-sim contributors

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "huskyActiveGuidance.h"

#include "architecture/utilities/rigidBodyKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;                     // [rad]
constexpr double kDeg2Rad = kPi / 180.0;                            // [rad/deg]
constexpr double kRad2Deg = 180.0 / kPi;                            // [deg/rad]
constexpr double kEarthRadiusKm = 6371.0;                           // [km]
constexpr double kOrbitAltitudeKm = 400.0;                          // [km]
constexpr double kTinyNorm = 1.0e-12;                               // [-]
constexpr double kValidStatePosFloorM = 1.0;                        // [m]
constexpr double kRollSweepStepDeg = 2.0;                           // [deg]
constexpr double kRollNearOptMarginDeg = 0.25;                      // [deg]
constexpr double kRollPhasePerStepRad = kRollSweepStepDeg * kDeg2Rad;  // [rad]
constexpr int kRollSweepCount = 180;                                // [-]

inline double clampUnitRange(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

inline double wrapDeltaDeg(double aDeg, double bDeg)
{
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

}  // namespace

HuskyActiveGuidance::HuskyActiveGuidance()
{
    this->rhoRad = std::asin(kEarthRadiusKm / (kEarthRadiusKm + kOrbitAltitudeKm));
    this->earthHalfAngleDeg = this->rhoRad * kRad2Deg;
    this->refreshThresholds();
}

HuskyActiveGuidance::~HuskyActiveGuidance()
{
    return;
}

void HuskyActiveGuidance::Reset(uint64_t CurrentSimNanos)
{
    this->hasPrevRoll = false;
    this->state = "UNINITIALIZED";
    this->lastStatusPrintNanos = CurrentSimNanos;
    this->hasPrintedStatus = false;

    this->writeIdentityReference(CurrentSimNanos);

    if (!this->scStateInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "HuskyActiveGuidance requires scStateInMsg to be connected.");
    }
}

void HuskyActiveGuidance::UpdateState(uint64_t CurrentSimNanos)
{
    if (!this->scStateInMsg.isLinked() || !this->scStateInMsg.isWritten()) {
        this->writeIdentityReference(CurrentSimNanos);
        return;
    }

    const SCStatesMsgPayload scState = this->scStateInMsg();
    double r_N[3] = {scState.r_BN_N[0], scState.r_BN_N[1], scState.r_BN_N[2]};
    const double rMag = this->norm3(r_N);

    if (rMag < kValidStatePosFloorM) {
        this->writeIdentityReference(CurrentSimNanos);
        return;
    }

    double earthHat_sc[3] = {
        -r_N[0] / rMag,
        -r_N[1] / rMag,
        -r_N[2] / rMag
    };

    double sunHat_sc[3] = {
        this->defaultSunHat_N[0],
        this->defaultSunHat_N[1],
        this->defaultSunHat_N[2]
    };

    double sunAbs_N[3] = {0.0, 0.0, 0.0};
    bool hasSunAbs = false;

    if (this->sunStateInMsg.isLinked() && this->sunStateInMsg.isWritten()) {
        const SpicePlanetStateMsgPayload sunState = this->sunStateInMsg();
        sunAbs_N[0] = sunState.PositionVector[0];
        sunAbs_N[1] = sunState.PositionVector[1];
        sunAbs_N[2] = sunState.PositionVector[2];

        double sunRel_N[3] = {
            sunAbs_N[0] - r_N[0],
            sunAbs_N[1] - r_N[1],
            sunAbs_N[2] - r_N[2]
        };
        if (this->safeUnit(sunRel_N, sunHat_sc)) {
            hasSunAbs = true;
        }
    }

    double moonHat_sc[3] = {0.0, 0.0, 0.0};
    bool hasMoonHat = false;
    if (this->moonStateInMsg.isLinked() && this->moonStateInMsg.isWritten()) {
        const SpicePlanetStateMsgPayload moonState = this->moonStateInMsg();
        double moonRel_N[3] = {
            moonState.PositionVector[0] - r_N[0],
            moonState.PositionVector[1] - r_N[1],
            moonState.PositionVector[2] - r_N[2]
        };
        hasMoonHat = this->safeUnit(moonRel_N, moonHat_sc);
    }

    double c_BN[3][3];
    double sigma_BN[3] = {scState.sigma_BN[0], scState.sigma_BN[1], scState.sigma_BN[2]};
    MRP2C(sigma_BN, c_BN);

    double posFoundOffset_N[3] = {
        c_BN[0][0] * this->posFound_B[0] + c_BN[1][0] * this->posFound_B[1] + c_BN[2][0] * this->posFound_B[2],
        c_BN[0][1] * this->posFound_B[0] + c_BN[1][1] * this->posFound_B[1] + c_BN[2][1] * this->posFound_B[2],
        c_BN[0][2] * this->posFound_B[0] + c_BN[1][2] * this->posFound_B[1] + c_BN[2][2] * this->posFound_B[2]
    };

    double foundPos_N[3] = {
        r_N[0] + posFoundOffset_N[0],
        r_N[1] + posFoundOffset_N[1],
        r_N[2] + posFoundOffset_N[2]
    };

    const double foundPosMag = this->norm3(foundPos_N);
    double earthHat_found[3] = {earthHat_sc[0], earthHat_sc[1], earthHat_sc[2]};
    if (foundPosMag >= kValidStatePosFloorM) {
        earthHat_found[0] = -foundPos_N[0] / foundPosMag;
        earthHat_found[1] = -foundPos_N[1] / foundPosMag;
        earthHat_found[2] = -foundPos_N[2] / foundPosMag;
    }

    double sunHat_found[3] = {sunHat_sc[0], sunHat_sc[1], sunHat_sc[2]};
    if (hasSunAbs) {
        double sunRelFound_N[3] = {
            sunAbs_N[0] - foundPos_N[0],
            sunAbs_N[1] - foundPos_N[1],
            sunAbs_N[2] - foundPos_N[2]
        };
        this->safeUnit(sunRelFound_N, sunHat_found);
    }

    double rollOnlyX_B[3] = {-sunHat_found[0], -sunHat_found[1], -sunHat_found[2]};
    std::vector<const double*> noExtras;
    const RollSolveResult rollOnlyResult = this->solveRollForLostClearance(
        rollOnlyX_B,
        earthHat_sc,
        sunHat_sc,
        noExtras
    );

    const double sunEarthAngleDeg = this->angleDegBetween(sunHat_found, earthHat_found);

    double x_B[3] = {rollOnlyX_B[0], rollOnlyX_B[1], rollOnlyX_B[2]};
    RollSolveResult selectedRoll = rollOnlyResult;
    std::string selectedState = "CHARGING";

    if (this->mode == MODE_EXPERIMENT) {
        this->computeCompromiseX(earthHat_found, sunHat_found, this->rhoRad, x_B);
        std::vector<const double*> experimentExtras;
        if (hasMoonHat) {
            experimentExtras.push_back(moonHat_sc);
        }
        selectedRoll = this->solveRollForLostClearance(x_B, earthHat_sc, sunHat_sc, experimentExtras);
        selectedState = "EXPERIMENT";
    } else if (this->mode == MODE_HYBRID) {
        bool chargingExitBand = (this->state == "CHARGING");
        const double clearThresholdDeg = chargingExitBand ? this->chargeExitClearDeg : this->chargeEnterClearDeg;
        const double sunVisThresholdDeg = chargingExitBand ? this->chargeExitSunVisDeg : this->chargeEnterSunVisDeg;

        const bool canCharge =
            (rollOnlyResult.scoreDeg >= clearThresholdDeg) &&
            (sunEarthAngleDeg >= sunVisThresholdDeg);

        if (canCharge) {
            selectedState = "CHARGING";
            copy3(rollOnlyX_B, x_B);
            selectedRoll = rollOnlyResult;
        } else {
            selectedState = "EXPERIMENT";
            this->computeCompromiseX(earthHat_found, sunHat_found, this->rhoRad, x_B);
            std::vector<const double*> experimentExtras;
            if (hasMoonHat) {
                experimentExtras.push_back(moonHat_sc);
            }
            selectedRoll = this->solveRollForLostClearance(x_B, earthHat_sc, sunHat_sc, experimentExtras);
        }
    }

    if (selectedState != this->state) {
        const double tSec = static_cast<double>(CurrentSimNanos) * 1.0e-9;  // [s]
        const char* panelCmd = (selectedState == "CHARGING") ? "OPEN" : "STOW";
        this->bskLogger.bskLog(
            BSK_INFORMATION,
            "[ADCS] t=%8.1fs -> %s (roll-only LOST clearance=%5.1f deg, panel cmd=%s)",
            tSec,
            selectedState.c_str(),
            rollOnlyResult.scoreDeg,
            panelCmd
        );
    }

    if (this->mode == MODE_HYBRID) {
        const bool periodElapsed = (CurrentSimNanos - this->lastStatusPrintNanos) >= this->statusPeriodNanos;
        if (!this->hasPrintedStatus || periodElapsed) {
            const double tSec = static_cast<double>(CurrentSimNanos) * 1.0e-9;  // [s]
            const char* panelCmd = (selectedState == "CHARGING") ? "OPEN" : "STOW";
            this->bskLogger.bskLog(
                BSK_INFORMATION,
                "[HYBRID] t=%8.1fs active=%s panel=%s roll-only-clearance=%5.1f deg sun-earth-angle=%5.1f deg",
                tSec,
                selectedState.c_str(),
                panelCmd,
                rollOnlyResult.scoreDeg,
                sunEarthAngleDeg
            );
            this->lastStatusPrintNanos = CurrentSimNanos;
            this->hasPrintedStatus = true;
        }
    }

    this->state = selectedState;
    this->prevRollDeg = selectedRoll.rollDeg;
    this->hasPrevRoll = true;

    double dcm_RN[3][3] = {
        {x_B[0], x_B[1], x_B[2]},
        {selectedRoll.y_B[0], selectedRoll.y_B[1], selectedRoll.y_B[2]},
        {selectedRoll.z_B[0], selectedRoll.z_B[1], selectedRoll.z_B[2]}
    };

    double sigma_RN[3] = {0.0, 0.0, 0.0};
    C2MRP(dcm_RN, sigma_RN);

    bool sigmaValid = std::isfinite(sigma_RN[0]) && std::isfinite(sigma_RN[1]) && std::isfinite(sigma_RN[2]);
    if (!sigmaValid) {
        sigma_RN[0] = 0.0;
        sigma_RN[1] = 0.0;
        sigma_RN[2] = 0.0;
    }

    AttRefMsgPayload outMsg = this->attRefOutMsg.zeroMsgPayload;
    outMsg.sigma_RN[0] = sigma_RN[0];
    outMsg.sigma_RN[1] = sigma_RN[1];
    outMsg.sigma_RN[2] = sigma_RN[2];
    outMsg.omega_RN_N[0] = 0.0;
    outMsg.omega_RN_N[1] = 0.0;
    outMsg.omega_RN_N[2] = 0.0;
    outMsg.domega_RN_N[0] = 0.0;
    outMsg.domega_RN_N[1] = 0.0;
    outMsg.domega_RN_N[2] = 0.0;

    this->attRefOutMsg.write(&outMsg, this->moduleID, CurrentSimNanos);
}

void HuskyActiveGuidance::setModeString(const std::string& modeString)
{
    if (modeString == "ROLL_ONLY") {
        this->mode = MODE_ROLL_ONLY;
        this->modeString = "ROLL_ONLY";
        return;
    }

    if (modeString == "EXPERIMENT" || modeString == "COMPROMISE") {
        this->mode = MODE_EXPERIMENT;
        this->modeString = "EXPERIMENT";
        return;
    }

    if (modeString == "HYBRID") {
        this->mode = MODE_HYBRID;
        this->modeString = "HYBRID";
        return;
    }

    this->bskLogger.bskLog(BSK_ERROR, "HuskyActiveGuidance: unsupported mode string '%s'.", modeString.c_str());
}

std::string HuskyActiveGuidance::getModeString() const
{
    return this->modeString;
}

void HuskyActiveGuidance::setLostExclHalfDeg(double valueDeg)
{
    if (valueDeg <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "HuskyActiveGuidance: lost exclusion half-angle must be > 0 deg.");
        return;
    }

    this->lostExclHalfDeg = valueDeg;
    this->refreshThresholds();
}

double HuskyActiveGuidance::getLostExclHalfDeg() const
{
    return this->lostExclHalfDeg;
}

void HuskyActiveGuidance::setStatusPeriodSec(double valueSec)
{
    if (valueSec <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "HuskyActiveGuidance: status period must be > 0 s.");
        return;
    }

    this->statusPeriodNanos = static_cast<uint64_t>(valueSec * 1.0e9);  // [ns]
}

double HuskyActiveGuidance::getStatusPeriodSec() const
{
    return static_cast<double>(this->statusPeriodNanos) * 1.0e-9;  // [s]
}

void HuskyActiveGuidance::setPosFound_B(double x_m, double y_m, double z_m)
{
    this->posFound_B[0] = x_m;
    this->posFound_B[1] = y_m;
    this->posFound_B[2] = z_m;
}

void HuskyActiveGuidance::setDefaultSunHat_N(double x, double y, double z)
{
    double candidate[3] = {x, y, z};
    if (!this->safeUnit(candidate, this->defaultSunHat_N)) {
        this->bskLogger.bskLog(BSK_ERROR, "HuskyActiveGuidance: default Sun vector cannot be near zero.");
    }
}

std::string HuskyActiveGuidance::getState() const
{
    return this->state;
}

void HuskyActiveGuidance::refreshThresholds()
{
    this->chargeEnterClearDeg = this->lostExclHalfDeg + 2.0;                         // [deg]
    this->chargeExitClearDeg = std::max(this->lostExclHalfDeg - 1.0, 0.0);           // [deg]
    this->chargeEnterSunVisDeg = this->earthHalfAngleDeg + 2.0;                      // [deg]
    this->chargeExitSunVisDeg = std::max(this->earthHalfAngleDeg - 1.0, 0.0);        // [deg]
}

void HuskyActiveGuidance::writeIdentityReference(uint64_t CurrentSimNanos)
{
    AttRefMsgPayload outMsg = this->attRefOutMsg.zeroMsgPayload;
    outMsg.sigma_RN[0] = 0.0;
    outMsg.sigma_RN[1] = 0.0;
    outMsg.sigma_RN[2] = 0.0;
    outMsg.omega_RN_N[0] = 0.0;
    outMsg.omega_RN_N[1] = 0.0;
    outMsg.omega_RN_N[2] = 0.0;
    outMsg.domega_RN_N[0] = 0.0;
    outMsg.domega_RN_N[1] = 0.0;
    outMsg.domega_RN_N[2] = 0.0;
    this->attRefOutMsg.write(&outMsg, this->moduleID, CurrentSimNanos);
}

bool HuskyActiveGuidance::safeUnit(const double vec[3], double out[3])
{
    const double mag = HuskyActiveGuidance::norm3(vec);
    if (mag < kTinyNorm) {
        return false;
    }

    out[0] = vec[0] / mag;
    out[1] = vec[1] / mag;
    out[2] = vec[2] / mag;
    return true;
}

double HuskyActiveGuidance::angleDegBetween(const double vecA[3], const double vecB[3])
{
    const double dot = clampUnitRange(HuskyActiveGuidance::dot3(vecA, vecB));
    return std::acos(dot) * kRad2Deg;
}

void HuskyActiveGuidance::cross3(const double vecA[3], const double vecB[3], double out[3])
{
    out[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
    out[1] = vecA[2] * vecB[0] - vecA[0] * vecB[2];
    out[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

double HuskyActiveGuidance::dot3(const double vecA[3], const double vecB[3])
{
    return vecA[0] * vecB[0] + vecA[1] * vecB[1] + vecA[2] * vecB[2];
}

double HuskyActiveGuidance::norm3(const double vec[3])
{
    return std::sqrt(HuskyActiveGuidance::dot3(vec, vec));
}

void HuskyActiveGuidance::computeCompromiseX(const double earthHat[3],
                                             const double sunHat[3],
                                             double rhoRad,
                                             double outX_B[3])
{
    double antiSun[3] = {-sunHat[0], -sunHat[1], -sunHat[2]};
    const double antiSunProjection = HuskyActiveGuidance::dot3(antiSun, earthHat);

    double u2[3] = {
        antiSun[0] - antiSunProjection * earthHat[0],
        antiSun[1] - antiSunProjection * earthHat[1],
        antiSun[2] - antiSunProjection * earthHat[2]
    };

    if (!HuskyActiveGuidance::safeUnit(u2, u2)) {
        double fallback[3] = {1.0, 0.0, 0.0};
        if (std::fabs(earthHat[0]) >= 0.9) {
            fallback[0] = 0.0;
            fallback[1] = 1.0;
        }

        const double fallbackProjection = HuskyActiveGuidance::dot3(fallback, earthHat);
        u2[0] = fallback[0] - fallbackProjection * earthHat[0];
        u2[1] = fallback[1] - fallbackProjection * earthHat[1];
        u2[2] = fallback[2] - fallbackProjection * earthHat[2];
        HuskyActiveGuidance::safeUnit(u2, u2);
    }

    outX_B[0] = std::cos(rhoRad) * earthHat[0] + std::sin(rhoRad) * u2[0];
    outX_B[1] = std::cos(rhoRad) * earthHat[1] + std::sin(rhoRad) * u2[1];
    outX_B[2] = std::cos(rhoRad) * earthHat[2] + std::sin(rhoRad) * u2[2];
    HuskyActiveGuidance::safeUnit(outX_B, outX_B);
}

void HuskyActiveGuidance::buildYzFrameAboutX(const double x_B[3],
                                             const double earthHat[3],
                                             double outY_B[3],
                                             double outZ_B[3])
{
    HuskyActiveGuidance::cross3(x_B, earthHat, outZ_B);
    if (!HuskyActiveGuidance::safeUnit(outZ_B, outZ_B)) {
        double fallbackY[3] = {0.0, 1.0, 0.0};
        HuskyActiveGuidance::cross3(x_B, fallbackY, outZ_B);
        if (!HuskyActiveGuidance::safeUnit(outZ_B, outZ_B)) {
            double fallbackZ[3] = {0.0, 0.0, 1.0};
            HuskyActiveGuidance::cross3(x_B, fallbackZ, outZ_B);
            HuskyActiveGuidance::safeUnit(outZ_B, outZ_B);
        }
    }

    HuskyActiveGuidance::cross3(outZ_B, x_B, outY_B);
    HuskyActiveGuidance::safeUnit(outY_B, outY_B);
}

HuskyActiveGuidance::RollSolveResult HuskyActiveGuidance::solveRollForLostClearance(
    const double x_B[3],
    const double earthHat[3],
    const double sunHat[3],
    const std::vector<const double*>& extraKeepoutHats)
{
    struct Candidate {
        double rollDeg;
        double scoreDeg;
        double y_B[3];
        double z_B[3];
    };

    double yTemp[3] = {0.0, 0.0, 0.0};
    double zTemp[3] = {0.0, 0.0, 0.0};
    this->buildYzFrameAboutX(x_B, earthHat, yTemp, zTemp);

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
        this->cross3(x_B, yTest, zTest);
        this->safeUnit(zTest, zTest);

        double scoreDeg = std::min(
            this->angleDegBetween(zTest, earthHat),
            this->angleDegBetween(zTest, sunHat)
        );

        for (const double* keepoutHat : extraKeepoutHats) {
            scoreDeg = std::min(scoreDeg, this->angleDegBetween(zTest, keepoutHat));
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

        if (!this->hasPrevRoll) {
            // Prefer the highest score when no previous roll is available.
            const double rank = -candidates[idx].scoreDeg;
            if (rank < bestRank) {
                bestRank = rank;
                bestIdx = idx;
            }
        } else {
            const double rank = wrapDeltaDeg(candidates[idx].rollDeg, this->prevRollDeg);
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
