#include "activeGuidance.h"
#include "activeGuidanceMath.h"

#include "architecture/utilities/rigidBodyKinematics.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <vector>

namespace {
constexpr double kEarthRadiusKm = 6371.0;                           // [km]
constexpr double kOrbitAltitudeKm = 400.0;                          // [km]
constexpr double kPi = 3.14159265358979323846;                      // [rad]
constexpr double kDeg2Rad = kPi / 180.0;                            // [rad/deg]
constexpr double kRad2Deg = 180.0 / kPi;                            // [deg/rad]
constexpr double kValidStatePosFloorM = 1.0;                        // [m]
constexpr double kEarthRadiusM = 6371000.0;                         // [m]
constexpr double kEarthRotationRateRadPerSec = 7.2921159e-5;        // [rad/s]
constexpr double kSecondsToNanos = 1.0e9;                           // [ns/s]

std::string trimWhitespace(const std::string& value)
{
    const auto isSpace = [](unsigned char character) {
        return std::isspace(character) != 0;
    };

    size_t first = 0U;
    while (first < value.size() && isSpace(static_cast<unsigned char>(value[first]))) {
        ++first;
    }

    size_t last = value.size();
    while (last > first && isSpace(static_cast<unsigned char>(value[last - 1U]))) {
        --last;
    }

    return value.substr(first, last - first);
}

bool parseStationToken(const std::string& token,
                       std::string* label,
                       double* latRad,
                       double* lonRad)
{
    std::stringstream tokenStream(token);
    std::string labelRaw;
    std::string latRaw;
    std::string lonRaw;

    if (!std::getline(tokenStream, labelRaw, ':')) {
        return false;
    }
    if (!std::getline(tokenStream, latRaw, ':')) {
        return false;
    }
    if (!std::getline(tokenStream, lonRaw, ':')) {
        return false;
    }

    labelRaw = trimWhitespace(labelRaw);
    latRaw = trimWhitespace(latRaw);
    lonRaw = trimWhitespace(lonRaw);

    if (labelRaw.empty() || latRaw.empty() || lonRaw.empty()) {
        return false;
    }

    try {
        const double latDeg = std::stod(latRaw);                    // [deg]
        const double lonDeg = std::stod(lonRaw);                    // [deg]
        *label = labelRaw;
        *latRad = latDeg * kDeg2Rad;                                // [rad]
        *lonRad = lonDeg * kDeg2Rad;                                // [rad]
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

ActiveGuidance::ActiveGuidance()
{
    // rhoRad is Earth's apparent half-angle from a circular 400 km orbit:
    // rho = asin(Re / (Re + h)).
    // This is the cone half-angle used for "Earth-limb" pointing in EXPERIMENT mode.
    this->rhoRad = ActiveGuidanceMath::earthLimbHalfAngleRad(kEarthRadiusKm, kOrbitAltitudeKm);
    this->earthHalfAngleDeg = this->rhoRad * kRad2Deg;
    this->refreshThresholds();
}

ActiveGuidance::~ActiveGuidance()
{
    return;
}

void ActiveGuidance::Reset(uint64_t CurrentSimNanos)
{
    this->hasPrevRoll = false;
    this->state = "UNINITIALIZED";
    this->lastStatusPrintNanos = CurrentSimNanos;
    this->hasPrintedStatus = false;
    this->activeGroundStation = "NONE";
    this->downlinkWindowActive = false;
    this->downlinkWindowStation.clear();
    this->hadVisibleStationLastStep = false;
    this->lastVisibleStationLabel.clear();
    this->gnssFixScheduleInitialized = false;
    this->nextGnssFixNanos = 0U;
    this->gnssFixEndNanos = 0U;
    this->lastGnssGoodNanos = CurrentSimNanos;
    this->hasGnssGoodTimestamp = false;

    this->writeIdentityReference(CurrentSimNanos);

    if (!this->scStateInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance requires scStateInMsg to be connected.");
    }
}

void ActiveGuidance::UpdateState(uint64_t CurrentSimNanos)
{
    // Fail-safe behavior: if required navigation data is unavailable,
    // publish identity reference (zero attitude command, zero rates).
    if (!this->scStateInMsg.isLinked() || !this->scStateInMsg.isWritten()) {
        this->writeIdentityReference(CurrentSimNanos);
        return;
    }

    const SCStatesMsgPayload scState = this->scStateInMsg();
    double r_N[3] = {scState.r_BN_N[0], scState.r_BN_N[1], scState.r_BN_N[2]};
    const double rMag = std::sqrt(r_N[0] * r_N[0] + r_N[1] * r_N[1] + r_N[2] * r_N[2]);

    // If position is invalid/near-zero, geometric LOS vectors become undefined.
    if (rMag < kValidStatePosFloorM) {
        this->writeIdentityReference(CurrentSimNanos);
        return;
    }

    // Earth direction as seen from spacecraft COM (nadir line-of-sight).
    double earthHat_sc[3] = {
        -r_N[0] / rMag,
        -r_N[1] / rMag,
        -r_N[2] / rMag
    };

    // Sun line-of-sight from spacecraft COM. If no Sun message exists, fall back to configured inertial direction.
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

        // Convert absolute Sun position to spacecraft->Sun LOS.
        double sunRel_N[3] = {
            sunAbs_N[0] - r_N[0],
            sunAbs_N[1] - r_N[1],
            sunAbs_N[2] - r_N[2]
        };
        if (ActiveGuidanceMath::safeUnit(sunRel_N, sunHat_sc)) {
            hasSunAbs = true;
        }
    }

    // Optional Moon line-of-sight from spacecraft COM, used as an extra LOST keep-out body in EXPERIMENT mode.
    double moonHat_sc[3] = {0.0, 0.0, 0.0};
    bool hasMoonHat = false;
    if (this->moonStateInMsg.isLinked() && this->moonStateInMsg.isWritten()) {
        const SpicePlanetStateMsgPayload moonState = this->moonStateInMsg();
        double moonRel_N[3] = {
            moonState.PositionVector[0] - r_N[0],
            moonState.PositionVector[1] - r_N[1],
            moonState.PositionVector[2] - r_N[2]
        };
        hasMoonHat = ActiveGuidanceMath::safeUnit(moonRel_N, moonHat_sc);
    }

    double c_BN[3][3];
    double sigma_BN[3] = {scState.sigma_BN[0], scState.sigma_BN[1], scState.sigma_BN[2]};
    MRP2C(sigma_BN, c_BN);

    // FOUND camera is physically offset from COM.
    // Rotate that body-frame offset into inertial and add it to COM position.
    // Using FOUND's true location keeps Earth/Sun cone tests accurate near boundaries.
    // MRP2C returns C_BN (inertial->body). To rotate body vector into inertial,
    // we use C_NB = C_BN^T. The explicit indexing below performs that transpose multiply.
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

    // Comms antenna is physically offset from COM.
    // Use the antenna location for downlink LOS geometry instead of COM.
    double posCommsOffset_N[3] = {
        c_BN[0][0] * this->posComms_B[0] + c_BN[1][0] * this->posComms_B[1] + c_BN[2][0] * this->posComms_B[2],
        c_BN[0][1] * this->posComms_B[0] + c_BN[1][1] * this->posComms_B[1] + c_BN[2][1] * this->posComms_B[2],
        c_BN[0][2] * this->posComms_B[0] + c_BN[1][2] * this->posComms_B[1] + c_BN[2][2] * this->posComms_B[2]
    };
    double commsPos_N[3] = {
        r_N[0] + posCommsOffset_N[0],
        r_N[1] + posCommsOffset_N[1],
        r_N[2] + posCommsOffset_N[2]
    };

    const double foundPosMag = std::sqrt(
        foundPos_N[0] * foundPos_N[0] +
        foundPos_N[1] * foundPos_N[1] +
        foundPos_N[2] * foundPos_N[2]
    );
    // Recompute Earth direction from FOUND camera location (not COM).
    double earthHat_found[3] = {earthHat_sc[0], earthHat_sc[1], earthHat_sc[2]};
    if (foundPosMag >= kValidStatePosFloorM) {
        earthHat_found[0] = -foundPos_N[0] / foundPosMag;
        earthHat_found[1] = -foundPos_N[1] / foundPosMag;
        earthHat_found[2] = -foundPos_N[2] / foundPosMag;
    }

    // Recompute Sun direction from FOUND camera location (not COM).
    double sunHat_found[3] = {sunHat_sc[0], sunHat_sc[1], sunHat_sc[2]};
    if (hasSunAbs) {
        double sunRelFound_N[3] = {
            sunAbs_N[0] - foundPos_N[0],
            sunAbs_N[1] - foundPos_N[1],
            sunAbs_N[2] - foundPos_N[2]
        };
        ActiveGuidanceMath::safeUnit(sunRelFound_N, sunHat_found);
    }

    // CHARGING target: point body -X at the Sun so body +X (FOUND boresight) stays anti-sun.
    double rollOnlyX_B[3] = {-sunHat_found[0], -sunHat_found[1], -sunHat_found[2]};
    std::vector<const double*> noExtras;
    // For this +X choice, sweep roll to maximize LOST (+Z) minimum separation from keep-out bodies.
    const ActiveGuidanceMath::RollSolveResult rollOnlyResult = ActiveGuidanceMath::solveRollForLostClearance(
        rollOnlyX_B,
        earthHat_sc,
        sunHat_sc,
        noExtras,
        this->hasPrevRoll,
        this->prevRollDeg
    );
    // Interpretation:
    // rollOnlyResult.scoreDeg = max_roll min(ang(LOST, Earth), ang(LOST, Sun))
    // in CHARGING geometry (Moon intentionally excluded from CHARGING gate).

    // If Sun-Earth angle is below Earth's apparent half-angle, Sun is geometrically hidden by Earth.
    // HYBRID uses this as a Sun-visibility gate before allowing CHARGING.
    const double sunEarthAngleDeg = ActiveGuidanceMath::angleDegBetween(sunHat_found, earthHat_found);

    double x_B[3] = {rollOnlyX_B[0], rollOnlyX_B[1], rollOnlyX_B[2]};
    ActiveGuidanceMath::RollSolveResult selectedRoll = rollOnlyResult;
    std::string selectedState = "CHARGING";
    bool selectedIsCharging = true;
    this->activeGroundStation = "NONE";

    auto copyRollResult = [](const ActiveGuidanceMath::RollSolveResult& src,
                             ActiveGuidanceMath::RollSolveResult* dst) {
        dst->rollDeg = src.rollDeg;
        dst->scoreDeg = src.scoreDeg;
        dst->y_B[0] = src.y_B[0];
        dst->y_B[1] = src.y_B[1];
        dst->y_B[2] = src.y_B[2];
        dst->z_B[0] = src.z_B[0];
        dst->z_B[1] = src.z_B[1];
        dst->z_B[2] = src.z_B[2];
    };

    auto assignFrameSolution = [&](const double frameX_B[3],
                                   const double frameY_B[3],
                                   const double frameZ_B[3]) {
        x_B[0] = frameX_B[0];
        x_B[1] = frameX_B[1];
        x_B[2] = frameX_B[2];
        selectedRoll.rollDeg = 0.0;                                 // [deg]
        selectedRoll.scoreDeg = rollOnlyResult.scoreDeg;            // [deg]
        selectedRoll.y_B[0] = frameY_B[0];
        selectedRoll.y_B[1] = frameY_B[1];
        selectedRoll.y_B[2] = frameY_B[2];
        selectedRoll.z_B[0] = frameZ_B[0];
        selectedRoll.z_B[1] = frameZ_B[1];
        selectedRoll.z_B[2] = frameZ_B[2];
    };

    bool experimentContext = false;

    if (this->mode == MODE_EXPERIMENT) {
        experimentContext = true;
    } else if (this->mode == MODE_HYBRID) {
        bool chargingExitBand = (this->state == "CHARGING");
        const double clearThresholdDeg = chargingExitBand ? this->chargeExitClearDeg : this->chargeEnterClearDeg;
        const double sunVisThresholdDeg = chargingExitBand ? this->chargeExitSunVisDeg : this->chargeEnterSunVisDeg;

        // Enter/exit thresholds create hysteresis to avoid chatter between CHARGING and EXPERIMENT.
        const bool canCharge =
            (rollOnlyResult.scoreDeg >= clearThresholdDeg) &&
            (sunEarthAngleDeg >= sunVisThresholdDeg);
        // First inequality ensures LOST has margin to Earth+Sun in CHARGING.
        // Second inequality ensures Sun is not eclipsed by Earth from FOUND viewpoint.

        if (canCharge) {
            selectedState = "CHARGING";
            selectedIsCharging = true;
            x_B[0] = rollOnlyX_B[0];
            x_B[1] = rollOnlyX_B[1];
            x_B[2] = rollOnlyX_B[2];
            copyRollResult(rollOnlyResult, &selectedRoll);
        } else {
            experimentContext = true;
        }
    }

    if (experimentContext) {
        selectedIsCharging = false;

        if (!this->hasGnssGoodTimestamp) {
            this->lastGnssGoodNanos = CurrentSimNanos;
            this->hasGnssGoodTimestamp = true;
        }

        const bool deadReckoningExceeded =
            (CurrentSimNanos - this->lastGnssGoodNanos) >= this->gnssDeadReckoningNanos;

        if (!this->gnssFixScheduleInitialized) {
            this->nextGnssFixNanos = CurrentSimNanos + this->gnssFixPeriodNanos;
            this->gnssFixScheduleInitialized = true;
        }

        if (this->gnssFixEndNanos > 0U && CurrentSimNanos >= this->gnssFixEndNanos) {
            this->gnssFixEndNanos = 0U;
        }

        if (this->gnssFixEndNanos == 0U &&
            (CurrentSimNanos >= this->nextGnssFixNanos || deadReckoningExceeded)) {
            this->gnssFixEndNanos = CurrentSimNanos + this->gnssFixDurationNanos;
            this->nextGnssFixNanos = CurrentSimNanos + this->gnssFixPeriodNanos;
        }

        if (this->gnssFixEndNanos > CurrentSimNanos) {
            // GNSS_FIX state: antenna (-Z) points to zenith to maximize sky view.
            double minusZTargetHat[3] = {-earthHat_sc[0], -earthHat_sc[1], -earthHat_sc[2]};
            double gnssX_B[3] = {0.0, 0.0, 0.0};
            double gnssY_B[3] = {0.0, 0.0, 0.0};
            double gnssZ_B[3] = {0.0, 0.0, 0.0};
            if (ActiveGuidanceMath::buildFrameForMinusZTarget(minusZTargetHat, sunHat_sc, gnssX_B, gnssY_B, gnssZ_B)) {
                assignFrameSolution(gnssX_B, gnssY_B, gnssZ_B);
                selectedState = "GNSS_FIX";
            }
        } else {
            // EXPERIMENT base geometry when GNSS fix is not currently active.
            ActiveGuidanceMath::computeCompromiseX(earthHat_found, sunHat_found, this->rhoRad, x_B);
            std::vector<const double*> experimentExtras;
            if (hasMoonHat) {
                experimentExtras.push_back(moonHat_sc);
            }
            const ActiveGuidanceMath::RollSolveResult experimentRoll = ActiveGuidanceMath::solveRollForLostClearance(
                x_B,
                earthHat_sc,
                sunHat_sc,
                experimentExtras,
                this->hasPrevRoll,
                this->prevRollDeg
            );
            copyRollResult(experimentRoll, &selectedRoll);
            selectedState = "EXPERIMENT";

            std::string visibleStation;
            double visibleLosHat_N[3] = {0.0, 0.0, 0.0};
            const bool hasVisibleStation = this->selectVisibleGroundStation(
                commsPos_N,
                CurrentSimNanos,
                &visibleStation,
                visibleLosHat_N
            );

            if (this->downlinkWindowActive) {
                const bool windowExpired = CurrentSimNanos >= this->downlinkWindowEndNanos;
                const bool stationStillVisible = hasVisibleStation && (visibleStation == this->downlinkWindowStation);
                if (windowExpired || !stationStillVisible) {
                    this->downlinkWindowActive = false;
                    this->downlinkWindowStation.clear();
                }
            }

            const bool visibilityRisingEdge =
                hasVisibleStation &&
                (!this->hadVisibleStationLastStep || visibleStation != this->lastVisibleStationLabel);

            if (!this->downlinkWindowActive && visibilityRisingEdge) {
                this->downlinkWindowActive = true;
                this->downlinkWindowStation = visibleStation;
                this->downlinkWindowEndNanos = CurrentSimNanos + this->downlinkWindowNanos;
            }

            if (this->downlinkWindowActive) {
                std::string activeLabel;
                double activeLosHat_N[3] = {0.0, 0.0, 0.0};
                const bool activeVisible = this->selectVisibleGroundStation(
                    commsPos_N,
                    CurrentSimNanos,
                    &activeLabel,
                    activeLosHat_N
                );

                if (activeVisible && activeLabel == this->downlinkWindowStation) {
                    double downlinkX_B[3] = {0.0, 0.0, 0.0};
                    double downlinkY_B[3] = {0.0, 0.0, 0.0};
                    double downlinkZ_B[3] = {0.0, 0.0, 0.0};
                    if (ActiveGuidanceMath::buildFrameForPlusXTarget(
                            activeLosHat_N,
                            sunHat_sc,
                            downlinkX_B,
                            downlinkY_B,
                            downlinkZ_B)) {
                        assignFrameSolution(downlinkX_B, downlinkY_B, downlinkZ_B);
                        selectedState = "DOWNLINK";
                        this->activeGroundStation = this->downlinkWindowStation;
                    }
                }
            }

            this->hadVisibleStationLastStep = hasVisibleStation;
            this->lastVisibleStationLabel = hasVisibleStation ? visibleStation : "";
        }
    } else {
        // GNSS fixing is only required during experiment context.
        this->gnssFixScheduleInitialized = false;
        this->nextGnssFixNanos = 0U;
        this->gnssFixEndNanos = 0U;
        this->hasGnssGoodTimestamp = false;

        this->downlinkWindowActive = false;
        this->downlinkWindowStation.clear();
        this->hadVisibleStationLastStep = false;
        this->lastVisibleStationLabel.clear();
    }

    double antennaHat_N[3] = {
        -selectedRoll.z_B[0],
        -selectedRoll.z_B[1],
        -selectedRoll.z_B[2]
    };
    double zenithHat_sc[3] = {
        -earthHat_sc[0],
        -earthHat_sc[1],
        -earthHat_sc[2]
    };
    const double antennaZenithAngleDeg = ActiveGuidanceMath::angleDegBetween(antennaHat_N, zenithHat_sc);

    if (experimentContext && antennaZenithAngleDeg <= this->gnssZenithHalfAngleDeg) {
        this->lastGnssGoodNanos = CurrentSimNanos;
        this->hasGnssGoodTimestamp = true;
    }

    if (this->state != selectedState) {
        const double tSec = static_cast<double>(CurrentSimNanos) * 1.0e-9;  // [s]
        const char* panelCmd = selectedIsCharging ? "OPEN" : "STOW";
        this->bskLogger.bskLog(
            BSK_INFORMATION,
            "[ADCS] t=%8.1fs -> %s (roll-only LOST clearance=%5.1f deg, panel cmd=%s, station=%s, zenith-angle=%5.1f deg)",
            tSec,
            selectedState.c_str(),
            rollOnlyResult.scoreDeg,
            panelCmd,
            this->activeGroundStation.c_str(),
            antennaZenithAngleDeg
        );
    }

    if (this->mode == MODE_HYBRID) {
        const bool periodElapsed = (CurrentSimNanos - this->lastStatusPrintNanos) >= this->statusPeriodNanos;
        if (!this->hasPrintedStatus || periodElapsed) {
            const double tSec = static_cast<double>(CurrentSimNanos) * 1.0e-9;  // [s]
            const char* panelCmd = selectedIsCharging ? "OPEN" : "STOW";
            this->bskLogger.bskLog(
                BSK_INFORMATION,
                "[HYBRID] t=%8.1fs active=%s panel=%s station=%s roll-only-clearance=%5.1f deg sun-earth-angle=%5.1f deg zenith-angle=%5.1f deg",
                tSec,
                selectedState.c_str(),
                panelCmd,
                this->activeGroundStation.c_str(),
                rollOnlyResult.scoreDeg,
                sunEarthAngleDeg,
                antennaZenithAngleDeg
            );
            this->lastStatusPrintNanos = CurrentSimNanos;
            this->hasPrintedStatus = true;
        }
    }

    this->state = selectedState;
    this->prevRollDeg = selectedRoll.rollDeg;
    this->hasPrevRoll = true;

    // Reference DCM rows are inertial components of body axes [x_B; y_B; z_B].
    double dcm_RN[3][3] = {
        {x_B[0], x_B[1], x_B[2]},
        {selectedRoll.y_B[0], selectedRoll.y_B[1], selectedRoll.y_B[2]},
        {selectedRoll.z_B[0], selectedRoll.z_B[1], selectedRoll.z_B[2]}
    };

    double sigma_RN[3] = {0.0, 0.0, 0.0};
    C2MRP(dcm_RN, sigma_RN);

    // Rare numerical guard for degenerate frame cases.
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

void ActiveGuidance::setModeString(const std::string& modeString)
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

    this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: unsupported mode string '%s'.", modeString.c_str());
}

std::string ActiveGuidance::getModeString() const
{
    return this->modeString;
}

void ActiveGuidance::setLostExclHalfDeg(double valueDeg)
{
    if (valueDeg <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: lost exclusion half-angle must be > 0 deg.");
        return;
    }

    this->lostExclHalfDeg = valueDeg;
    this->refreshThresholds();
}

double ActiveGuidance::getLostExclHalfDeg() const
{
    return this->lostExclHalfDeg;
}

void ActiveGuidance::setStatusPeriodSec(double valueSec)
{
    if (valueSec <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: status period must be > 0 s.");
        return;
    }

    this->statusPeriodNanos = static_cast<uint64_t>(valueSec * kSecondsToNanos);  // [ns]
}

double ActiveGuidance::getStatusPeriodSec() const
{
    return static_cast<double>(this->statusPeriodNanos) * 1.0e-9;  // [s]
}

void ActiveGuidance::setGnssFixPeriodSec(double valueSec)
{
    if (valueSec <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: GNSS fix period must be > 0 s.");
        return;
    }

    this->gnssFixPeriodNanos = static_cast<uint64_t>(valueSec * kSecondsToNanos);  // [ns]
}

void ActiveGuidance::setGnssFixDurationSec(double valueSec)
{
    if (valueSec <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: GNSS fix duration must be > 0 s.");
        return;
    }

    this->gnssFixDurationNanos = static_cast<uint64_t>(valueSec * kSecondsToNanos);  // [ns]
}

void ActiveGuidance::setGnssZenithHalfAngleDeg(double valueDeg)
{
    if (valueDeg <= 0.0 || valueDeg > 90.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: GNSS zenith half-angle must be in (0, 90] deg.");
        return;
    }

    this->gnssZenithHalfAngleDeg = valueDeg;
}

void ActiveGuidance::setGnssDeadReckoningSec(double valueSec)
{
    if (valueSec <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: GNSS dead-reckoning horizon must be > 0 s.");
        return;
    }

    this->gnssDeadReckoningNanos = static_cast<uint64_t>(valueSec * kSecondsToNanos);  // [ns]
}

void ActiveGuidance::setDownlinkWindowSec(double valueSec)
{
    if (valueSec <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: downlink window must be > 0 s.");
        return;
    }

    this->downlinkWindowNanos = static_cast<uint64_t>(valueSec * kSecondsToNanos);  // [ns]
}

void ActiveGuidance::setGroundStationsCsv(const std::string& stationsCsv)
{
    this->groundStations.clear();

    const std::string trimmedCsv = trimWhitespace(stationsCsv);
    if (trimmedCsv.empty()) {
        return;
    }

    std::stringstream csvStream(trimmedCsv);
    std::string token;
    while (std::getline(csvStream, token, ';')) {
        const std::string trimmedToken = trimWhitespace(token);
        if (trimmedToken.empty()) {
            continue;
        }

        GroundStation station = {};
        if (!parseStationToken(trimmedToken, &station.label, &station.latRad, &station.lonRad)) {
            this->bskLogger.bskLog(
                BSK_ERROR,
                "ActiveGuidance: invalid ground station token '%s'. Expected label:lat_deg:lon_deg.",
                trimmedToken.c_str()
            );
            continue;
        }

        this->groundStations.push_back(station);
    }
}

void ActiveGuidance::setPosFound_B(double x_m, double y_m, double z_m)
{
    this->posFound_B[0] = x_m;
    this->posFound_B[1] = y_m;
    this->posFound_B[2] = z_m;
}

void ActiveGuidance::setPosComms_B(double x_m, double y_m, double z_m)
{
    this->posComms_B[0] = x_m;
    this->posComms_B[1] = y_m;
    this->posComms_B[2] = z_m;
}

void ActiveGuidance::setDefaultSunHat_N(double x, double y, double z)
{
    double candidate[3] = {x, y, z};
    if (!ActiveGuidanceMath::safeUnit(candidate, this->defaultSunHat_N)) {
        this->bskLogger.bskLog(BSK_ERROR, "ActiveGuidance: default Sun vector cannot be near zero.");
    }
}

std::string ActiveGuidance::getState() const
{
    return this->state;
}

std::string ActiveGuidance::getActiveGroundStation() const
{
    return this->activeGroundStation;
}

bool ActiveGuidance::selectVisibleGroundStation(const double scPos_N[3],
                                                uint64_t CurrentSimNanos,
                                                std::string* stationLabel,
                                                double stationLosHat_N[3]) const
{
    if (this->groundStations.empty()) {
        return false;
    }

    const double tSec = static_cast<double>(CurrentSimNanos) * 1.0e-9;                 // [s]
    const double earthThetaRad = kEarthRotationRateRadPerSec * tSec;                   // [rad]
    const double cosTheta = std::cos(earthThetaRad);
    const double sinTheta = std::sin(earthThetaRad);

    bool hasVisibleStation = false;
    double bestElevationProxy = -2.0;

    for (const GroundStation& station : this->groundStations) {
        const double cosLat = std::cos(station.latRad);
        const double sinLat = std::sin(station.latRad);
        const double cosLon = std::cos(station.lonRad);
        const double sinLon = std::sin(station.lonRad);

        const double stationPosEcef[3] = {
            kEarthRadiusM * cosLat * cosLon,
            kEarthRadiusM * cosLat * sinLon,
            kEarthRadiusM * sinLat
        };

        // Approximate Earth-fixed station drift in inertial frame using a simple z-axis rotation.
        const double stationPos_N[3] = {
            cosTheta * stationPosEcef[0] - sinTheta * stationPosEcef[1],
            sinTheta * stationPosEcef[0] + cosTheta * stationPosEcef[1],
            stationPosEcef[2]
        };

        double stationZenithHat_N[3] = {0.0, 0.0, 0.0};
        if (!ActiveGuidanceMath::safeUnit(stationPos_N, stationZenithHat_N)) {
            continue;
        }

        const double stationToSc_N[3] = {
            scPos_N[0] - stationPos_N[0],
            scPos_N[1] - stationPos_N[1],
            scPos_N[2] - stationPos_N[2]
        };
        double stationToScHat_N[3] = {0.0, 0.0, 0.0};
        if (!ActiveGuidanceMath::safeUnit(stationToSc_N, stationToScHat_N)) {
            continue;
        }

        const double elevationProxy =
            stationZenithHat_N[0] * stationToScHat_N[0] +
            stationZenithHat_N[1] * stationToScHat_N[1] +
            stationZenithHat_N[2] * stationToScHat_N[2];

        if (elevationProxy <= 0.0) {
            continue;
        }

        const double scToStation_N[3] = {
            stationPos_N[0] - scPos_N[0],
            stationPos_N[1] - scPos_N[1],
            stationPos_N[2] - scPos_N[2]
        };
        double scToStationHat_N[3] = {0.0, 0.0, 0.0};
        if (!ActiveGuidanceMath::safeUnit(scToStation_N, scToStationHat_N)) {
            continue;
        }

        if (!hasVisibleStation || elevationProxy > bestElevationProxy) {
            hasVisibleStation = true;
            bestElevationProxy = elevationProxy;
            *stationLabel = station.label;
            stationLosHat_N[0] = scToStationHat_N[0];
            stationLosHat_N[1] = scToStationHat_N[1];
            stationLosHat_N[2] = scToStationHat_N[2];
        }
    }

    return hasVisibleStation;
}

void ActiveGuidance::refreshThresholds()
{
    // HYBRID hysteresis:
    // - entering CHARGING is stricter (+2 deg)
    // - exiting CHARGING is looser (-1 deg)
    // This prevents rapid state toggling near boundaries.
    this->chargeEnterClearDeg = this->lostExclHalfDeg + 2.0;                         // [deg]
    this->chargeExitClearDeg = std::max(this->lostExclHalfDeg - 1.0, 0.0);           // [deg]
    this->chargeEnterSunVisDeg = this->earthHalfAngleDeg + 2.0;                      // [deg]
    this->chargeExitSunVisDeg = std::max(this->earthHalfAngleDeg - 1.0, 0.0);        // [deg]
}

void ActiveGuidance::writeIdentityReference(uint64_t CurrentSimNanos)
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
