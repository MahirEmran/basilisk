#include "activeGuidanceStandalone.h"
#include "activeGuidanceMath.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <chrono>

namespace {
constexpr double kEarthRadiusKm = 6371.0;
constexpr double kOrbitAltitudeKm = 400.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kSecondsToNanos = 1.0e9;

std::string trimWhitespace(const std::string& value) {
    const auto isSpace = [](unsigned char character) {
        return std::isspace(character) != 0;
    };

    size_t first = 0;
    while (first < value.size() && isSpace(static_cast<unsigned char>(value[first]))) {
        ++first;
    }

    size_t last = value.size();
    while (last > first && isSpace(static_cast<unsigned char>(value[last - 1]))) {
        --last;
    }

    return value.substr(first, last - first);
}

bool parseStationToken(const std::string& token,
                       std::string* label,
                       double* latRad,
                       double* lonRad) {
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
        const double latDeg = std::stod(latRaw);
        const double lonDeg = std::stod(lonRaw);
        *label = labelRaw;
        *latRad = latDeg * kDeg2Rad;
        *lonRad = lonDeg * kDeg2Rad;
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

ActiveGuidanceStandalone::ActiveGuidanceStandalone()
    : mode_(MODE_HYBRID)
    , modeString_("HYBRID")
    , state_("UNINITIALIZED")
    , lostExclHalfDeg_(17.5)
    , chargeEnterClearDeg_(19.5)
    , chargeExitClearDeg_(16.5)
    , earthHalfAngleDeg_(70.210073)
    , chargeEnterSunVisDeg_(72.210073)
    , chargeExitSunVisDeg_(69.210073)
    , statusPeriodNanos_(60000000000ULL)
    , lastStatusPrintNanos_(0U)
    , hasPrintedStatus_(false)
    , gnssFixPeriodNanos_(600000000000ULL)
    , gnssFixDurationNanos_(60000000000ULL)
    , gnssZenithHalfAngleDeg_(45.0)
    , gnssDeadReckoningNanos_(900000000000ULL)
    , nextGnssFixNanos_(0U)
    , gnssFixEndNanos_(0U)
    , lastGnssGoodNanos_(0U)
    , hasGnssGoodTimestamp_(false)
    , gnssFixScheduleInitialized_(false)
    , downlinkWindowNanos_(420000000000ULL)
    , downlinkWindowEndNanos_(0U)
    , downlinkWindowActive_(false)
    , downlinkWindowStation_("")
    , activeGroundStation_("NONE")
    , hadVisibleStationLastStep_(false)
    , lastVisibleStationLabel_("")
    , prevRollDeg_(0.0)
    , hasPrevRoll_(false) {

    // Initialize position vectors
    posFound_B_[0] = 0.05;
    posFound_B_[1] = 0.0;
    posFound_B_[2] = 0.105;

    defaultSunHat_N_[0] = 1.0;
    defaultSunHat_N_[1] = 0.0;
    defaultSunHat_N_[2] = 0.0;

    // Calculate Earth limb half angle
    rhoRad_ = ActiveGuidanceMath::earthLimbHalfAngleRad(kEarthRadiusKm, kOrbitAltitudeKm);
    earthHalfAngleDeg_ = rhoRad_ * kRad2Deg;

    refreshThresholds();
}

ActiveGuidanceStandalone::~ActiveGuidanceStandalone() {
    return;
}

void ActiveGuidanceStandalone::Reset(uint64_t currentSimNanos) {
    state_ = "UNINITIALIZED";
    lastStatusPrintNanos_ = currentSimNanos;
    hasPrintedStatus_ = false;

    nextGnssFixNanos_ = currentSimNanos + gnssFixPeriodNanos_;
    gnssFixEndNanos_ = 0U;
    lastGnssGoodNanos_ = 0U;
    hasGnssGoodTimestamp_ = false;
    gnssFixScheduleInitialized_ = false;

    downlinkWindowEndNanos_ = 0U;
    downlinkWindowActive_ = false;
    downlinkWindowStation_ = "";
    activeGroundStation_ = "NONE";
    hadVisibleStationLastStep_ = false;
    lastVisibleStationLabel_ = "";

    prevRollDeg_ = 0.0;
    hasPrevRoll_ = false;

    refreshThresholds();
}

void ActiveGuidanceStandalone::UpdateState(uint64_t currentSimNanos) {
    // Simplified state machine for standalone version
    // In a full implementation, this would include all the guidance logic

    if (state_ == "UNINITIALIZED") {
        state_ = "NOMINAL";
    }

    // Print status periodically
    if (currentSimNanos - lastStatusPrintNanos_ >= statusPeriodNanos_) {
        lastStatusPrintNanos_ = currentSimNanos;
        hasPrintedStatus_ = true;
    }
}

void ActiveGuidanceStandalone::setModeString(const std::string& modeString) {
    modeString_ = modeString;

    if (modeString == "ROLL_ONLY") {
        mode_ = MODE_ROLL_ONLY;
    } else if (modeString == "EXPERIMENT") {
        mode_ = MODE_EXPERIMENT;
    } else {
        mode_ = MODE_HYBRID;
    }
}

std::string ActiveGuidanceStandalone::getModeString() const {
    return modeString_;
}

void ActiveGuidanceStandalone::setLostExclHalfDeg(double valueDeg) {
    lostExclHalfDeg_ = valueDeg;
    refreshThresholds();
}

double ActiveGuidanceStandalone::getLostExclHalfDeg() const {
    return lostExclHalfDeg_;
}

void ActiveGuidanceStandalone::setStatusPeriodSec(double valueSec) {
    statusPeriodNanos_ = static_cast<uint64_t>(valueSec * kSecondsToNanos);
}

double ActiveGuidanceStandalone::getStatusPeriodSec() const {
    return static_cast<double>(statusPeriodNanos_) / kSecondsToNanos;
}

void ActiveGuidanceStandalone::setGnssFixPeriodSec(double valueSec) {
    gnssFixPeriodNanos_ = static_cast<uint64_t>(valueSec * kSecondsToNanos);
}

double ActiveGuidanceStandalone::getGnssFixPeriodSec() const {
    return static_cast<double>(gnssFixPeriodNanos_) / kSecondsToNanos;
}

void ActiveGuidanceStandalone::setGnssFixDurationSec(double valueSec) {
    gnssFixDurationNanos_ = static_cast<uint64_t>(valueSec * kSecondsToNanos);
}

double ActiveGuidanceStandalone::getGnssFixDurationSec() const {
    return static_cast<double>(gnssFixDurationNanos_) / kSecondsToNanos;
}

void ActiveGuidanceStandalone::setGnssZenithHalfAngleDeg(double valueDeg) {
    gnssZenithHalfAngleDeg_ = valueDeg;
}

double ActiveGuidanceStandalone::getGnssZenithHalfAngleDeg() const {
    return gnssZenithHalfAngleDeg_;
}

void ActiveGuidanceStandalone::setGnssDeadReckoningSec(double valueSec) {
    gnssDeadReckoningNanos_ = static_cast<uint64_t>(valueSec * kSecondsToNanos);
}

double ActiveGuidanceStandalone::getGnssDeadReckoningSec() const {
    return static_cast<double>(gnssDeadReckoningNanos_) / kSecondsToNanos;
}

void ActiveGuidanceStandalone::setDownlinkWindowSec(double valueSec) {
    downlinkWindowNanos_ = static_cast<uint64_t>(valueSec * kSecondsToNanos);
}

double ActiveGuidanceStandalone::getDownlinkWindowSec() const {
    return static_cast<double>(downlinkWindowNanos_) / kSecondsToNanos;
}

void ActiveGuidanceStandalone::setGroundStationsCsv(const std::string& stationsCsv) {
    groundStations_.clear();

    if (stationsCsv.empty()) {
        return;
    }

    std::stringstream ss(stationsCsv);
    std::string token;

    while (std::getline(ss, token, ',')) {
        std::string label;
        double latRad, lonRad;

        if (parseStationToken(token, &label, &latRad, &lonRad)) {
            GroundStation station;
            station.label = label;
            station.latRad = latRad;
            station.lonRad = lonRad;
            groundStations_.push_back(station);
        }
    }
}

void ActiveGuidanceStandalone::setPosFound_B(double x_m, double y_m, double z_m) {
    posFound_B_[0] = x_m;
    posFound_B_[1] = y_m;
    posFound_B_[2] = z_m;
}

void ActiveGuidanceStandalone::setDefaultSunHat_N(double x, double y, double z) {
    defaultSunHat_N_[0] = x;
    defaultSunHat_N_[1] = y;
    defaultSunHat_N_[2] = z;
}

std::string ActiveGuidanceStandalone::getState() const {
    return state_;
}

std::string ActiveGuidanceStandalone::getActiveGroundStation() const {
    return activeGroundStation_;
}

void ActiveGuidanceStandalone::refreshThresholds() {
    // Update thresholds based on current configuration
    chargeEnterClearDeg_ = lostExclHalfDeg_ + 2.0;
    chargeExitClearDeg_ = lostExclHalfDeg_ - 1.0;
    chargeEnterSunVisDeg_ = earthHalfAngleDeg_ + 2.0;
    chargeExitSunVisDeg_ = earthHalfAngleDeg_ - 1.0;
}

void ActiveGuidanceStandalone::writeIdentityReference(uint64_t currentSimNanos) {
    // Write identity reference (simplified for standalone version)
    // In full implementation, this would set the attitude reference
}

bool ActiveGuidanceStandalone::selectVisibleGroundStation(const double scPos_N[3],
                                                        uint64_t currentSimNanos,
                                                        std::string* stationLabel,
                                                        double stationLosHat_N[3]) const {
    // Simplified ground station selection
    // In full implementation, this would check visibility geometry

    if (groundStations_.empty()) {
        return false;
    }

    // Return first station as placeholder
    *stationLabel = groundStations_[0].label;

    // Set placeholder LOS vector
    stationLosHat_N[0] = 0.0;
    stationLosHat_N[1] = 0.0;
    stationLosHat_N[2] = -1.0;

    return true;
}