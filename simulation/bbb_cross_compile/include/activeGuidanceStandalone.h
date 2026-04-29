#ifndef ACTIVE_GUIDANCE_STANDALONE_H
#define ACTIVE_GUIDANCE_STANDALONE_H

#include <string>
#include <vector>
#include <cstdint>

/**
 * @brief Standalone Active Guidance module for BBB
 *
 * This is a simplified version of the ActiveGuidance module that doesn't
 * depend on the full Basilisk framework, making it suitable for
 * cross-compilation and standalone deployment.
 */
class ActiveGuidanceStandalone {
public:
    ActiveGuidanceStandalone();
    ~ActiveGuidanceStandalone();

    /**
     * @brief Reset the guidance module
     * @param currentSimNanos Current simulation time in nanoseconds
     */
    void Reset(uint64_t currentSimNanos);

    /**
     * @brief Update the guidance state
     * @param currentSimNanos Current simulation time in nanoseconds
     */
    void UpdateState(uint64_t currentSimNanos);

    /**
     * @brief Set the guidance mode string
     * @param modeString Mode string (e.g., "HYBRID", "ROLL_ONLY", "EXPERIMENT")
     */
    void setModeString(const std::string& modeString);

    /**
     * @brief Get the guidance mode string
     * @return Current mode string
     */
    std::string getModeString() const;

    /**
     * @brief Set LOST exclusion half angle
     * @param valueDeg Half angle in degrees
     */
    void setLostExclHalfDeg(double valueDeg);

    /**
     * @brief Get LOST exclusion half angle
     * @return Half angle in degrees
     */
    double getLostExclHalfDeg() const;

    /**
     * @brief Set status print period
     * @param valueSec Period in seconds
     */
    void setStatusPeriodSec(double valueSec);

    /**
     * @brief Get status print period
     * @return Period in seconds
     */
    double getStatusPeriodSec() const;

    /**
     * @brief Set GNSS fix period
     * @param valueSec Period in seconds
     */
    void setGnssFixPeriodSec(double valueSec);

    /**
     * @brief Get GNSS fix period
     * @return Period in seconds
     */
    double getGnssFixPeriodSec() const;

    /**
     * @brief Set GNSS fix duration
     * @param valueSec Duration in seconds
     */
    void setGnssFixDurationSec(double valueSec);

    /**
     * @brief Get GNSS fix duration
     * @return Duration in seconds
     */
    double getGnssFixDurationSec() const;

    /**
     * @brief Set GNSS zenith half angle
     * @param valueDeg Half angle in degrees
     */
    void setGnssZenithHalfAngleDeg(double valueDeg);

    /**
     * @brief Get GNSS zenith half angle
     * @return Half angle in degrees
     */
    double getGnssZenithHalfAngleDeg() const;

    /**
     * @brief Set GNSS dead reckoning time
     * @param valueSec Time in seconds
     */
    void setGnssDeadReckoningSec(double valueSec);

    /**
     * @brief Get GNSS dead reckoning time
     * @return Time in seconds
     */
    double getGnssDeadReckoningSec() const;

    /**
     * @brief Set downlink window duration
     * @param valueSec Duration in seconds
     */
    void setDownlinkWindowSec(double valueSec);

    /**
     * @brief Get downlink window duration
     * @return Duration in seconds
     */
    double getDownlinkWindowSec() const;

    /**
     * @brief Set ground stations CSV
     * @param stationsCsv CSV string of ground stations
     */
    void setGroundStationsCsv(const std::string& stationsCsv);

    /**
     * @brief Set FOUND camera position
     * @param x_m X position in meters
     * @param y_m Y position in meters
     * @param z_m Z position in meters
     */
    void setPosFound_B(double x_m, double y_m, double z_m);

    /**
     * @brief Set default Sun unit vector
     * @param x X component
     * @param y Y component
     * @param z Z component
     */
    void setDefaultSunHat_N(double x, double y, double z);

    /**
     * @brief Get current state
     * @return Current state string
     */
    std::string getState() const;

    /**
     * @brief Get active ground station
     * @return Active ground station label
     */
    std::string getActiveGroundStation() const;

private:
    // Guidance mode
    enum GuidanceMode {
        MODE_ROLL_ONLY = 0,
        MODE_EXPERIMENT = 1,
        MODE_HYBRID = 2
    };

    GuidanceMode mode_;
    std::string modeString_;
    std::string state_;

    // Configuration parameters
    double posFound_B_[3];
    double defaultSunHat_N_[3];
    double lostExclHalfDeg_;
    double chargeEnterClearDeg_;
    double chargeExitClearDeg_;
    double earthHalfAngleDeg_;
    double chargeEnterSunVisDeg_;
    double chargeExitSunVisDeg_;

    // Timing parameters
    uint64_t statusPeriodNanos_;
    uint64_t lastStatusPrintNanos_;
    bool hasPrintedStatus_;

    uint64_t gnssFixPeriodNanos_;
    uint64_t gnssFixDurationNanos_;
    double gnssZenithHalfAngleDeg_;
    uint64_t gnssDeadReckoningNanos_;
    uint64_t nextGnssFixNanos_;
    uint64_t gnssFixEndNanos_;
    uint64_t lastGnssGoodNanos_;
    bool hasGnssGoodTimestamp_;
    bool gnssFixScheduleInitialized_;

    uint64_t downlinkWindowNanos_;
    uint64_t downlinkWindowEndNanos_;
    bool downlinkWindowActive_;
    std::string downlinkWindowStation_;
    std::string activeGroundStation_;
    bool hadVisibleStationLastStep_;
    std::string lastVisibleStationLabel_;

    // Ground stations
    struct GroundStation {
        std::string label;
        double latRad;
        double lonRad;
    };
    std::vector<GroundStation> groundStations_;

    // State tracking
    double prevRollDeg_;
    bool hasPrevRoll_;
    double rhoRad_;

    // Helper methods
    void refreshThresholds();
    void writeIdentityReference(uint64_t currentSimNanos);
    bool selectVisibleGroundStation(const double scPos_N[3],
                                    uint64_t currentSimNanos,
                                    std::string* stationLabel,
                                    double stationLosHat_N[3]) const;
};

#endif // ACTIVE_GUIDANCE_STANDALONE_H