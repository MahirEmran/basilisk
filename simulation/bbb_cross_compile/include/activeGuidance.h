#ifndef ACTIVE_GUIDANCE_H
#define ACTIVE_GUIDANCE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

#include <string>
#include <vector>

class ActiveGuidance : public SysModel {
public:
    ActiveGuidance();
    ~ActiveGuidance() override;

    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    Message<AttRefMsgPayload> attRefOutMsg;                  //!< attitude reference output
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;            //!< spacecraft state input
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;   //!< Sun SPICE state input
    ReadFunctor<SpicePlanetStateMsgPayload> moonStateInMsg;  //!< Moon SPICE state input

    BSKLogger bskLogger;                                     //!< Basilisk logger

    void setModeString(const std::string& modeString);
    std::string getModeString() const;

    void setLostExclHalfDeg(double valueDeg);
    double getLostExclHalfDeg() const;

    void setStatusPeriodSec(double valueSec);
    double getStatusPeriodSec() const;

    void setGnssFixPeriodSec(double valueSec);
    void setGnssFixDurationSec(double valueSec);
    void setGnssZenithHalfAngleDeg(double valueDeg);
    void setGnssDeadReckoningSec(double valueSec);
    void setDownlinkWindowSec(double valueSec);
    void setGroundStationsCsv(const std::string& stationsCsv);

    void setPosFound_B(double x_m, double y_m, double z_m);
    void setPosComms_B(double x_m, double y_m, double z_m);
    void setDefaultSunHat_N(double x, double y, double z);

    std::string getState() const;
    std::string getActiveGroundStation() const;

private:
    enum GuidanceMode {
        MODE_ROLL_ONLY = 0,
        MODE_EXPERIMENT = 1,
        MODE_HYBRID = 2
    };

    GuidanceMode mode = MODE_HYBRID;                         //!< guidance mode
    std::string modeString = "HYBRID";                        //!< mode name for logging
    std::string state = "UNINITIALIZED";                      //!< active state string

    double posFound_B[3] = {0.05, 0.0, 0.105};              //!< [m] FOUND camera location in body frame (for LOS from camera, not COM)
    double posComms_B[3] = {0.05, 0.0, 0.0};                //!< [m] comms antenna location in body frame (for station LOS from antenna, not COM)
    double defaultSunHat_N[3] = {1.0, 0.0, 0.0};            //!< [-] fallback Sun unit vector in inertial frame

    double lostExclHalfDeg = 17.5;                          //!< [deg] LOST keep-out half angle (minimum body LOS separation from LOST +Z)
    double chargeEnterClearDeg = 19.5;                      //!< [deg] CHARGING entry threshold for roll-only LOST score (hysteresis high band)
    double chargeExitClearDeg = 16.5;                       //!< [deg] CHARGING exit threshold for roll-only LOST score (hysteresis low band)
    double earthHalfAngleDeg = 70.210073;                   //!< [deg] Earth apparent half-angle at 400 km
    double chargeEnterSunVisDeg = 72.210073;                //!< [deg] CHARGING entry threshold for Sun-Earth LOS separation
    double chargeExitSunVisDeg = 69.210073;                 //!< [deg] CHARGING exit threshold for Sun-Earth LOS separation

    uint64_t statusPeriodNanos = 60000000000ULL;            //!< [ns] status print period
    uint64_t lastStatusPrintNanos = 0U;                     //!< [ns] last status print time
    bool hasPrintedStatus = false;

    uint64_t gnssFixPeriodNanos = 600000000000ULL;          //!< [ns] periodic GNSS fix interval while in experiment context (10 min default)
    uint64_t gnssFixDurationNanos = 60000000000ULL;         //!< [ns] time spent in GNSS_FIX when a fix cycle starts
    double gnssZenithHalfAngleDeg = 45.0;                   //!< [deg] GNSS reception cone half-angle away from zenith
    uint64_t gnssDeadReckoningNanos = 900000000000ULL;      //!< [ns] maximum dead-reckoning time before GNSS fix is forced
    uint64_t nextGnssFixNanos = 0U;                         //!< [ns] next scheduled GNSS fix start
    uint64_t gnssFixEndNanos = 0U;                          //!< [ns] active GNSS fix end time
    uint64_t lastGnssGoodNanos = 0U;                        //!< [ns] last timestamp where antenna was inside GNSS zenith cone
    bool hasGnssGoodTimestamp = false;                      //!< [-] indicates lastGnssGoodNanos is initialized
    bool gnssFixScheduleInitialized = false;                //!< [-] indicates GNSS schedule has been initialized

    uint64_t downlinkWindowNanos = 420000000000ULL;         //!< [ns] max downlink dwell per visibility window (7 min default)
    uint64_t downlinkWindowEndNanos = 0U;                   //!< [ns] current downlink window end
    bool downlinkWindowActive = false;                      //!< [-] true while downlink window is active
    std::string downlinkWindowStation = "";                  //!< [-] label of station assigned to active downlink window
    std::string activeGroundStation = "NONE";                //!< [-] label of station currently selected for downlink
    bool hadVisibleStationLastStep = false;                 //!< [-] visibility edge detector for downlink window triggering
    std::string lastVisibleStationLabel = "";                //!< [-] station label from previous visibility sample

    struct GroundStation {
        std::string label;                                  //!< [-] station identifier
        double latRad;                                      //!< [rad] geodetic latitude approximation
        double lonRad;                                      //!< [rad] east longitude at epoch
    };
    std::vector<GroundStation> groundStations;              //!< [-] configured ground stations for downlink scheduling

    double prevRollDeg = 0.0;                               //!< [deg] previous chosen roll (used to prefer smooth roll continuity)
    bool hasPrevRoll = false;

    double rhoRad = 1.225122;                               //!< [rad] Earth-limb cone half-angle used by EXPERIMENT +X target

    void refreshThresholds();
    void writeIdentityReference(uint64_t CurrentSimNanos);
    bool selectVisibleGroundStation(const double scPos_N[3],
                                    uint64_t CurrentSimNanos,
                                    std::string* stationLabel,
                                    double stationLosHat_N[3]) const;
};

#endif
