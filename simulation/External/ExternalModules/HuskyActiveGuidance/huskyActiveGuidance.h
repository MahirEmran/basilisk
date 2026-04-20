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

#ifndef HUSKY_ACTIVE_GUIDANCE_H
#define HUSKY_ACTIVE_GUIDANCE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

#include <string>
#include <vector>

class HuskyActiveGuidance : public SysModel {
public:
    HuskyActiveGuidance();
    ~HuskyActiveGuidance() override;

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

    void setPosFound_B(double x_m, double y_m, double z_m);
    void setDefaultSunHat_N(double x, double y, double z);

    std::string getState() const;

private:
    enum GuidanceMode {
        MODE_ROLL_ONLY = 0,
        MODE_EXPERIMENT = 1,
        MODE_HYBRID = 2
    };

    struct RollSolveResult {
        double rollDeg;
        double y_B[3];
        double z_B[3];
        double scoreDeg;
    };

    GuidanceMode mode = MODE_HYBRID;                         //!< guidance mode
    std::string modeString = "HYBRID";                        //!< mode name for logging
    std::string state = "UNINITIALIZED";                      //!< active state string

    double posFound_B[3] = {0.05, 0.0, 0.105};              //!< [m] FOUND camera position in body frame
    double defaultSunHat_N[3] = {1.0, 0.0, 0.0};            //!< [-] fallback Sun unit vector in inertial frame

    double lostExclHalfDeg = 17.5;                          //!< [deg] LOST keep-out half angle
    double chargeEnterClearDeg = 19.5;                      //!< [deg] CHARGING entry clearance threshold
    double chargeExitClearDeg = 16.5;                       //!< [deg] CHARGING exit clearance threshold
    double earthHalfAngleDeg = 70.210073;                   //!< [deg] Earth apparent half-angle at 400 km
    double chargeEnterSunVisDeg = 72.210073;                //!< [deg] CHARGING entry Sun visibility threshold
    double chargeExitSunVisDeg = 69.210073;                 //!< [deg] CHARGING exit Sun visibility threshold

    uint64_t statusPeriodNanos = 60000000000ULL;            //!< [ns] status print period
    uint64_t lastStatusPrintNanos = 0U;                     //!< [ns] last status print time
    bool hasPrintedStatus = false;

    double prevRollDeg = 0.0;                               //!< [deg] previous chosen roll
    bool hasPrevRoll = false;

    double rhoRad = 1.225122;                               //!< [rad] Earth limb cone half-angle at 400 km

    void refreshThresholds();
    void writeIdentityReference(uint64_t CurrentSimNanos);

    static bool safeUnit(const double vec[3], double out[3]);
    static double angleDegBetween(const double vecA[3], const double vecB[3]);
    static void cross3(const double vecA[3], const double vecB[3], double out[3]);
    static double dot3(const double vecA[3], const double vecB[3]);
    static double norm3(const double vec[3]);

    static void computeCompromiseX(const double earthHat[3],
                                   const double sunHat[3],
                                   double rhoRad,
                                   double outX_B[3]);

    static void buildYzFrameAboutX(const double x_B[3],
                                   const double earthHat[3],
                                   double outY_B[3],
                                   double outZ_B[3]);

    RollSolveResult solveRollForLostClearance(const double x_B[3],
                                              const double earthHat[3],
                                              const double sunHat[3],
                                              const std::vector<const double*>& extraKeepoutHats);
};

#endif
