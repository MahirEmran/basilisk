#ifndef ACTIVE_GUIDANCE_MATH_H
#define ACTIVE_GUIDANCE_MATH_H

#include <vector>

namespace ActiveGuidanceMath {

struct RollSolveResult {
    double rollDeg;  //!< [deg] selected roll angle about +X
    double y_B[3];   //!< [-] body +Y axis in inertial coordinates
    double z_B[3];   //!< [-] body +Z axis in inertial coordinates (LOST boresight)
    double scoreDeg; //!< [deg] max-min LOST clearance score for selected geometry
};

double earthLimbHalfAngleRad(double earthRadiusKm, double orbitAltitudeKm);

bool safeUnit(const double vec[3], double out[3]);
double angleDegBetween(const double vecA[3], const double vecB[3]);

void computeCompromiseX(const double earthHat[3],
                        const double sunHat[3],
                        double rhoRad,
                        double outX_B[3]);

RollSolveResult solveRollForLostClearance(const double x_B[3],
                                          const double earthHat[3],
                                          const double sunHat[3],
                                          const std::vector<const double*>& extraKeepoutHats,
                                          bool hasPrevRoll,
                                          double prevRollDeg);

}  // namespace ActiveGuidanceMath

#endif
