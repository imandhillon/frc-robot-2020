
#pragma once

#include "frc/PIDSource.h"
#include "rev/CANEncoder.h"
#include <limits>
#include <units/units.h>

enum BBSourceType { kBBSourceRate = 0, kBBSourcePosition = 1 };

class BangBangController {
public:
    /**
     * The BangBangController will apply lower/upper bounds power and attempt to keep
     * requested setpoint.  When the position/rate is below the lower limit, the "lowSpeed"
     * power is applied (typically high power, i.e. 0.95).  When the position/rate exceeds the
     * high bounds, then the "highSpeed" power is applied (typically a low or negative value).
     */
    BangBangController(double bbLowSpeed, double bbHighSpeed);
    BangBangController(double bbLowSpeed, double bbHighSpeed, double lowLimit, double highLimit);
    ~BangBangController();

    //BangBangController& operator=(const BangBangController&) = default;
    void SetSource(frc::PIDSource * source, BBSourceType type);
    void SetSource(rev::CANEncoder * source, BBSourceType type);

    /**
     * Set the BangBang output speed (+/-)
     */
    void SetBBSpeeds(double bbLowSpeed, double bbHighSpeed);
    double GetBBLowSpeed() const { return m_lowSpeed; }
    double GetBBHighSpeed() const { return m_highSpeed; }

    /**
     * Set the BangBang error limits
     */
    void SetBBLimits(double lowlimit, double highlimit);

    /**
     * Sets the setpoint for the Controller.
     *
     * @param setpoint The desired setpoint.
     */
    void SetSetpoint(double setpoint);

    /**
     * Returns the current setpoint of the Controller.
     *
     * @return The current setpoint.
     */
    double GetSetpoint() const;

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * This will return false until at least one input value has been computed.
     */
    bool AtSetpoint() const;

    /**
     * Sets the error which is considered tolerable for use with AtSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velociytTolerance Velocity error which is tolerable.
     */
    void SetTolerance(double positionTolerance, double velocityTolerance = std::numeric_limits<double>::infinity());
    
    /**
     * Returns the difference between the setpoint and the measurement.
     */
    double GetPositionTolerance() const;

    /**
     * Returns the velocity error.
     */
    double GetVelocityError() const;

    /**
     * Returns the next output of the controller.
     *
     * This method uses the supplied source & type
     */
    double Calculate();

    /**
     * Returns the next output of the controller.
     *
     * @param measurement The current measurement of the process variable (i.e. Encoder).
     */
    double Calculate(double measurement);

    /**
     * Returns the next output of the controller.
     *
     * @param measurement The current measurement of the process variable (i.e. Encoder).
     * @param setpoint The new setpoint of the controller.
     */
    double Calculate(double measurement, double setpoint);

private:

    double m_setpoint = 0;
    double m_bangLow = -100.0;
    double m_bangHigh = 100.0;
    double m_lowSpeed = 0;
    double m_highSpeed = 0;

    double m_maxinput = 0.0;
    double m_mininput = 0.0;


    // Source (for now, just pass the encoder value into Calculate)
    frc::PIDSource* m_pidsource = NULL;
    rev::CANEncoder* m_CANencoder = NULL;
    BBSourceType m_sourcetype = kBBSourceRate;

    // The error at the time of the most recent call to Calculate()
    double m_positionError = 0;
    double m_velocityError = 0;

    // The error at the time of the second-most-recent call to Calculate() (used
    // to compute velocity)
    double m_prevError = 0;

    // The sum of the errors for use in the integral calc
    double m_totalError = 0;

    // The error that is considered at setpoint.
    double m_positionTolerance = 0.05;
    double m_velocityTolerance = std::numeric_limits<double>::infinity();

};