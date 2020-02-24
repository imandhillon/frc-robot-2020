
#include "BangBangController.h"
#include <algorithm>


BangBangController::BangBangController()
{
}

BangBangController::BangBangController(double lowLimit, double highLimit)
{
    m_bangLow = lowLimit;
    m_bangHigh = highLimit;
}

BangBangController::~BangBangController()
{
}

void BangBangController::SetSource(frc::PIDSource * source, BBSourceType type)
{
    m_CANencoder = NULL;
    m_pidsource = source;
}

void BangBangController::SetSource(rev::CANEncoder * source, BBSourceType type)
{
    m_pidsource = NULL;
    m_CANencoder = source;
}

/**
 * Set the BangBang limits
 */
void BangBangController::SetBBLimits(double lowlimit, double highlimit)
{
    m_bangLow = lowlimit;
    m_bangHigh = highlimit;
}

/**
 * Sets the setpoint for the PIDController.
 *
 * @param setpoint The desired setpoint.
 */
void BangBangController::SetSetpoint(double setpoint)
{
    if (m_maxinput > m_mininput) {
      m_setpoint = std::clamp(setpoint, m_mininput, m_maxinput);
    } else {
      m_setpoint = setpoint;
    }
}

/**
 * Returns the current setpoint of the Controller.
 *
 * @return The current setpoint.
 */
double BangBangController::GetSetpoint() const
{
    return m_setpoint;
}

/**
 * Returns true if the error is within the tolerance of the error.
 *
 * This will return false until at least one input value has been computed.
 */
bool BangBangController::AtSetpoint() const
{
    return std::abs(m_positionError) < m_positionTolerance &&
           std::abs(m_velocityError) < m_velocityTolerance;

}

/**
 * Sets the error which is considered tolerable for use with AtSetpoint().
 *
 * @param positionTolerance Position error which is tolerable.
 * @param velociytTolerance Velocity error which is tolerable.
 */
void BangBangController::SetTolerance(double positionTolerance, double velocityTolerance)
{
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
}
    
/**
 * Returns the difference between the setpoint and the measurement.
 */
double BangBangController::GetPositionTolerance() const
{
    return m_positionError;
}

/**
 * Returns the velocity error.
 */
double BangBangController::GetVelocityError() const
{
    return m_velocityError;
}

/**
 * Returns the next output of the controller.
 *
 * This method uses the supplied source & type
 */
double BangBangController::Calculate()
{
    if (m_pidsource != NULL) {
        return Calculate(m_pidsource->PIDGet());
    }
    else if (m_CANencoder != NULL) {
        double measurement = (m_sourcetype == kBBSourceRate) ? m_CANencoder->GetVelocity() : m_CANencoder->GetPosition();
        return Calculate(measurement);
    }
    return 0.0;
}

/**
 * Returns the next output of the controller.
 *
 * @param measurement The current measurement of the process variable.
 */
double BangBangController::Calculate(double measurement)
{
    return Calculate(measurement, m_setpoint);
}

/**
 * Returns the next output of the PID controller.
 *
 * @param measurement The current measurement of the process variable (i.e. Encoder).
 * @param setpoint The new setpoint of the controller.
 */
double BangBangController::Calculate(double measurement, double setpoint)
{
    double err = setpoint - measurement;
    double output = 0;

    if (err < m_bangLow) {
        output = m_speed;
    }
    else if (err > m_bangHigh) {
        output = -m_speed;
    }
    return output;
}
