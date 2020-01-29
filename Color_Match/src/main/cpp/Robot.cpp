/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "frc/Joystick.h"
//#include "frc/buttons/JoystickButton.h"

using namespace frc;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect various colors.
 */
class Robot : public frc::TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};

  WPI_TalonSRX * _rghtFront = new WPI_TalonSRX(3);
  //std::shared_ptr<frc::Joystick> operatorJoystick;
  Joystick * _operator = new Joystick(0);
  

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  rev::ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

 public:
  void RobotInit() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
  }
  void RobotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    frc::Color detectedColor = m_colorSensor.GetColor();
    float mSpeed = 0.;
    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    boolean buttonValue;


    buttonValue = _operator.getTrigger()

    if (matchedColor == kBlueTarget && buttonValue) {
      colorString = "Blue";
      mSpeed = 0.2;
    } else if (matchedColor == kRedTarget && buttonValue) {
      colorString = "Red";
      mSpeed = 0.2;
    } else if (matchedColor == kGreenTarget && buttonValue) {
      colorString = "Green";
      mSpeed = 0.2;
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
      mSpeed = 0.;
      if(buttonValue) {
        mSpeed = 0.2
      }
    } else {
      colorString = "Unknown";
      mSpeed = 0.;
      if(buttonValue) {
        mSpeed = 0.2
      }
    }
    
    _rghtFront->Set(mSpeed);

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);

  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif