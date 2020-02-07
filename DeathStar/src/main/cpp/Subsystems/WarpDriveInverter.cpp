// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/WarpDriveInverter.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

WarpDriveInverter::WarpDriveInverter() : frc::Subsystem("WarpDriveInverter") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
wheelMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(5));
colorSensor.reset(new rev::ColorSensorV3(frc::I2C::Port::kOnboard));
colorMatcher.reset(new rev::ColorMatch());


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void WarpDriveInverter::ColorMatcherInit()
{
    colorMatcher -> AddColorMatch(kBlueTarget);
    colorMatcher -> AddColorMatch(kGreenTarget);
    colorMatcher -> AddColorMatch(kRedTarget);
    colorMatcher -> AddColorMatch(kYellowTarget);
}

void WarpDriveInverter::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void WarpDriveInverter::Periodic() {
    frc::Color detectedColor = colorSensor -> GetColor();
    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = colorMatcher -> MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
    } 
    else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } 
    else if (matchedColor == kGreenTarget) {
      colorString = "Green";
    } 
    else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    } 
    else {
      colorString = "Unknown";
    }
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
