
#include "Subsystems/WarpDriveInverter.h"

WarpDriveInverter::WarpDriveInverter() : frc::Subsystem("WarpDriveInverter") {

    wheelMotor.reset(new rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless));
    wheelEncoder.reset(new rev::CANEncoder(*wheelMotor));
    colorSensor.reset(new rev::ColorSensorV3(frc::I2C::Port::kOnboard));
    colorMatcher.reset(new rev::ColorMatch());
}

void WarpDriveInverter::ColorMatcherInit()
{
    colorMatcher->AddColorMatch(kBlueTarget);
    colorMatcher->AddColorMatch(kGreenTarget);
    colorMatcher->AddColorMatch(kRedTarget);
    colorMatcher->AddColorMatch(kYellowTarget);
}

  
void WarpDriveInverter::MoveMotor(double speed){
  wheelMotor->Set(speed); //set at a fixed power for now, will eventually be on a PID velocity
}

void WarpDriveInverter::Halt(){
  wheelMotor->StopMotor();
}

/*
void WarpDriveInverter::ReverseReverse(){
  wheelMotor->Set(-1.0); //set at a fixed power for now, will eventually be on a PID velocity
}
*/

int WarpDriveInverter::getDetectedColor(){
  return currentColor;
}
int WarpDriveInverter::getRequestedColor(){
  return requestedColor;
} 

uint32_t WarpDriveInverter::getProximity(){
  return colorSensor->GetProximity();
}

double WarpDriveInverter::getPosition(){
  return wheelEncoder->GetPosition();
}

bool WarpDriveInverter::inRange(){
  return colorSensor->GetProximity() > MAX_DISTANCE;
}

void WarpDriveInverter::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
}

void WarpDriveInverter::Periodic() {
    frc::Color detectedColor = colorSensor->GetColor();
    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    double confidence = -1.0;
    frc::Color matchedColor = colorMatcher->MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
      currentColor = BLUE;
    } 
    else if (matchedColor == kRedTarget) {
      colorString = "Red";
      currentColor = RED;
    } 
    else if (matchedColor == kGreenTarget) {
      colorString = "Green";
      currentColor = GREEN;
    } 
    else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
      currentColor = YELLOW;
    } 
    else {
      colorString = "Unknown";
      currentColor = ERROR;
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
    frc::SmartDashboard::PutNumber("Proximity", colorSensor->GetProximity());
    frc::SmartDashboard::PutBoolean("CPanel In Range", inRange());
    
    std::string gameData;
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData[0])
      {
        case 'B' :
          frc::SmartDashboard::PutString("Game Data Color", "BLUE");
          requestedColor = BLUE;
          frc::SmartDashboard::PutBoolean("Got color FMS", true);
          break;
        case 'G' :
          frc::SmartDashboard::PutString("Game Data Color", "GREEN");
          requestedColor = GREEN;
          frc::SmartDashboard::PutBoolean("Got color FMS", true);
          break;
        case 'R' :
          frc::SmartDashboard::PutString("Game Data Color", "RED");
          requestedColor = RED;
          frc::SmartDashboard::PutBoolean("Got color FMS", true);
          break;
        case 'Y' :
          frc::SmartDashboard::PutString("Game Data Color", "YELLOW");
          requestedColor = YELLOW;
          frc::SmartDashboard::PutBoolean("Got color FMS", true);
          break;
        default :
          frc::SmartDashboard::PutString("Game Data Color", "ERROR");
          requestedColor = ERROR;
          frc::SmartDashboard::PutBoolean("Got color FMS", false);
          break;
      }
    } 
    else {
      frc::SmartDashboard::PutString("Game Data Color", "NO INFO");
      requestedColor = NO_DATA;
      frc::SmartDashboard::PutBoolean("Got color FMS", false);
    }
}
