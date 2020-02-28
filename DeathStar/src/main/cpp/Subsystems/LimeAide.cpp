

#include "Subsystems/LimeAide.h"
#include "ntcore.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "SensorFilter.h"

LimeAide::LimeAide() : frc::Subsystem("LimeAide") { 

}

void LimeAide::InitDefaultCommand() {}
void LimeAide::Periodic() {

  LimeRoxTrack();

        frc::SmartDashboard::PutNumber("targetX",getLimeRoxX() );
        frc::SmartDashboard::PutNumber("targetY",getLimeRoxY() );
        frc::SmartDashboard::PutNumber("targetA",getLimeRoxA() );
        frc::SmartDashboard::PutNumber("targetSkew",getLimeRoxS() );

        frc::SmartDashboard::PutBoolean("IseeTrgt", getLimeRoxInView());
}

void LimeAide::LimeRoxTrack()
{
  // filter moving average, use 5 values for now
  static SensorFilter txFilter(5);
  static SensorFilter tyFilter(5);
  static SensorFilter taFilter(5);
  
std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
double tx = 0.;
double ty = 0.;
double ta = 0.;
double tv = 0.;
  //if(table != NULL ) {
  // x offset deg
     tx = table->GetNumber("tx",0.0);
  // y offset deg
     ty = table->GetNumber("ty",0.0);
  // target size
     ta = table->GetNumber("ta",0.0);
  // target(s) available

     tv = table->GetNumber("tv",0.0);
  //}

  txFilter.AddValue(tx);
  tyFilter.AddValue(ty);
  taFilter.AddValue(ta);
  
  m_targetsAvailable = tv;
  m_targetX          = txFilter.GetAverage();
  m_targetY          = tyFilter.GetAverage();
  m_targetArea       = taFilter.GetAverage();

}
bool LimeAide::getLimeRoxInView(){
	bool InView;
	if(m_targetsAvailable < 1.0) {
		InView =  false;
	} else{
		InView =  true;
	}
	return InView;
}
	
double LimeAide::getLimeRoxX(){

	return m_targetX;
}

double LimeAide::getLimeRoxY(){

	return m_targetY;
}

double LimeAide::getLimeRoxA(){
    return m_targetArea;
}

double LimeAide::getLimeRoxS(){
    return m_targetSkew;
}

void LimeAide::LimeRoxLEDOn(){

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  table->PutNumber("ledMode",0);
  //table->PutNumber("ledMode",3);

}

void LimeAide::LimeRoxLEDOff(){

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  table->PutNumber("ledMode",1);

}

void LimeAide::LimeRoxLEDBlnk(){

  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  table->PutNumber("ledMode",2);
}

void LimeAide::setLimeRoxPipe0(){
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("pipeline",0);
}

// functions to use with tables of targeting information

void LimeAide::LimeSpeeds(float xIn){
  float *x; 
  x = &m_trgtAreas[0];
  float *y; 
  y = &m_shooterSetPoints[0];
  int nx = m_nX;
  float spd = SPAMutil::SPAMlerp(x , y  , nx, xIn);
   m_limeSpeedSetpoint = (double)spd;
}
 
void LimeAide::LimeAngles(float xIn){
  float *x; 
  x = &m_trgtAreas[0];
  float *y; 
  y = &m_domeAngles[0];
  int nx = m_nX;
  float ang = SPAMutil::SPAMlerp(x , y  , nx, xIn);
   m_limeAngleSetpoint = (double)ang;
}

  double LimeAide::getLimeSpdStPt(){
   return m_limeSpeedSetpoint;
  }

	double LimeAide::getLimeDomeStPt(){
   return m_limeAngleSetpoint;
  }



    // SmartDashboard Buttons
    //frc::SmartDashboard::PutData("GrabHatch", new GrabHatch());






