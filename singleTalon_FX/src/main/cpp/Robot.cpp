/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */
#include <iostream>
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonSRX * _rghtFront = new WPI_TalonSRX(3);


	Joystick * _joystick = new Joystick(0);

	Faults _faults_L;
	Faults _faults_R;

	void TeleopPeriodic() {

		/* get gamepad stick values */
		double forw = -1 * _joystick->GetRawAxis(1); /* positive is forward */
		double turn = +1 * _joystick->GetRawAxis(2); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

        _rghtFront->Set(forw);
		/* drive robot */

	}

	void RobotInit() {
		/* factory default values */
		_rghtFront->ConfigFactoryDefault();

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		_rghtFront->SetInverted(false);

		/* [4] adjust sensor phase so sensor moves
		 * positive when Talon LEDs are green */
		_rghtFront->SetSensorPhase(true);


	}

private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif