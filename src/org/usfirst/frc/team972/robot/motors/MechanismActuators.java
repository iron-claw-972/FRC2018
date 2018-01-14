package org.usfirst.frc.team972.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class MechanismActuators {
	
	WPI_TalonSRX intakeMotorLeft;
	WPI_TalonSRX intakeMotorRight;
	
	public void SetupIntakeMotors(int left, int right) {
		intakeMotorLeft = new WPI_TalonSRX(left);
		intakeMotorRight = new WPI_TalonSRX(right);
		
		intakeMotorLeft.set(ControlMode.PercentOutput, 0);
		intakeMotorRight.set(ControlMode.PercentOutput, 0);
	}
	
	public void RunIntakeMotors(double power) {
		intakeMotorLeft.set(power);
		intakeMotorRight.set(-power);
	}
}