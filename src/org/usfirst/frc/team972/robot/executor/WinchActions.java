package org.usfirst.frc.team972.robot.executor;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class WinchActions {
	
	WPI_TalonSRX winch;
	
	public void setWinchMotors(int winchMotorValue) {
		winch = new WPI_TalonSRX(winchMotorValue);
	}

	public void controlWinch(double power) {
		if(power > 0.3) {
			power = 0.1;
			winch.set(power);
		} else if (power < -0.3) {
			power = -0.1;
			winch.set(power);
		} else {
			winch.set(0);
		}
	}
}