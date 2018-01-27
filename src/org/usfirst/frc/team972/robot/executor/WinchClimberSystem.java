package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.*;

public class WinchClimberSystem extends Task{
	
	UserInputGamepad uig;
	final double power = 0.3;
	WinchActions winchMovement;
	double winchSpeed;
	
	public WinchClimberSystem(double _executionTime, UserInputGamepad _uig, int winchMotorValue) {
		super(_executionTime);
		uig = _uig;
		winchMovement = new WinchActions();
		winchMovement.setWinchMotors(winchMotorValue);
	}

	@Override
	public void init(double dt) {	
	}

	@Override
	public void execute(double dt) {
		Joystick winchJoystick = uig.getStickA();	
		winchSpeed = winchJoystick.getRawAxis(3);
		if(winchSpeed != 0) {
			winchMovement.controlWinch(winchSpeed);
		}
	}
}
