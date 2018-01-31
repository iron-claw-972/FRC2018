package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopWinchTask extends Task {
	
	UserInputGamepad uig;
	MechanismActuators mechanismMotors;
	
	final int winchAxisJoystick = 1;
	final double deadbandValue = 0.05;
	
	public TeleopWinchTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors) {
		super(_executionTime);
		uig = _uig;
		mechanismMotors = _mechanismMotors;
	}

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
	
	@Override
	public void init(double dt) {
	}

	@Override
	public void execute(double dt) {
		Joystick joystickB = uig.getStickB();
		double output = joystickB.getRawAxis(winchAxisJoystick);
		output = handleDeadband(output, deadbandValue);
		mechanismMotors.RunWinchMotor(output);
	}
}