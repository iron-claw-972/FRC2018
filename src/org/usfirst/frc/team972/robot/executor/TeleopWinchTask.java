package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopWinchTask extends Task {
	
	UserInputGamepad uig;
	MechanismActuators mechanismMotors;
	
	final int winchAxisJoystick = 1;
	final int winchButton = 6; 
	final double deadbandValue = 0.05;
	double output = 0;
	
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
		
		if(joystickB.getRawButton(winchButton) ) {
			output = joystickB.getRawAxis(winchAxisJoystick);
			output = handleDeadband(output, deadbandValue);
		} else {
			output = 0;
		}
		
		mechanismMotors.RunWinchMotor(output);
	}
}