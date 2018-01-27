package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

public class TeleopElevatorTask extends Task {

	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	
	double easingValue = 0.25;
	
	public TeleopElevatorTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors) {
		super(_executionTime);
		mechanismMotors = _mechanismMotors;
		uig = _uig;
	}

	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}
	
	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void execute(double dt) {
		
	}
	
}
