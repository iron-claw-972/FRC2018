package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopElevatorTask extends Task {

	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	
	double easingValue = 0.25;
	final int elevatorAxisJoystick = 2;
	
	final double deadbandValue = 0.05;
	
	double output = 0;
	
	public TeleopElevatorTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors) {
		super(_executionTime);
		mechanismMotors = _mechanismMotors;
		uig = _uig;
	}

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
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
		Joystick joystickB = uig.getStickB();
		double wantedOutput = joystickB.getRawAxis(elevatorAxisJoystick);
		wantedOutput = handleDeadband(wantedOutput, deadbandValue);
		output = interpolateValues(wantedOutput, output);
		mechanismMotors.RunElevatorLiftMotor(output);
	}
	
}
