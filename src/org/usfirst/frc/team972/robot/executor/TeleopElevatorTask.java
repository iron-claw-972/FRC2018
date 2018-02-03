package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopElevatorTask extends Task {

	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	ControlElevatorTask elevatorControl;
	
	double easingValue = 0.25;
	final int elevatorAxisJoystick = 2;
	
	final double deadbandValue = 0.05;
	
	final double[] ELEVATOR_POSITIONS = {0.0, 4.0, 6.0};
	final int[] ELEVATOR_BUTTONS = {1, 2, 3};
	
	double output = 0;
	
	public TeleopElevatorTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors, ControlElevatorTask _elevatorControl) {
		super(_executionTime);
		mechanismMotors = _mechanismMotors;
		uig = _uig;
		elevatorControl = _elevatorControl;
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
		
		for(int i=0; i<ELEVATOR_BUTTONS.length; i++) {
			if(joystickB.getRawButtonPressed(ELEVATOR_BUTTONS[i])) {
				elevatorControl.setElevatorPositionTarget((float)ELEVATOR_POSITIONS[i]);
			}
		}
		
	
	}
	
}