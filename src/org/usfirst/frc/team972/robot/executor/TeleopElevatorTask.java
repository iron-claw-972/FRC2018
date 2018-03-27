package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlFlopTask;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopElevatorTask extends Task {

	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	ControlElevatorTask elevatorControl;
	
	boolean flopDown = false;
	
	double easingValue = 0.25;
	final int elevatorAxisJoystick = 3;
	final int elevatorOverrideButton = 6;
	final int flopButton = 3;
	
	final double deadbandValue = 0.05;
	
	final double[] ELEVATOR_POSITIONS = {0.0, 0.2, 0.5, 1.0};
	final int[] ELEVATOR_BUTTONS = {180, 270, 0, 90};
	//public static final double POINT_OF_BAR_HIT = 0.75;
	
	double output = 0;
	double currentTarget = 0;
	
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
		elevatorControl.setElevatorPositionTarget((float)ELEVATOR_POSITIONS[0]);
	}

	@Override
	public void execute(double dt) {
		Joystick joystickB = uig.getStickB();
		double wantedOutput = joystickB.getRawAxis(elevatorAxisJoystick);
		wantedOutput = handleDeadband(wantedOutput, deadbandValue);
		
		if(joystickB.getRawButton(elevatorOverrideButton)) {
			output = interpolateValues(wantedOutput, output);
			elevatorControl.setControl(false);
			mechanismMotors.RunElevatorLiftMotor(output);
			RobotLogger.toast("man: "+ output);
		} else if(joystickB.getRawButtonReleased(elevatorOverrideButton)) {
			elevatorControl.setElevatorPositionTargetOnceCycle(dt); //make motion profile think we are there
			elevatorControl.setControl(true); //TODO: SWITCH FR REAL
		} else {
			output = interpolateValues(0, output);
			elevatorControl.setControl(true); //TODO: should we be true for real
		}
	
		/*
		if(Math.abs(joystickB.getRawAxis(0)) > 0.05) {
			flopControl.setControl(false);
			mechanismMotors.RunFlopMotor(joystickB.getRawAxis(0));
		} else if(joystickB.getRawButtonPressed(flopButton)) {
			flopControl.setFlopPositionTarget(flopControl.getFlopCurrentPos());
			flopControl.setControl(true);
			if(flopDown) {
				flopControl.setFlopPositionTarget(flopControl.STARTING_REV * 0.9);
				flopDown = false;
			} else {
				flopControl.setFlopPositionTarget(flopControl.DOWN_MINIMUM);
				flopDown = true;
			}
		} else {
			mechanismMotors.RunFlopMotor(0);
		}
		*/
		
		for(int i=0; i<ELEVATOR_BUTTONS.length; i++) {
			if(joystickB.getPOV() == ELEVATOR_BUTTONS[i]) {
				RobotLogger.toast("Setting Elevator Position to: " + ELEVATOR_POSITIONS[i]);
				elevatorControl.setElevatorPositionTarget(ELEVATOR_POSITIONS[i]);
				currentTarget = ELEVATOR_POSITIONS[i];
				break;
			}
		}
	}
	
	public void setEnable() {
		
	}
	
}