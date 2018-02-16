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
	ControlFlopTask flopControl;
	
	double easingValue = 0.25;
	final int elevatorAxisJoystick = 3;
	final int elevatorOverrideButton = 6;
	
	final double deadbandValue = 0.05;
	
	final double[] ELEVATOR_POSITIONS = {0, 1 * 12 * 0.0254, 5 * 12 * 0.0254, 7 * 12 * 0.0254};
	final int[] ELEVATOR_BUTTONS = {180, 270, 0, 90};
	
	double output = 0;
	
	public TeleopElevatorTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors, ControlElevatorTask _elevatorControl, ControlFlopTask _flopControl) {
		super(_executionTime);
		mechanismMotors = _mechanismMotors;
		uig = _uig;
		elevatorControl = _elevatorControl;
		flopControl = _flopControl;
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
		
		if(joystickB.getRawButton(3)) {
			RobotLogger.toast("manual elevator");
			flopControl.setElevatorPositionTarget(0.1);
		}
		
		for(int i=0; i<ELEVATOR_BUTTONS.length; i++) {
			if(joystickB.getPOV() == ELEVATOR_BUTTONS[i]) {
				RobotLogger.toast("Setting Elevator Position to: " + ELEVATOR_POSITIONS[i]);
				elevatorControl.setElevatorPositionTarget((float)ELEVATOR_POSITIONS[i]);
				break;
			}
		}
		
		
	
	}
	
	public void setEnable() {
		
	}
	
}