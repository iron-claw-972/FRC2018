package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlIntakeArmTask;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopElevatorTask extends Task {

	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	ControlElevatorTask elevatorControl;
	ControlIntakeArmTask intakeControl;
	
	boolean flopDown = false;
	
	double easingValue = 0.25;
	final int elevatorAxisJoystick = 3;
	final int elevatorOverrideButton = 6;
	final int flopButton = 3;
	
	final double deadbandValue = 0.05;
	
	final double[] ELEVATOR_POSITIONS = {0.0, 0.09, 0.5, 1.27};
	final int[] ELEVATOR_BUTTONS = {180, 270, 0, 90};
	//public static final double POINT_OF_BAR_HIT = 0.75;
	
	boolean runUpOut = false;
	boolean runUpIn = false;
	
	double output = 0;
	double currentTarget = 0;
	
	TeleopIntakeArmTask intakeArm;
	
	public TeleopElevatorTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors, ControlElevatorTask _elevatorControl, ControlIntakeArmTask _intakeControl) {
		super(_executionTime);
		mechanismMotors = _mechanismMotors;
		uig = _uig;
		elevatorControl = _elevatorControl;
		intakeControl = _intakeControl;
	}
	
	public void GiveIntakeArm(TeleopIntakeArmTask _intakeArm) {
		intakeArm = _intakeArm;
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
		elevatorControl.setElevatorPositionTarget(Math.abs(elevatorControl.getPosition()));
		//elevatorControl.setElevatorPositionTargetOnceCycle(dt);
	}

	private void automaticUpAndOut() {
		if(runUpOut && (Math.abs(elevatorControl.getPosition()) < 0.1)) {
			if(intakeArm.isGoodMovePositionReal(elevatorControl.getPosition())) {
				intakeArm.clampModeToggle = false;
				intakeArm.currentMode = 'C';
				RobotLogger.toast("Finish Up and Out");
				stopAllRequestMove();
			} else {
				elevatorControl.setElevatorPositionTarget(0.08);
			}
		} else {
			stopAllRequestMove();
		}
	}
	
	private int upAndInStep = 0;
	private void automaticUpAndIn() {
		if(runUpIn && (intakeControl.isSafeOutwards() || (upAndInStep != 1))) {
			//add 0.025 to think we are a tiny bit high to have MoE
			if(intakeControl.isSafeOutwards() && intakeArm.isGoodMovePositionReal(elevatorControl.getPosition() + 0.025)) {
				intakeArm.currentMode = 'B';
				upAndInStep = 2;
			} else if(intakeControl.isSafeOutwards() == false) { 
				elevatorControl.setElevatorPositionTarget(0);
				upAndInStep = 3;
			} else if((Math.abs(elevatorControl.getPosition()) < 0.02) && (upAndInStep > 1)) {
				RobotLogger.toast("Finish Up and In");
				stopAllRequestMove();
			} else {
				elevatorControl.setElevatorPositionTarget(0.05);
				upAndInStep = 1;
			}
		} else {
			stopAllRequestMove();
		}
	}
	
	public void requestUpOut() {
		RobotLogger.toast("Up and Out Requested!");
		runUpOut = true;
	}
	
	public void requestUpAndIn() {
		RobotLogger.toast("Up and In Requested!");
		runUpIn = true;
		upAndInStep = 0;
	}
	
	private void stopAllRequestMove() {
		runUpOut = false;
		runUpIn = false;
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
			runUpOut = false;
		} else if(joystickB.getRawButtonReleased(elevatorOverrideButton)) {
			elevatorControl.setElevatorPositionTargetOnceCycle(dt); //make motion profile think we are there
			elevatorControl.setControl(true); //TODO: SWITCH FR REAL
		} else {
			output = interpolateValues(0, output);
			elevatorControl.setControl(true); //TODO: should we be true for real
		}
		
		for(int i=0; i<ELEVATOR_BUTTONS.length; i++) {
			if(joystickB.getPOV() == ELEVATOR_BUTTONS[i]) {
				stopAllRequestMove();
				if(intakeControl.isSafeOutwards()) {
					RobotLogger.toast("Setting Elevator Position to: " + ELEVATOR_POSITIONS[i]);
					elevatorControl.setElevatorPositionTarget(ELEVATOR_POSITIONS[i]);
					currentTarget = ELEVATOR_POSITIONS[i];
				} else {
					if((i == 1) || (i == 0)) {
						RobotLogger.toast("Allow Elevator because only go up little");
						elevatorControl.setElevatorPositionTarget(ELEVATOR_POSITIONS[i]);
						currentTarget = ELEVATOR_POSITIONS[i];
					} else {
						RobotLogger.toast("Deny Elevator Movement, Intake Arms not safe! : " + intakeControl.getLeftPos() + " " + intakeControl.getRightPos());
					}
				}
				break;
			}
		}
		
		automaticUpAndOut();
		automaticUpAndIn();
	}
	
	public void setEnable() {
		
	}
	
}