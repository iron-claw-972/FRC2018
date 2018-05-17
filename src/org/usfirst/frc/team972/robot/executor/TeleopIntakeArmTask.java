package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlIntakeArmTask;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopIntakeArmTask extends Task {

	UserInputGamepad uig;
	
	ControlIntakeArmTask armControl;
	ControlElevatorTask elevatorControl;
	MechanismActuators motors;
	
	boolean clampModeToggle = false;
	
	private final double MIN_HEIGHT_MOVE = 0.03;
	private final double MAX_HEIGHT_MOVE = 0.1;
	private final double CLAMP_REV = 0.925;
	private final double OPEN_REV = 0.8;
	private final double BACK_REV = 0;
	
	char currentMode = 'B';
	
	Sensors sensors;
	
	TeleopIntakeTask intakeTask;
	TeleopElevatorTask elevatorTask;
	
	public TeleopIntakeArmTask(double _executionTime, UserInputGamepad _uig, ControlIntakeArmTask _armControl, Sensors _sensors, MechanismActuators _motors, ControlElevatorTask _elevatorControl, TeleopIntakeTask _intakeTask, TeleopElevatorTask _elevatorTask) {
		super(_executionTime);
		uig = _uig;
		armControl = _armControl;
		sensors = _sensors;
		motors = _motors;
		elevatorControl = _elevatorControl;
		intakeTask = _intakeTask;
		elevatorTask = _elevatorTask;
	}

	@Override
	public void init(double dt) {
		currentMode = 'C';
	}
	
	public boolean isGoodMovePositionReal(double pos) {
		return (Math.abs(pos) > MIN_HEIGHT_MOVE) && (Math.abs(pos) < MAX_HEIGHT_MOVE);
	}

	@Override
	public void execute(double dt) {
		Joystick joystick = uig.getStickB();
		
		double twistOffset = 0;
		if((Math.abs(elevatorControl.getPosition()) < 0.2) && joystick.getRawButton(1)) {
			twistOffset = joystick.getRawAxis(3) * 0.2;
		}

		boolean moveBackOverride = joystick.getRawButton(5);
		boolean openArm = joystick.getRawButton(4);
		boolean clampArm = joystick.getRawButton(3);

		if(joystick.getRawButtonPressed(1)) {
			currentMode = 'C';
			clampModeToggle = false;
		} else if(joystick.getRawButtonReleased(1)) {
			currentMode = 'C';
			clampModeToggle = true;
		} else if(joystick.getRawButton(2)) {
			clampModeToggle = false;
		}
		
		if(openArm) {
			clampModeToggle = true;
		} else if(clampArm) {
			clampModeToggle = false;
		}
		
		if(moveBackOverride) {
			currentMode = 'B';
			clampModeToggle = true;
		} else if(clampArm || openArm) {
			currentMode = 'C';
		}

		if(currentMode == 'B') {
			if(isGoodMovePositionReal(elevatorControl.getPosition()) == false) {
				RobotLogger.toast("Arm collision backwards, prevent move!!!");
				currentMode = 'C';
				//elevatorTask.requestUpAndIn();
			} else {
				armControl.allowedControl = true;
				armControl.setPositionTarget(BACK_REV, BACK_REV);
			}
		} else if(currentMode == 'C') {
			if((Math.abs(elevatorControl.getPosition()) > MIN_HEIGHT_MOVE) || (armControl.isSafeOutwards())) {
				armControl.allowedControl = true;
				if(clampModeToggle) {
					armControl.setGainP(25);
					armControl.setPositionTarget(CLAMP_REV, -CLAMP_REV); //figure this out at compy
				} else {
					armControl.setGainP(armControl.STARTING_PID_GAIN);
					armControl.setPositionTarget(OPEN_REV + twistOffset, -OPEN_REV + twistOffset); //figure out
				}
			} else {
				RobotLogger.toast("Arm collision fowards, prevent move!!!");
				RobotLogger.toast(elevatorControl.getPosition() + " " + MIN_HEIGHT_MOVE);
				currentMode = 'N';
				//elevatorTask.requestUpOut();
			}
		}
		
		SmartDashboard.putString("arm mode", Character.toString(currentMode));
		
	}

}
