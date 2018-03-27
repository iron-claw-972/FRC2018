package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.auto.ControlIntakeArmTask;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopIntakeArmTask extends Task {

	UserInputGamepad uig;
	
	ControlIntakeArmTask armControl;
	
	boolean clampModeToggle = false;
	
	private final double CLAMP_REV = 0.6;
	private final double OPEN_REV = 0.5;
	private final double BACK_REV = 0;
	
	char currentMode = 'B';
	
	Sensors sensors;
	
	public TeleopIntakeArmTask(double _executionTime, UserInputGamepad _uig, ControlIntakeArmTask _armControl, Sensors _sensors) {
		super(_executionTime);
		uig = _uig;
		armControl = _armControl;
	}

	@Override
	public void init(double dt) {
		if((Math.abs(sensors.getLeftIntake()) + Math.abs(sensors.getRightIntake())) > 1) {
			currentMode = 'C';
			RobotLogger.toast("Intake Arm Detected as READY");
		} else {
			currentMode = 'B';
			RobotLogger.toast("Intake Arm Detected as BACK");
		}
	}

	@Override
	public void execute(double dt) {
		Joystick joystick = uig.getStickB();
		
		boolean moveBackOverride = joystick.getRawButton(5);
		
		if(moveBackOverride) {
			currentMode = 'B';
		} else if(joystick.getRawButtonPressed(3)) {
			clampModeToggle = !clampModeToggle;
			currentMode = 'C';
		}

		if(currentMode == 'B') {
			armControl.setPositionTarget(BACK_REV, BACK_REV);
		} else if(currentMode == 'C') {
			if(clampModeToggle) {
				armControl.setPositionTarget(CLAMP_REV, -CLAMP_REV); //figure this out at compy
			} else {
				armControl.setPositionTarget(OPEN_REV, -OPEN_REV); //figure out
			}
		}
		
	}

}
