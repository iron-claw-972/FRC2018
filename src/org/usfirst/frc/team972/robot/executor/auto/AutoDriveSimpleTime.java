package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;

public class AutoDriveSimpleTime extends Task {

	double motorPower = 0;
	double lengthPower = 0;
	
	MainDriveTrain mdt;
	
	public AutoDriveSimpleTime(double _executionTime, double _lengthPower, double _motorPower, MainDriveTrain _mdt) {
		super(_executionTime);
		motorPower = _motorPower;
		lengthPower = _lengthPower;
		mdt = _mdt;
	}

	public void execute(double dt) {
		if(dt <= lengthPower) {
			mdt.driveSidesPWM(motorPower, motorPower);
		} else {
			mdt.driveSidesPWM(0, 0);
			super.finish();
		}
	}

	public void init(double dt) {
		
	}

}
