package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;

import com.kauailabs.navx.frc.AHRS;

public class AutoDrivePositionAngle extends Task {

	double desiredDistance;
	double desiredAngle;
	double maxTime;
	
	Sensors sensors;
	
	double usePositionPidMeterMark = 0.3;
	double constantFeedVelocity = 0.4;
	
	MainDriveTrain mdt;
	PIDControl pidDistance;
	PIDControl pidAngle;
	AHRS ahrs;
	
	public AutoDrivePositionAngle(double _executionTime, double _distance, double _angle, MainDriveTrain _mdt, double _maxTime, Sensors _sensors, AHRS _ahrs) {
		super(_executionTime);
		desiredAngle = _angle;
		desiredDistance = _distance;
		maxTime = _maxTime;
		mdt = _mdt;
		sensors = _sensors;
		ahrs = _ahrs;
		pidDistance = new PIDControl(0.8, 0.005, 1);
		pidDistance.setOutputLimits(-0.24, 0.24);
		
		pidAngle = new PIDControl(0.1, 0, 5);
		pidAngle.setOutputLimits(-0.1, 0.1);
		pidAngle.setSetpointRange(0.5);
		pidAngle.setOutputFilter(0.1);
	}

	public void execute(double dt) {
		double distanceLeft = -mdt.pulseToMetersLinear(sensors.getLeftDriveEncoder());
		double distanceRight = mdt.pulseToMetersLinear(sensors.getRightDriveEncoder());
		
		double averageDistance = (distanceLeft + distanceRight)/2;
		double distanceRemaining = desiredDistance - averageDistance;
		
		double currentAngle = ahrs.getAngle();
		
		double leftPower;
		double rightPower;
		
		if(dt <= maxTime) {
			System.out.println("dist: " + averageDistance);
				
			if(distanceRemaining < usePositionPidMeterMark) {
				double drivePositionOutput = pidDistance.getOutput(averageDistance, desiredDistance);
				leftPower = drivePositionOutput;
				rightPower = drivePositionOutput;
				mdt.driveSidesPWM(leftPower, rightPower);
			} else {
				double correctionAngleOutput = pidAngle.getOutput(currentAngle, desiredAngle);
				leftPower = Math.signum(distanceRemaining) * constantFeedVelocity;
				rightPower = leftPower;
				mdt.driveSidesPWM(leftPower + correctionAngleOutput, rightPower - correctionAngleOutput);
				System.out.println("angle: " + currentAngle + " correction: " + correctionAngleOutput);
			}
		} else {
			mdt.driveSidesPWM(0, 0);
			super.destroy();
			super.free();
		}
	}

	public void init(double dt) {
		super.block();
	}

}
