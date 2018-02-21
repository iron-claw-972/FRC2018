package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
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
	
	double inGoodRange = 0.1;
	
	int inGoodCount = 0;
	int maxGoodCount = 50;
	
	double usePositionPidMeterMark = 1;
	double constantFeedVelocity = 0.55;
	
	MainDriveTrain mdt;
	PIDControl pidDistance;
	PIDControl pidAngle;
	AHRS ahrs;
	
	double regularPower = 0;
	
	public AutoDrivePositionAngle(double _executionTime, double _distance, double _angle, MainDriveTrain _mdt, double _maxTime, Sensors _sensors, AHRS _ahrs, double power) {
		super(_executionTime);
		desiredAngle = _angle;
		desiredDistance = _distance;
		maxTime = _maxTime;
		mdt = _mdt;
		sensors = _sensors;
		ahrs = _ahrs;
		pidDistance = new PIDControl(3.15, 0.05, 8);
		pidDistance.setOutputLimits(-0.5, 0.5);
		pidDistance.setSetpointRange(0.1);
		
		pidAngle = new PIDControl(0.025, 0, 0.05);
		pidAngle.setOutputLimits(-0.08, 0.08);
		pidAngle.setSetpointRange(2);
		pidAngle.setOutputFilter(0.1);

		constantFeedVelocity = power;
	}
	
	public AutoDrivePositionAngle(double _executionTime, double _distance, double _angle, MainDriveTrain _mdt, double _maxTime, Sensors _sensors, AHRS _ahrs) {
		super(_executionTime);
		desiredAngle = _angle;
		desiredDistance = _distance;
		maxTime = _maxTime;
		mdt = _mdt;
		sensors = _sensors;
		ahrs = _ahrs;
		pidDistance = new PIDControl(3.15, 0.05, 8);
		pidDistance.setOutputLimits(-0.5, 0.5);
		pidDistance.setSetpointRange(0.1);
		
		pidAngle = new PIDControl(0.025, 0, 0.05);
		pidAngle.setOutputLimits(-0.08, 0.08);
		pidAngle.setSetpointRange(2);
		pidAngle.setOutputFilter(0.1);
	}

	public void execute(double dt) {
		double distanceLeft = mdt.pulseToMetersLinear(sensors.getLeftDriveEncoder());
		double distanceRight = -mdt.pulseToMetersLinear(sensors.getRightDriveEncoder());
		
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
				
				if(distanceRemaining < inGoodRange) {
					inGoodCount++;
				} else {
					inGoodCount = 0;
				}
				if(inGoodCount > maxGoodCount) {
					mdt.driveSidesPWM(0, 0);
					RobotLogger.toast("good!");
					super.destroy();
					super.free();
				}
			} else {
				double correctionAngleOutput = pidAngle.getOutput(currentAngle, desiredAngle);
				leftPower = Math.signum(distanceRemaining) * constantFeedVelocity;
				regularPower = interpolateValues(leftPower, regularPower);
				mdt.driveSidesPWM(regularPower + correctionAngleOutput, regularPower - correctionAngleOutput);
				System.out.println("angle: " + currentAngle + " correction: " + correctionAngleOutput);
			}
		} else {
			mdt.driveSidesPWM(0, 0);
			super.destroy();
			super.free();
		}
	}
	
	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * 0.1;
		return actual + error;
	}

	public void init(double dt) {
		sensors.resetDriveEncoders();
		super.block();
	}

}
