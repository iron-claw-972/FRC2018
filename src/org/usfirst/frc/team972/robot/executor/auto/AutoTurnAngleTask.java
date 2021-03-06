package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;

import com.kauailabs.navx.frc.AHRS;

public class AutoTurnAngleTask extends Task {
	
	double goalAngle;
	double maxTime;
	double feedFowardOffRange = 10;
	double feedFowardPower = 0.6;
	double errorRange = 1;
	
	double inTheZoneMax = 50;
	
	AHRS ahrs;
	MainDriveTrain mdt;
	PIDControl pid;
	
	double realStartTime = 0;
	double inTheZone = 0;
	
	public AutoTurnAngleTask(double _executionTime, double _angle, double _maxTime, MainDriveTrain _mdt, AHRS _ahrs) {
		super(_executionTime);
		maxTime = _maxTime;
		ahrs = _ahrs;
		mdt = _mdt;
		goalAngle = _angle;
	}

	public void execute(double dt) {
		if(dt > (maxTime + realStartTime)) {
			RobotLogger.toast("Auto Turn Angle Timeout, Auto Killing", RobotLogger.WARNING);
			super.free();
			super.destroy();
		} else {
			double heading = ahrs.getAngle();
			double error = goalAngle - heading;
			double output = pid.getOutput(heading, goalAngle);
			if(Math.abs(error) > feedFowardOffRange) {
				output = output + (Math.signum(error) * feedFowardPower);
			}
			if(Math.abs(error) < errorRange) {
				inTheZone++;
				if(inTheZone > inTheZoneMax) {
					RobotLogger.toast("Auto Turn Angle Finished with Error of: " + error);
					mdt.stopHard();
					mdt.driveSidesPWM(0, 0);
					super.free();
					super.destroy();
				}
			} else {
				inTheZone = 0; //reset if we go out of error range
			}
			RobotLogger.toast("pid output: " + output + " hdg: " + heading);
			mdt.driveSidesPWM(output/2, -output/2);
		}
		
	}

	public void init(double dt) {
		pid = new PIDControl(0.6, 0.0005, 0.5);
		pid.setMaxIOutput(0.15);
		pid.setOutputLimits(-0.3, 0.3);
		pid.setOutputRampRate(0.005);
		pid.setSetpointRange(errorRange);
		realStartTime = dt;
		super.block();
	}

}
