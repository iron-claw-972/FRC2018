package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;

import com.kauailabs.navx.frc.AHRS;

public class AutoTurnAngleTask extends Task {
	
	double goalAngle;
	double maxTime;
	double feedFowardOffRange = 15;
	double errorRange = 1;
	
	double inTheZoneMax = 200;
	
	AHRS ahrs;
	MainDriveTrain mdt;
	PIDControl pid;
	
	double inTheZone = 0;
	
	public AutoTurnAngleTask(double _executionTime, double _angle, double _maxTime, MainDriveTrain _mdt, AHRS _ahrs) {
		super(_executionTime);
		maxTime = _maxTime;
		ahrs = _ahrs;
		mdt = _mdt;
		goalAngle = _angle;
	}

	public void execute(double dt) {
		if(dt > maxTime) {
			RobotLogger.toast("Auto Turn Angle Timeout, Auto Killing", RobotLogger.WARNING);
			super.free();
			super.destroy();
		} else {
			double heading = ahrs.getAngle();
			double error = goalAngle - heading;
			double output = pid.getOutput(heading, goalAngle);
			if(Math.abs(error) > feedFowardOffRange) {
				output = output + (Math.signum(error) * 0.25);
			}
			if(Math.abs(error) < errorRange) {
				inTheZone++;
				if(inTheZone > inTheZoneMax) {
					RobotLogger.toast("Auto Turn Angle Finished with Error of: " + error);
					super.free();
					super.destroy();
				}
			} else {
				inTheZone = 0; //reset if we go out of error range
			}
			RobotLogger.toast("pid output: " + output + " hdg: " + heading);
			mdt.driveSidesPWM(output, -output);
		}
		
	}

	public void init(double dt) {
		pid = new PIDControl(0.15, 0.0005, 0.1);
		pid.setMaxIOutput(0.15);
		pid.setOutputLimits(-1, 1);
		pid.setOutputRampRate(0.025);
		pid.setSetpointRange(errorRange);
		super.block();
	}

}
