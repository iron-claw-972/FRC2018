package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.Trajectory;
import org.usfirst.frc.team972.robot.motionlib.Trajectory.Segment;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;

public class AutoDriveVelocityProfileTask extends Task {

	Trajectory[] wheelTrajectories;
	MainDriveTrain mainDriveTrain;
	Sensors sensors;
	
	double leftPower = 0;
	double rightPower = 0;
	
	int currentVelocityIndex = 0;
	double integratedVelocity = 0;
	
	public AutoDriveVelocityProfileTask(double _executionTime, Trajectory[] _traj, Sensors _sensors, MainDriveTrain _mainDriveTrain) {
		super(_executionTime);
		wheelTrajectories = _traj;
		sensors = _sensors;
		mainDriveTrain = _mainDriveTrain;
	}

	public void init() {
		// TODO Auto-generated method stub
		currentVelocityIndex = 0;
	}

	public void execute(double dt) {
		int numSegs = wheelTrajectories[0].getNumSegments();
		if(currentVelocityIndex < numSegs) {
			Segment leftSeg = wheelTrajectories[0].getSegment(currentVelocityIndex);
			Segment rightSeg = wheelTrajectories[1].getSegment(currentVelocityIndex);
			
			double desiredLeftSpeed = leftSeg.vel;
			double desiredRightSpeed = rightSeg.vel;
			
			integratedVelocity = integratedVelocity + (desiredLeftSpeed * 0.02);
			
			RobotLogger.toast(desiredLeftSpeed + " " + desiredRightSpeed + " ------ " + integratedVelocity);
			mainDriveTrain.driveSidesPWM(desiredLeftSpeed, desiredRightSpeed);
			currentVelocityIndex++;
		} else {
			super.finish();
		}
	}

}
