package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motionlib.Trajectory;
import org.usfirst.frc.team972.robot.motionlib.Trajectory.Segment;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;

import com.kauailabs.navx.frc.AHRS;

public class AutoDriveVelocityProfileTask extends Task {

	TrajectoryExecutionTask follower;
	Trajectory[] wheelTrajectories;
	Trajectory realTraj;
	Sensors sensors;
	
	double leftPower = 0;
	double rightPower = 0;
	
	double integratedVelocity = 0;
	
	double initTime = 0;
	double finalPos = 0;
	double deltaWaitTime = 0;
	
	boolean invert = false;
	
	public AutoDriveVelocityProfileTask(double _executionTime, Trajectory[] _traj, Sensors _sensors, TrajectoryExecutionTask _follower, Trajectory _realTraj, boolean _invert) {
		super(_executionTime);
		wheelTrajectories = _traj;
		sensors = _sensors;
		follower = _follower;
		realTraj = _realTraj;
		invert = _invert;
	}

	public void deltaWait(double wait) {
		deltaWaitTime = wait;
	}
	
	public void init(double dt) {
		// TODO Auto-generated method stub
		initTime = dt;
		finalPos = (wheelTrajectories[0].getSegment(wheelTrajectories[0].getNumSegments()).pos + wheelTrajectories[0].getSegment(wheelTrajectories[0].getNumSegments()).pos) / (double) 2;
	}
	
	public int getSegmentDt(double currentTime) {
		Segment lastSeg = wheelTrajectories[0].getSegment(0);
		for(int i=0; i<wheelTrajectories[0].getNumSegments(); i++) {
			double segTime = wheelTrajectories[0].getSegment(i).dt * i;
			if(currentTime < segTime) {
				return i;
			}
		}
		return wheelTrajectories[0].getNumSegments();
	}

	public void execute(double dt) {
		dt = dt - initTime;
		if(dt < deltaWaitTime) {
			//Waiting on it!
			return;
		}
		int numSegs = wheelTrajectories[0].getNumSegments();
		int currentSegIndex = getSegmentDt(dt);
		if(currentSegIndex < numSegs) {
			double targetAngle = realTraj.getSegment(currentSegIndex).heading;
			
			Segment leftSeg = wheelTrajectories[0].getSegment(currentSegIndex);
			Segment rightSeg = wheelTrajectories[1].getSegment(currentSegIndex);
			
			double desiredLeftSpeed = leftSeg.vel;
			double desiredRightSpeed = rightSeg.vel;
			double desiredLeftP = leftSeg.pos;
			double desiredRightP = rightSeg.pos;
			double desiredLeftA = leftSeg.acc;
			double desiredRightA = rightSeg.acc;
			
			if(invert) {
				desiredLeftSpeed = -desiredLeftSpeed;
				desiredRightSpeed = -desiredRightSpeed;
				desiredLeftP = -desiredLeftP;
				desiredRightP = -desiredRightP;
				desiredLeftA = -desiredLeftA;
				desiredRightA = -desiredRightA;
			}
			
			follower.setpoint(desiredLeftSpeed, desiredRightSpeed, desiredLeftP, desiredRightP, desiredLeftA, desiredRightA, targetAngle, false);
		} else {
			follower.setpoint(0, 0, 0, 0, 0, 0, 0, true); //we done
			super.destroy();
		}
	}

}
