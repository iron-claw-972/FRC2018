package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.Robot;
import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motionlib.ChezyMath;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;

//based on 254's traj follower

public class TrajectoryExecutionTask extends Task {

	double kp = 0.5;
	double ki = 0.05;
	double kd = 0.1;
	
	double ka = 0.05;
	double kv = 0.2;
	
	double saturatedLimit = 0.5;
	
	//-----------------------------
	
	MainDriveTrain driveTrain = new MainDriveTrain();
	
	double desiredLeftVel = 0;
	double desiredRightVel = 0;
	
	double desiredLeftPos = 0;
	double desiredRightPos = 0;
	
	double desiredLeftAcc = 0;
	double desiredRightAcc = 0;
	
	double leftError = 0;
	double rightError = 0;
	double leftErrorLast = 0;
	double rightErrorLast = 0;
	double leftErrorSum = 0;
	double rightErrorSum = 0;
	
	Sensors sensors;
	
	double lastTime = 0;
	
	public TrajectoryExecutionTask(double _executionTime, MainDriveTrain _mdt, Sensors _sensors) {
		super(_executionTime);
		driveTrain = _mdt;
		sensors = _sensors;
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		lastTime = dt;
		leftError = 0;
		rightError = 0;
	}

	private double calculateOutput(double errorPos, double velWant, double accWant, double lastError, double realDt) {
	    double output = kp * errorPos + kd
	                * ((errorPos - lastError) / realDt - velWant)
	                + (kv * velWant + ka * accWant);
		return output;
	}
	
	@Override
	public void execute(double dt) {
		double realDt = Math.max(dt - lastTime, (double)1000/Robot.REAL_TIME_LOOP_HZ/1000);
		
		double leftRealDist = desiredLeftPos;//driveTrain.pulseToMetersLinear(sensors.getLeftDriveEncoder());
		double rightRealDist = desiredRightPos;//driveTrain.pulseToMetersLinear(sensors.getRightDriveEncoder());
		
		double leftError = desiredLeftPos - leftRealDist;
		double rightError = desiredRightPos - rightRealDist;
		
		double lo = calculateOutput(leftError, desiredLeftVel, desiredLeftAcc, leftErrorLast, realDt);
		double ro = calculateOutput(rightError, desiredRightVel, desiredRightAcc, rightErrorLast, realDt);
		
		if(Math.abs(lo) <= saturatedLimit) {
			leftErrorSum += leftError * realDt;
		}
		if(Math.abs(ro) <= saturatedLimit) {
			rightErrorSum += rightError * realDt;
		}
		
		lo += leftErrorSum * ki;
		ro += rightErrorSum * ki;
		
		RobotLogger.toast(lo + " " + ro);
		
		driveTrain.driveSidesPWM(lo, ro);
		
		lastTime = dt;
		leftErrorLast = leftError;
		rightErrorLast = rightError;
	}

	public void setpoint(double a, double b, double c, double d, double e, double f) {
		desiredLeftVel = a;
		desiredRightVel = b;
			
		desiredLeftPos = c;
		desiredRightPos = d;
			
		desiredLeftAcc = e;
		desiredRightAcc = f;
	}

}
