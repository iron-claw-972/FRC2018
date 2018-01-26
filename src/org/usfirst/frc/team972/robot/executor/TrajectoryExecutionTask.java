package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.Robot;
import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motionlib.ChezyMath;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;

import com.kauailabs.navx.frc.AHRS;

//based on 254's traj follower

public class TrajectoryExecutionTask extends Task {

	double kp = 0.0;
	double ki = 0.0;
	double kd = 0.0;

	double ka = 0.0;
	double kv = 1/1.4;

	double saturatedLimit = 0.5;

	// -----------------------------
	
	AHRS ahrs;
	MainDriveTrain driveTrain = new MainDriveTrain();
	PIDControl headingPid = new PIDControl(0.135, 0.0, 0.8);

	double desiredLeftVel = 0;
	double desiredRightVel = 0;

	double desiredLeftPos = 0;
	double desiredRightPos = 0;

	double desiredLeftAcc = 0;
	double desiredRightAcc = 0;

	double desiredAngle = 0;
	
	double leftError = 0;
	double rightError = 0;
	double leftErrorLast = 0;
	double rightErrorLast = 0;
	double leftErrorSum = 0;
	double rightErrorSum = 0;

	Sensors sensors;

	double lastTime = 0;

	public TrajectoryExecutionTask(double _executionTime, MainDriveTrain _mdt, Sensors _sensors, AHRS _ahrs) {
		super(_executionTime);
		driveTrain = _mdt;
		sensors = _sensors;
		ahrs = _ahrs;
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		lastTime = dt;
		leftError = 0;
		rightError = 0;
		headingPid.setOutputLimits(-0.15, 0.15);
		headingPid.setSetpointRange(1);
	}

	private double calculateOutput(double errorPos, double velWant, double accWant, double lastError, double realDt) {
		double output = kp * errorPos + kd * ((errorPos - lastError) / realDt - velWant)
				+ (kv * velWant + ka * accWant);
		return output;
	}

	@Override
	public void execute(double dt) {
		double actualAngle = ahrs.getAngle();
		double steeringCorrect = headingPid.getOutput(actualAngle, desiredAngle);
				
		double realDt = Math.max(dt - lastTime, (double) 1000 / Robot.REAL_TIME_LOOP_HZ / 1000);

		double leftRealDist = desiredLeftPos;// driveTrain.pulseToMetersLinear(sensors.getLeftDriveEncoder());
		double rightRealDist = desiredRightPos;// driveTrain.pulseToMetersLinear(sensors.getRightDriveEncoder());

		double leftError = desiredLeftPos - leftRealDist;
		double rightError = desiredRightPos - rightRealDist;

		double lo = calculateOutput(leftError, desiredLeftVel, desiredLeftAcc, leftErrorLast, realDt) + steeringCorrect;
		double ro = calculateOutput(rightError, desiredRightVel, desiredRightAcc, rightErrorLast, realDt) - steeringCorrect;

		if (Math.abs(lo) <= saturatedLimit) {
			leftErrorSum += leftError * realDt;
		}
		if (Math.abs(ro) <= saturatedLimit) {
			rightErrorSum += rightError * realDt;
		}

		lo += leftErrorSum * ki;
		ro += rightErrorSum * ki;

		//RobotLogger.toast(desiredLeftVel + " " + desiredRightVel);

		RobotLogger.toast((actualAngle - desiredAngle) + " output: " + steeringCorrect);
		
		driveTrain.driveSidesPWM(lo, ro);

		lastTime = dt;
		leftErrorLast = leftError;
		rightErrorLast = rightError;
	}

	public void setpoint(double a, double b, double c, double d, double e, double f, double _ang) {
		desiredLeftVel = a;
		desiredRightVel = b;

		desiredLeftPos = c;
		desiredRightPos = d;

		desiredLeftAcc = e;
		desiredRightAcc = f;
		
		desiredAngle = _ang;
	}

}
