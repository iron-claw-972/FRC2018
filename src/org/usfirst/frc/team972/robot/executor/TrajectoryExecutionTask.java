package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.Robot;
import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motionlib.CoolMath;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//based on 254's traj follower

public class TrajectoryExecutionTask extends Task {

	double kp = 0.35;
	double ki = 0.005;
	double kd = 0.01;

	double ka = 0.125;
	double kv = 1 / 1.4;

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

	boolean finished = false;
	
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

		desiredLeftVel = 0;
		desiredRightVel = 0;

		desiredLeftPos = 0;
		desiredRightPos = 0;

		desiredLeftAcc = 0;
		desiredRightAcc = 0;

		desiredAngle = 0;

		leftError = 0;
		rightError = 0;
		leftErrorLast = 0;
		rightErrorLast = 0;
		leftErrorSum = 0;
		rightErrorSum = 0;

		lastTime = 0;
		super.block();
	}

	private double calculateOutput(double errorPos, double velWant, double accWant, double lastError, double realDt) {
		double output = (kp * errorPos) + (kd * (errorPos - lastError) / (realDt)) + (kv * velWant)
				+ (ka * accWant);
		return output;
	}

	@Override
	public void execute(double dt) {
		if(finished == false ) {
			double realDt = Math.max(dt - lastTime, (double) 1000 / Robot.REAL_TIME_LOOP_HZ / 1000);
	
			double leftRealDist = -driveTrain.pulseToMetersLinear(sensors.getLeftDriveEncoder());
			double rightRealDist = driveTrain.pulseToMetersLinear(sensors.getRightDriveEncoder());
	
			double leftError = desiredLeftPos - leftRealDist;
			double rightError = desiredRightPos - rightRealDist;
	
			double lo = calculateOutput(leftError, desiredLeftVel, desiredLeftAcc, leftErrorLast, realDt);
			double ro = calculateOutput(rightError, desiredRightVel, desiredRightAcc, rightErrorLast, realDt);
			
			double currentAngle = -ahrs.getAngle(); //Invert Angle
	
			RobotLogger.toast(desiredAngle - currentAngle + "");
			
			if (Math.abs(lo) <= saturatedLimit) {
				leftErrorSum += leftError * realDt;
			}
			if (Math.abs(ro) <= saturatedLimit) {
				rightErrorSum += rightError * realDt;
			}
	
			lo += leftErrorSum * ki;
			ro += rightErrorSum * ki;
	
			if (Math.abs(lo) > 1) {
				lo = Math.signum(lo);
			}
			if (Math.abs(ro) > 1) {
				ro = Math.signum(ro);
			}
	
			driveTrain.driveSidesPWM(lo, ro);
	
			lastTime = dt;
			leftErrorLast = leftError;
			rightErrorLast = rightError;
		} else {
			driveTrain.stopHard();
			super.free();
			super.destroy();
		}
	}

	public void setpoint(double a, double b, double c, double d, double e, double f, double _ang, boolean _finished) {
		desiredLeftVel = a;
		desiredRightVel = b;

		desiredLeftPos = c;
		desiredRightPos = d;

		desiredLeftAcc = e;
		desiredRightAcc = f;

		desiredAngle = Math.toDegrees(_ang);
		finished = _finished;
	}

}
