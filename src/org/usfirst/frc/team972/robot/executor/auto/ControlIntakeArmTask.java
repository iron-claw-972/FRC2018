package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.CoolMath;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motionlib.TrapezoidalMotionProfile;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlIntakeArmTask extends Task {

	Sensors sensors;
	MechanismActuators motors;
	
	double feedFowardGain = 1;
	public boolean allowedControl = false;
	
	double leftPositionTarget = 0;
	double rightPositionTarget = 0;
	
	double GEARBOX_RATIO = 100;
	
	final double SAFE_OUTWARDS_HITBOX = 0.3;
	final double OUTWARDS_LIMIT = 1.15;
	boolean setPositionOnce = false;
	
	public double STARTING_PID_GAIN = 10;
	
	TrapezoidalMotionProfile mpLeft = new TrapezoidalMotionProfile(1.1, 2.2);
	TrapezoidalMotionProfile mpRight = new TrapezoidalMotionProfile(1.1, 2.2);
	
	PIDControl pidLeft = new PIDControl(STARTING_PID_GAIN, 0, 0.25);
	PIDControl pidRight = new PIDControl(STARTING_PID_GAIN, 0, 0.25);
	
	public ControlIntakeArmTask(double _executionTime, Sensors _sensors, MechanismActuators _motors) {
		super(_executionTime);
		// TODO Auto-generated constructor stub
		sensors = _sensors;
		motors = _motors;
	}

	@Override
	public void init(double dt) {
		pidLeft.setOutputLimits(1);
		pidRight.setOutputLimits(1);
	}
		
	public void setGainP(double p) {
		pidLeft.setP(p);
		pidRight.setP(p);
	}
	
	public double getLeftPos() {
		double leftRealPos = -((double)sensors.getLeftIntake())/2048;
		leftRealPos = leftRealPos/GEARBOX_RATIO;
		return leftRealPos;
	}
	
	public double getRightPos() {
		double rightRealPos = ((double)sensors.getRightIntake())/2048;
		rightRealPos = rightRealPos/GEARBOX_RATIO;
		return rightRealPos;
	}
	
	public void setPositionTargetOnceCycle(double dt) {
		mpLeft.setRealPositions(getLeftPos());
		mpRight.setRealPositions(getRightPos());
		mpLeft.setTime(dt);
		mpRight.setTime(dt);
		setPositionOnce = true;
	}
	
	public double[] update(TrapezoidalMotionProfile mp, double target, double dt, double real) {
		mp.update(target, CoolMath.roundDigits(dt, 4));
		double position = CoolMath.roundDigits(mp.position, 3);
		double velocity = CoolMath.roundDigits(mp.velocity, 3);
	
		return new double[]{position, velocity};
	}

	public boolean isSafeOutwards() {
		double leftRealPos = getLeftPos();
		double rightRealPos = getRightPos();
		
		leftRealPos = Math.abs(leftRealPos);
		rightRealPos = Math.abs(rightRealPos);
		
		if((Math.abs(leftRealPos) < SAFE_OUTWARDS_HITBOX) || (Math.abs(rightRealPos) < SAFE_OUTWARDS_HITBOX)) {
			return false;
		} else {
			return true;
		}
	}
	
	double lastLeftWant = 0;
	double lastRightWant = 0;
	
	public void execute(double dt) {
		if(setPositionOnce) {
			setPositionTarget(getLeftPos(), getRightPos());
			mpLeft.setRealPositions(getLeftPos());
			mpRight.setRealPositions(getRightPos());
			setPositionOnce = false;
		}
		
		double leftRealPos = getLeftPos();
		double[] leftDesiredState = update(mpLeft, leftPositionTarget, dt, leftRealPos);
		pidLeft.setSetpoint(leftDesiredState[0]);
		double leftOutput = pidLeft.getOutput(leftRealPos) + (leftDesiredState[1] * feedFowardGain); //position PID
		
		double rightRealPos = getRightPos();
		double[] rightDesiredState = update(mpRight, rightPositionTarget, dt, rightRealPos);
		pidRight.setSetpoint(rightDesiredState[0]);;
		double rightOutput = pidRight.getOutput(rightRealPos) + (rightDesiredState[1] * feedFowardGain); //position PID
		
		SmartDashboard.putNumber("left arm real pos", leftRealPos);
		SmartDashboard.putNumber("left arm desired pos", leftDesiredState[0]);
		
		SmartDashboard.putNumber("right arm real pos", rightRealPos);
		SmartDashboard.putNumber("right arm desired pos", rightDesiredState[0]);
		
		lastLeftWant = leftDesiredState[0];
		lastRightWant = rightDesiredState[0];
		
		if(allowedControl) {
			motors.RunIntakeArmMotors(-rightOutput, leftOutput);
		}
	}
	
	public void setPositionTarget(double left, double right) {
		if(Math.abs(left) > OUTWARDS_LIMIT) {
			left = Math.signum(left) * 1.15;
		}
		
		if(Math.abs(right) > OUTWARDS_LIMIT) {
			right = Math.signum(right) * 1.15;
		}
		
		leftPositionTarget = left;
		rightPositionTarget = right;
	}

}
