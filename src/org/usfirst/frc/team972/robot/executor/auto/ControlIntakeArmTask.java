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
	
	double feedFowardGain = 0.9;
	boolean allowedControl = false;
	
	double leftPositionTarget = 0;
	double rightPositionTarget = 0;
	
	double GEARBOX_RATIO = 100;
	
	TrapezoidalMotionProfile mpLeft = new TrapezoidalMotionProfile(0.9, 1.25);
	TrapezoidalMotionProfile mpRight = new TrapezoidalMotionProfile(0.9, 1.25);
	
	PIDControl pidLeft = new PIDControl(9, 0, 0.25);
	PIDControl pidRight = new PIDControl(9, 0, 0.25);
	
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
	
	public void setPositionTargetOnce(double left, double right) {
		mpLeft.setRealPositions(left);
		mpRight.setRealPositions(right);
	}
	
	public double[] update(TrapezoidalMotionProfile mp, double target, double dt, double real) {
		mp.update(target, CoolMath.roundDigits(dt, 4));
		double position = CoolMath.roundDigits(mp.position, 3);
		double velocity = CoolMath.roundDigits(mp.velocity, 3);
	
		return new double[]{position, velocity};
	}

	public void execute(double dt) {
		double leftRealPos = -((double)sensors.getLeftIntake())/2048;
		leftRealPos = leftRealPos/GEARBOX_RATIO;
		double[] leftDesiredState = update(mpLeft, leftPositionTarget, dt, leftRealPos);
		pidLeft.setSetpoint(leftDesiredState[0]);
		pidLeft.setF(leftDesiredState[1] * feedFowardGain);
		double leftOutput = pidLeft.getOutput(leftRealPos); //position PID
		
		double rightRealPos = ((double)sensors.getRightIntake())/2048;
		rightRealPos = rightRealPos/GEARBOX_RATIO;
		double[] rightDesiredState = update(mpRight, rightPositionTarget, dt, rightRealPos);
		pidRight.setSetpoint(rightDesiredState[0]);
		pidRight.setF(rightDesiredState[1] * feedFowardGain);
		double rightOutput = pidRight.getOutput(rightRealPos); //position PID
		
		SmartDashboard.putNumber("left arm real pos", leftRealPos);
		SmartDashboard.putNumber("left arm desired pos", leftDesiredState[0]);
		
		SmartDashboard.putNumber("right arm real pos", rightRealPos);
		SmartDashboard.putNumber("right arm desired pos", rightDesiredState[0]);
		
		System.out.println(motors.intakeArmMotorLeft.getOutputCurrent() + motors.intakeArmMotorRight.getOutputCurrent());
		RobotLogger.toast(leftRealPos + " " + rightRealPos + " " + leftDesiredState[0] + " " + rightDesiredState[0]);
		
		motors.RunIntakeArmMotors(leftOutput, rightOutput);
	}
	
	public void setPositionTarget(double left, double right) {
		if(Math.abs(left) > 1.15) {
			left = Math.signum(left) * 1.15;
		}
		
		if(Math.abs(right) > 1.15) {
			right = Math.signum(right) * 1.15;
		}
		
		leftPositionTarget = left;
		rightPositionTarget = right;
	}

}
