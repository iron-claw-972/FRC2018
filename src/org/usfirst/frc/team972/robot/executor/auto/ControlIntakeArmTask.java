package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.CoolMath;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motionlib.TrapezoidalMotionProfile;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

public class ControlIntakeArmTask extends Task {

	Sensors sensors;
	MechanismActuators motors;
	
	boolean allowedControl = false;
	
	double leftPositionTarget = 0;
	double rightPositionTarget = 0;
	
	TrapezoidalMotionProfile mpLeft = new TrapezoidalMotionProfile(0.5, 0.5);
	TrapezoidalMotionProfile mpRight = new TrapezoidalMotionProfile(0.5, 0.5);
	
	PIDControl pidLeft = new PIDControl(1, 0.05, 0.5);
	PIDControl pidRight = new PIDControl(1, 0.05, 0.5);
	
	public ControlIntakeArmTask(double _executionTime, Sensors _sensors, MechanismActuators _motors) {
		super(_executionTime);
		// TODO Auto-generated constructor stub
		sensors = _sensors;
		motors = _motors;
	}

	@Override
	public void init(double dt) {
		
	}
	
	public double[] update(TrapezoidalMotionProfile mp, double target, double dt, double real) {
		mp.update(target, CoolMath.roundDigits(dt, 4));
		double position = CoolMath.roundDigits(mp.position, 3);
		double velocity = CoolMath.roundDigits(mp.velocity, 3);
	
		return new double[]{position, velocity};
	}

	public void execute(double dt) {
		double leftRealPos = ((double)sensors.getLeftIntake())/2048;
		double[] leftDesiredState = update(mpLeft, leftPositionTarget, dt, leftRealPos);
		pidLeft.setF(leftDesiredState[1]); //velocity FF
		double leftOutput = pidLeft.getOutput(leftDesiredState[0]); //position PID
		
		double rightRealPos = ((double)sensors.getRightIntake())/2048;
		double[] rightDesiredState = update(mpRight, rightPositionTarget, dt, rightRealPos);
		pidRight.setF(rightDesiredState[1]); //velocity FF
		double rightOutput = pidRight.getOutput(rightDesiredState[0]); //position PID
	
		motors.RunIntakeArmMotors(leftOutput, rightOutput);
	}
	
	public void setPositionTarget(double left, double right) {
		leftPositionTarget = left;
		rightPositionTarget = right;
	}

}
