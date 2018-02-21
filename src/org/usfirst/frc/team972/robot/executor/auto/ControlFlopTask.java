package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.CoolMath;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motionlib.TrapezoidalMotionProfile;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlFlopTask extends Task {

	final double DOWN_MINIMUM = 0.05;
	
	final double COEFFICENT_TORQUE_FEEDFOWARD = 0.05;
	final double STARTING_REV = 0.245;
	
	boolean allowedControl = false;
	boolean setPositionOnce = false;
	
	double feedfoward = 0;
	double easingValue = 0.9;
	double lastWantedPos = 0;
	
	PIDControl pidFlopMotor = new PIDControl(2, 0.0005, 0);
	
	double ka = 0.005;
	double kv = (double)5.5;
	
	MechanismActuators flopMech;
	TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(0.05, 0.2);
	
	Sensors sensors;
	
	double flopPositionRevs = 0;
	
	public ControlFlopTask(double _executionTime, MechanismActuators _flopMech, Sensors _sensors) {
		super(_executionTime);
		flopMech = _flopMech;
		sensors = _sensors;
		mp.setRealPositions(STARTING_REV);
	}

	private double encoderPulseRevs(int pulse) {
		double rev = (double)pulse/(double)2048;
		return rev;
	}
	
	@Override
	public void init(double dt) {
		pidFlopMotor.setOutputLimits(-0.3, 0.3);
	}

	public void setFlopPositionTarget(double target) {
		flopPositionRevs = target;
	}
	
	@Override
	public void execute(double dt) {		
		mp.update(flopPositionRevs, CoolMath.roundDigits(dt, 4));
		double realPosition = encoderPulseRevs(sensors.getFlopEncoder()) + STARTING_REV;
		double position = CoolMath.roundDigits(mp.position, 3);
		double velocity = CoolMath.roundDigits(mp.velocity, 3);
		double acceleration = mp.acceleration;
		
		SmartDashboard.putNumber("flop want", position);
		SmartDashboard.putNumber("flop pos", realPosition);
		
		executePid(velocity, acceleration, realPosition, position);
		
		lastWantedPos = position;
	}
	
	
	private void executePid(double velWant, double accWant, double realPos, double currWantPos) {
		double signnum = Math.signum(currWantPos - lastWantedPos);
		feedfoward = interpolateValues((kv * Math.abs(velWant) * signnum) + (ka * Math.abs(accWant) * signnum), feedfoward);
		pidFlopMotor.setF(feedfoward);
		double output = pidFlopMotor.getOutput(realPos, currWantPos);

		double angle = (realPos * 360);
		double torque = Math.cos(Math.toRadians(angle)) * COEFFICENT_TORQUE_FEEDFOWARD; 
		
		output = output + torque;
		
		if(output > 1) {
			output = 1;
		} else if(output < -1) {
			output = -1;
		}

		//flopMech.RunFlopMotor(handleDeadband(output, 0.005));
		SmartDashboard.putNumber("flop o", output);
	}
	
	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}
	
	public void setControl(boolean control) {
		allowedControl = control;
	}
	
	public double getFlopCurrentPos() {
		return encoderPulseRevs(sensors.getFlopEncoder());
	}

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

	public boolean isDown() {
		//readibility
		if(getFlopCurrentPos() < DOWN_MINIMUM) {
			return true;
		} else {
			return false;
		}
	}
}
