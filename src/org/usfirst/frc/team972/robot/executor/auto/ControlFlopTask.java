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

	boolean allowedControl = false;
	boolean setPositionOnce = false;
	
	double feedfoward = 0;
	double easingValue = 0.9;
	double lastWantedPos = 0;
	
	PIDControl pidFlopMotor = new PIDControl(15, 0.01, 0.025);
	
	double ka = 0.005;
	double kv = (double)1.5;

	final double GEARBOX_RATIO = 90;
	
	MechanismActuators flopMech;
	TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(0.25, 0.25);
	
	Sensors sensors;
	
	double flopPositionRevs = 0;
	
	public ControlFlopTask(double _executionTime, MechanismActuators _flopMech, Sensors _sensors) {
		super(_executionTime);
		flopMech = _flopMech;
		sensors = _sensors;
	}


	private double encoderPulseRevs(int pulse) {
		double rev = (double)pulse/(double)4096/GEARBOX_RATIO;
		return rev;
	}
	
	@Override
	public void init(double dt) {
		pidFlopMotor.setOutputLimits(-1, 1);
	}

	public void setElevatorPositionTarget(double target) {
		flopPositionRevs = target;
	}
	
	@Override
	public void execute(double dt) {		
		mp.update(flopPositionRevs, CoolMath.roundDigits(dt, 4));
		double realPosition = encoderPulseRevs(sensors.getFlopEncoder());
		double position = CoolMath.roundDigits(mp.position, 3);
		double velocity = CoolMath.roundDigits(mp.velocity, 3);
		double acceleration = mp.acceleration;
		
		RobotLogger.toast("flop pos: " + realPosition);
		
		executePid(velocity, acceleration, realPosition, position);
		
		lastWantedPos = position;
	}
	
	
	private void executePid(double velWant, double accWant, double realPos, double currWantPos) {
		double signnum = Math.signum(currWantPos - lastWantedPos);
		feedfoward = interpolateValues((kv * Math.abs(velWant) * signnum) + (ka * Math.abs(accWant) * signnum), feedfoward);
		pidFlopMotor.setF(feedfoward);
		double output = pidFlopMotor.getOutput(realPos, currWantPos) + feedfoward;

		flopMech.RunFlopMotor(handleDeadband(output, 0.025));
		RobotLogger.toast("flop o: "+ output);
	}
	
	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}
	
	public void setControl(boolean control) {
		allowedControl = control;
	}

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}
