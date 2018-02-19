package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.executor.TeleopElevatorTask;
import org.usfirst.frc.team972.robot.motionlib.CoolMath;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motionlib.TrapezoidalMotionProfile;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlElevatorTask extends Task {

	boolean allowedControl = false;
	boolean setPositionOnce = false;
	
	boolean goingUpPastBar = false;
	
	double feedfoward = 0;
	double easingValue = 0.9;
	double lastWantedPos = 0;
	
	PIDControl pidRightWinch = new PIDControl(0.9, 0.005, 0.025);
	
	double ka = 0.005;
	double kv = (double)1/.42;

	final double DIAMETER_ELEVATOR_WINCH = 0.023495; // meters, not accounting for cord windup radius.
	final double GEARBOX_RATIO = 36;
	
	MechanismActuators elevatorMech;
	TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(0.3, 0.5);
	
	ControlFlopTask flopControl;
	
	Sensors sensors;
	
	double elevatorPositionTarget = 0;
	
	public ControlElevatorTask(double _executionTime, MechanismActuators _elevatorMech, Sensors _sensors, ControlFlopTask _flopControl) {
		super(_executionTime);
		elevatorMech = _elevatorMech;
		flopControl = _flopControl;
		sensors = _sensors;
		// TODO Auto-generated constructor stub
	}
	
	private double calculatePositionMeters(double radians) {
		return radians * DIAMETER_ELEVATOR_WINCH;
	}
	
	private double encoderPulseToRadians(int pulse) {
		double rev = (double)pulse/(double)4096/GEARBOX_RATIO;
		return rev * Math.PI;
	}
	
	@Override
	public void init(double dt) {
		pidRightWinch.setOutputFilter(0.025);
		pidRightWinch.setMaxIOutput(0.05);
		pidRightWinch.setOutputLimits(-1, 1);
	}

	public void setElevatorPositionTarget(double elevatorPositionTarget) {
		RobotLogger.toast("Setting Elevator Position to: " + elevatorPositionTarget);
		this.elevatorPositionTarget = elevatorPositionTarget;
		SmartDashboard.putNumber("elevator target pos: ", elevatorPositionTarget);
	}
	
	@Override
	public void execute(double dt) {		
		mp.update(elevatorPositionTarget, CoolMath.roundDigits(dt, 4));
		double realPosition = calculatePositionMeters(encoderPulseToRadians(sensors.getElevatorEncoder()));
		double position = CoolMath.roundDigits(mp.position, 3);
		double velocity = CoolMath.roundDigits(mp.velocity, 3);
		double acceleration = mp.acceleration;
		
		SmartDashboard.putNumber("elevator real pos: ", realPosition);
		SmartDashboard.putNumber("elevator curr target: ", position);
		SmartDashboard.putNumber("elevator desired vel", velocity);

		if(setPositionOnce) {
			setElevatorPositionTarget(realPosition);
			mp.setRealPositions(realPosition);
			setPositionOnce = false;
		}
		
		if(checkElevatorSafety(realPosition, velocity) && allowedControl) {
			executePid(velocity, acceleration, realPosition, position, elevatorPositionTarget);
		} else {
			if(allowedControl) {
				elevatorMech.RunElevatorLiftMotor(0);
			}
		}
		
		lastWantedPos = position;
	}
	
	private boolean checkElevatorSafety(double position, double velocity) {
		//TODO: write elevator bound  checking so we dont break the mechanism
		if((position > TeleopElevatorTask.POINT_OF_BAR_HIT) && (goingUpPastBar) && (flopControl.isDown() == false))
		{
			RobotLogger.toast("Elevator Safety Tripped, Bar Movment Up: " + position, RobotLogger.URGENT);
			return false;
		} else if(position > 2.1) {
			RobotLogger.toast("Elevator Safety Tripped, Max Height: " + position, RobotLogger.URGENT);
			return false;
		} else {
			return true;
		}
	}
	
	public double getPosition() {
		return calculatePositionMeters(encoderPulseToRadians(sensors.getElevatorEncoder()));
	}
	
	public void setGoPastBar(boolean val) {
		goingUpPastBar = val;
	}
	
	private void executePid(double velWant, double accWant, double realPos, double currWantPos, double finalWant) {
		/*double output = (kp * errorPos) + (kd * (errorPos - lastError) / dt) + (kv * velWant)
				+ (ka * accWant);
		if((output > 1) || (output < -1)) {
			output = Math.signum(output);
		}*/
		double signnum = Math.signum(currWantPos - lastWantedPos);
		feedfoward = interpolateValues((kv * Math.abs(velWant) * signnum) + (ka * Math.abs(accWant) * signnum), feedfoward);
		pidRightWinch.setF(feedfoward);
		double output = pidRightWinch.getOutput(realPos, currWantPos) + feedfoward;
		
		SmartDashboard.putNumber("elevator ff", feedfoward);
		
		elevatorMech.RunElevatorLiftMotor(handleDeadband(output, 0.05));
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
	
	public void setElevatorPositionTargetOnceCycle(double dt) {
		mp.setTime(dt);
		setPositionOnce = true;
	}
}
