package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.TrapezoidalMotionProfile;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

public class ControlElevatorTask extends Task {

	double kp = 0.0;
	double ki = 0.0;
	double kd = 0.00;

	double ka = 0.0;
	double kv = 0.0;
	
	double lastError = 0;
	
	final double DIAMETER_ELEVATOR_WINCH = 0.01905; // meters, not accounting for cord windup radius.
	
	MechanismActuators elevatorMech;
	TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(0.5f, 0.25f);
	
	Sensors sensors;
	
	float elevatorPositionTarget = 0;
	
	public ControlElevatorTask(double _executionTime, MechanismActuators _elevatorMech, Sensors _sensors) {
		super(_executionTime);
		elevatorMech = _elevatorMech;
		sensors = _sensors;
		// TODO Auto-generated constructor stub
	}

	private double calculatePositionMeters(double radians) {
		return radians * DIAMETER_ELEVATOR_WINCH;
	}
	
	private double encoderPulseToRadians(int pulse) {
		double rev = pulse/2048;
		return rev * 2 * Math.PI;
	}
	
	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}

	public void setElevatorPositionTarget(float elevatorPositionTarget) {
		RobotLogger.toast("Setting Elevator Position to: " + elevatorPositionTarget);
		this.elevatorPositionTarget = elevatorPositionTarget;
	}
	
	@Override
	public void execute(double dt) {
		mp.update(elevatorPositionTarget, dt);
		double realPosition = calculatePositionMeters(encoderPulseToRadians(sensors.getElevatorEncoder()));
		double position = mp.position;
		double velocity = mp.velocity;
		double acceleration = mp.acceleration;
		
		double errorPos = position - realPosition;
		
		if(checkElevatorSafety(position, velocity)) {
			executePid(dt, velocity, acceleration, errorPos);
		}
		
		lastError = errorPos;
	}
	
	private boolean checkElevatorSafety(double position, double velocity) {
		//TODO: write elevator bound  checking so we dont break the mechanism
		return true;	
	}
	
	private void executePid(double dt, double velWant, double accWant, double errorPos) {
		double output = (kp * errorPos) + (kd * (errorPos - lastError) / dt) + (kv * velWant)
				+ (ka * accWant);
		elevatorMech.RunElevatorLiftMotor(output);
	}
}
