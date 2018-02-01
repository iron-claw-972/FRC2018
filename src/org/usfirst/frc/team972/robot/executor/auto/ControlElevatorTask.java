package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motionlib.TrapezoidalMotionProfile;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;

public class ControlElevatorTask extends Task {

	double kp = 0.0;
	double ki = 0.0;
	double kd = 0.00;

	double ka = 0.0;
	double kv = 0.0;
	
	double lastError = 0;
	
	MechanismActuators elevatorMech;
	TrapezoidalMotionProfile mp = new TrapezoidalMotionProfile(0.5f, 0.25f);
	
	float elevatorPositionTarget = 0;
	
	public ControlElevatorTask(double _executionTime, MechanismActuators _elevatorMech) {
		super(_executionTime);
		elevatorMech = _elevatorMech;
		// TODO Auto-generated constructor stub
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void execute(double dt) {
		mp.update(elevatorPositionTarget, dt);
		double position = mp.position;
		double velocity = mp.velocity;
		double acceleration = mp.acceleration;
		
		double errorPos = elevatorPositionTarget - position;
		
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
