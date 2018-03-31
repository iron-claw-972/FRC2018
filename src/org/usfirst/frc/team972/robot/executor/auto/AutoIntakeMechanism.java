package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

public class AutoIntakeMechanism extends Task {

	double timeout;
	boolean pullingIn;
	double power;

	double output = 0;
	double easingValue = 0.2;
	double overdrawReducePower = 0.15;
	
	int counterIntakeTripped = 0;

	Sensors sensors;
	MechanismActuators motors;
	
	public AutoIntakeMechanism(double _executionTime, double _timeout, boolean _pullingIn, double _power, Sensors _sensors, MechanismActuators _motors) {
		super(_executionTime);
		timeout = _timeout;
		pullingIn = _pullingIn;
		power = Math.abs(_power);
		sensors = _sensors;
		motors = _motors;
	}

	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}
	
	@Override
	public void init(double dt) {
		
	}
	
	@Override
	public void execute(double dt) {
		/*
		 * TODO: FIX ALL THIS 
		 */
		if(dt > timeout) {
			RobotLogger.toast("Intake Timeout");
			output = 0;
			super.destroy();
		} else {
			//boolean frontIntakeSensorValue = sensors.getFrontIntakeSensorValue();

			if(pullingIn) {
				output = interpolateValues(-power, output);
			} else {
				output = interpolateValues(power, output);
			}
			
			if(motors.IntakeMotorOverdraw()) {
				output = interpolateValues(Math.signum(output) * overdrawReducePower, output);
			}
		}
		
		motors.RunIntakeMotors(output);
	}

}
