package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

public class AutoIntakeMechanism extends Task {

	double timeout;
	boolean pullingIn;
	double power;

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
			super.destroy();
		} else {
			boolean frontIntakeSensorValue = sensors.getFrontIntakeSensorValue();
			boolean backIntakeSensorValue = true; //sensors.getBackIntakeSensorValue();
			if(pullingIn) {
				if (backIntakeSensorValue == false) {
					motors.RunIntakeMotors(-power);
				} else {
					RobotLogger.toast("Intake Finished Pulling In");
					super.destroy(); //we have it in all the way!
				}
			} else {
				if(frontIntakeSensorValue || backIntakeSensorValue) {
					motors.RunIntakeMotors(power);
				} else {
					//we have shot out the cube!
					RobotLogger.toast("Intake Finished Shooting");
					super.destroy();
				}
			}
		}
	}

}
