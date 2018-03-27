package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class IntakeSystemTask extends Task{
	
	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	Sensors sensors;
	
	double easingValue = 0.1;
	double intakeMotorPower = 0.5;
	double intakeSlowPower = 0.15;
	double overdrawReducePower = 0.1;
	
	double intakeOutputPower = 0;
	
	boolean activateIntakeMotors;
	boolean reverseIntakeMotors;
	boolean frontIntakeSensorValue;
	boolean backIntakeSensorValue;
	boolean fireBlockSlow;
	
	public IntakeSystemTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors, Sensors _sensors) {
		super(_executionTime);
		uig = _uig;
		mechanismMotors = _mechanismMotors;
		sensors = _sensors;
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}
	
	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}

	@Override
	public void execute(double dt) {
		Joystick mainJoystick = uig.getStickB();
		activateIntakeMotors = mainJoystick.getRawButton(1);
		reverseIntakeMotors = mainJoystick.getRawButton(2);
		
		easingValue = 0.1;
		if (activateIntakeMotors) {
			intakeOutputPower = interpolateValues(intakeMotorPower, intakeOutputPower);
		} else if(reverseIntakeMotors) {
			easingValue = 0.5;
			intakeOutputPower = interpolateValues(-0.8, intakeOutputPower);
		} else {
			intakeOutputPower = 0;
		}
		
		if((activateIntakeMotors || fireBlockSlow) && mechanismMotors.IntakeMotorOverdraw()) {
			intakeOutputPower = interpolateValues(Math.signum(intakeOutputPower) * overdrawReducePower, intakeOutputPower);
		}
		
		mechanismMotors.RunIntakeMotors(-intakeOutputPower);
		
	}
	
}
