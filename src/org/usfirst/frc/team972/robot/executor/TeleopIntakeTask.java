package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopIntakeTask extends Task{
	
	MechanismActuators mechanismMotors;
	UserInputGamepad uig;
	Sensors sensors;
	
	double easingValue = 0.1;
	double intakeMotorPower = 0.5;
	double intakeShootingPower = -0.5;
	double overdrawReducePower = 0.1;
	
	double intakeOutputPower = 0;
	
	boolean activateIntakeMotors;
	boolean reverseIntakeMotors;
	boolean frontIntakeSensorValue;
	boolean backIntakeSensorValue;
	boolean fireBlockSlow;
	
	char mode = 'N';
	
	public TeleopIntakeTask(double _executionTime, UserInputGamepad _uig, MechanismActuators _mechanismMotors, Sensors _sensors) {
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
		double holdingThrottle = (1 - mainJoystick.getRawAxis(2));
		
		if(activateIntakeMotors) {
			mode = 'I';
		} else if(reverseIntakeMotors) {
			mode = 'R';
		} else {
			mode = 'N';
		}
		
		easingValue = 0.1; //reset to regular easing
		
		if (mode == 'I') {
			intakeOutputPower = interpolateValues(intakeMotorPower, intakeOutputPower);
		} else if(mode == 'R') {
			easingValue = 0.2; //we want to shoot blocks out
			intakeOutputPower = interpolateValues(intakeShootingPower, intakeOutputPower); 
		} else if(holdingThrottle > 0.5) {
			intakeOutputPower = (holdingThrottle * 0.118);
		} else {
			intakeOutputPower = 0;
		}
		
		if (((mode == 'I') || (mode == 'H')) && mechanismMotors.IntakeMotorOverdraw()) {
			intakeOutputPower = interpolateValues(Math.signum(intakeOutputPower) * overdrawReducePower, intakeOutputPower);
		}

		mechanismMotors.RunIntakeMotors(-intakeOutputPower);
	}
	
}
