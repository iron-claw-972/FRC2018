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
	double intakeMotorPower = 0.5;
	boolean activateIntakeMotors;
	boolean reverseIntakeMotors;
	boolean frontIntakeSensorValue;
	boolean backIntakeSensorValue;
	
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

	@Override //test
	public void execute(double dt) {
		Joystick mainJoystick = uig.getStickA();
		activateIntakeMotors = mainJoystick.getRawButton(1);
		reverseIntakeMotors = mainJoystick.getRawButton(2);
		frontIntakeSensorValue = sensors.getFrontIntakeSensorValue();
		backIntakeSensorValue = sensors.getBackIntakeSensorValue();
		
		if (activateIntakeMotors && !(backIntakeSensorValue && frontIntakeSensorValue)) {
			mechanismMotors.RunIntakeMotors(intakeMotorPower);
		} else if(reverseIntakeMotors) {
			mechanismMotors.RunIntakeMotors(-intakeMotorPower);
		} else {
			mechanismMotors.RunIntakeMotors(0);
		}
		
	}
	
}
