package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test {
	
	MainDriveTrain driveTrain;
	MechanismActuators mechanismMotors;
	SendableChooser<WPI_TalonSRX> chooser = new SendableChooser<WPI_TalonSRX>();
	
	public Test(MainDriveTrain _driveTrain, MechanismActuators _mechanismMotors) {
		driveTrain = _driveTrain;
		mechanismMotors = _mechanismMotors;
	}

	public void init() {
		for (int i = 0; i < driveTrain.talons.length; i++) {
			chooser.addObject("Drive Motor" + i, driveTrain.talons[i]);
		}
		chooser.addObject("intakeMotorLeft", mechanismMotors.intakeMotorLeft);
		chooser.addObject("intakeMotorRight", mechanismMotors.intakeMotorRight);
		SmartDashboard.putData("Motor Chooser", chooser);
		
	}

	public void periodic() {
		if(chooser.getSelected() != null) {
			chooser.getSelected().set(0.5);	
		}
	}
	
}
