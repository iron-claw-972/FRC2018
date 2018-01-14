package org.usfirst.frc.team972.robot.ui;

import edu.wpi.first.wpilibj.*;

public class Sensors {
	Encoder leftSideEncoderDriveTrain;
	Encoder rightSideEncoderDriveTrain;
	
	DigitalInput frontIntakeOpticalSensor;
	DigitalInput backIntakeOpticalSensor;
	
	public void SetupEncoderDriveTrain(int l1, int l2, int r1, int r2) {
		leftSideEncoderDriveTrain = new Encoder(l1, l2);
		rightSideEncoderDriveTrain = new Encoder(r1, r2);
		
		leftSideEncoderDriveTrain.setDistancePerPulse(1);
		rightSideEncoderDriveTrain.setDistancePerPulse(1);
	}
	
	public void resetDriveEncoder() {
		leftSideEncoderDriveTrain.reset();
		rightSideEncoderDriveTrain.reset();
	}
	
	public int getLeftDriveEncoder() {
		return leftSideEncoderDriveTrain.get();
	}
	
	public int getRightDriveEncoder() {
		return rightSideEncoderDriveTrain.get();
	}
	
	public void SetupIntakeSensors(int frontSensorPort, int backSensorPort) {
		frontIntakeOpticalSensor = new DigitalInput(frontSensorPort);
		backIntakeOpticalSensor = new DigitalInput(backSensorPort);
	}
	
	public boolean getFrontIntakeSensorValue() {
		return frontIntakeOpticalSensor.get();
	}
	
	public boolean getBackIntakeSensorValue() {
		return backIntakeOpticalSensor.get();
	}
}
