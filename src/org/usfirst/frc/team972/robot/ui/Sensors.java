package org.usfirst.frc.team972.robot.ui;

import org.usfirst.frc.team972.robot.RobotLogger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;

public class Sensors {
	AHRS ahrs;
	
	Encoder leftSideEncoderDriveTrain;
	Encoder rightSideEncoderDriveTrain;
	Encoder elevatorEncoder;
	
	DigitalInput frontIntakeOpticalSensor;
	DigitalInput backIntakeOpticalSensor;
	
	public void SetupEncoderDriveTrain(int l1, int l2, int r1, int r2) {
		leftSideEncoderDriveTrain = new Encoder(l1, l2);
		rightSideEncoderDriveTrain = new Encoder(r1, r2);
		
		leftSideEncoderDriveTrain.setDistancePerPulse(1);
		rightSideEncoderDriveTrain.setDistancePerPulse(1);
	}
	
	public void SetupEncoderElevator(int a, int b) {
		elevatorEncoder = new Encoder(a, b);
		elevatorEncoder.setDistancePerPulse(1);
	}
	
	public void resetDriveEncoder() {
		leftSideEncoderDriveTrain.reset();
		rightSideEncoderDriveTrain.reset();
	}
	
	public int getElevatorEncoder() {
		return elevatorEncoder.get();
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
	
	public void resetElevatorEncoder() {
		elevatorEncoder.reset();
	}
	
	public void resetDriveEncoders() {
		leftSideEncoderDriveTrain.reset();
		rightSideEncoderDriveTrain.reset();
	}

	public AHRS createAHRS() {
		RobotLogger.toast("Preparing to obtain the AHRS");
		try {
			ahrs = new AHRS(SPI.Port.kMXP);
			if (ahrs.isConnected()) {
				RobotLogger.toast("AHRS Success");
			} else {
				RobotLogger.toast("AHRS Not Connected");
			}
		} catch(Exception e) {
			RobotLogger.toast("Failed to obtain the AHRS! " + e.getMessage(), RobotLogger.URGENT);
		}
		return ahrs;
	}
}
