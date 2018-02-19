package org.usfirst.frc.team972.robot.ui;

import org.usfirst.frc.team972.robot.RobotLogger;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;

public class Sensors {
	AHRS ahrs;
	
	Encoder leftSideEncoderDriveTrain;
	Encoder rightSideEncoderDriveTrain;
	Encoder flop;
	
	WPI_TalonSRX elevatorTalon;

	
	DigitalInput frontIntakeOpticalSensor;
	//DigitalInput backIntakeOpticalSensor;
	
	public void SetupEncoderDriveTrain(int l1, int l2, int r1, int r2) {
		leftSideEncoderDriveTrain = new Encoder(l1, l2);
		rightSideEncoderDriveTrain = new Encoder(r1, r2);
		
		leftSideEncoderDriveTrain.setDistancePerPulse(1);
		rightSideEncoderDriveTrain.setDistancePerPulse(1);
	}
	
	public void SetupEncoderElevator(WPI_TalonSRX _elevatorTalon) {
		elevatorTalon = _elevatorTalon;
		elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevatorTalon.setSensorPhase(false);
	}
	
	public void resetDriveEncoder() {
		leftSideEncoderDriveTrain.reset();
		rightSideEncoderDriveTrain.reset();
	}
	
	public int getElevatorEncoder() {
		//System.out.println("e:"+elevatorTalon.getSelectedSensorPosition(0));
		return elevatorTalon.getSelectedSensorPosition(0);
	}
	
	public int getFlopEncoder() {
		return flop.get();
	}
	
	public int getLeftDriveEncoder() {
		return leftSideEncoderDriveTrain.get();
	}
	
	public int getLeftDriveEncoderSpeed() {
		return (int)leftSideEncoderDriveTrain.getRate();
	}
	
	public int getRightDriveEncoder() {
		return rightSideEncoderDriveTrain.get();
	}
	
	public void SetupIntakeSensors(int frontSensorPort) {
		frontIntakeOpticalSensor = new DigitalInput(frontSensorPort);
		//backIntakeOpticalSensor = new DigitalInput(backSensorPort);
	}
	
	public boolean getFrontIntakeSensorValue() {
		return !frontIntakeOpticalSensor.get();
	}
	
	/*
	public boolean getBackIntakeSensorValue() {
		//return backIntakeOpticalSensor.get();
	}*/
	
	public void resetElevatorEncoder() {
		elevatorTalon.setSelectedSensorPosition(0, 0, 0);
	}
	public void resetFlopEncoder() {
		flop.reset();
	}
	
	public void resetDriveEncoders() {
		leftSideEncoderDriveTrain.reset();
		rightSideEncoderDriveTrain.reset();
	}

	public AHRS createAHRS() {
		RobotLogger.toast("Preparing to obtain the AHRS");
		try {
			ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);
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

	public double getElevatorEncoderVelocity() {
		return elevatorTalon.getSelectedSensorVelocity(0);
	}

	public void SetupEncoderFlop(int a, int b) {
		// TODO Auto-generated method stub
		flop = new Encoder(a, b);
	}
}
