package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoQuery {
	DriverStation driverStation;
	
	public char switchSide;
	public char scaleSide;
	public char enemySwitchSide;
	
	public AutoQuery() {
		driverStation = driverStation.getInstance();
		RobotLogger.toast("driver station INSTANCE match #: " + driverStation.getMatchNumber());
	}
	
	public char[] getData() {
		char[] dataBuffer = new char[3];
		
		String dataString = driverStation.getGameSpecificMessage();
		
		if(dataString.length() == 3) {
			RobotLogger.toast("driver station RECV data: " + dataString, RobotLogger.WARNING);
			dataBuffer[0] = dataString.charAt(0);
			dataBuffer[1] = dataString.charAt(1);
			dataBuffer[2] = dataString.charAt(2);
			
			switchSide = dataBuffer[0];
			scaleSide = dataBuffer[1];
			enemySwitchSide = dataBuffer[2];
		} else {
			RobotLogger.toast("driver station ABNORMAL data: " + dataString, RobotLogger.URGENT);
			RobotLogger.toast("driver station, auto, defaulting to ALL LEFT", RobotLogger.URGENT);
			
			dataBuffer[0] = 'L';
			dataBuffer[1] = 'L';
			dataBuffer[2] = 'L';
			
			switchSide = 'L';
			scaleSide = 'L';
			enemySwitchSide = 'L';
		}
		
		return dataBuffer;
	}
}
