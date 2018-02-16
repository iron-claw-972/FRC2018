package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotLogger {
	public static final int NORMAL = 0;
	public static final int WARNING = 1;
	public static final int URGENT = 2;
	
	public static void toast(String message) {
		toast(message, NORMAL);
	}
		
	public static void toast(String message, int type) {
		if(type == NORMAL) {
			System.out.println("[ironclaw_logs][norm]: " + message);
		} else if(type == WARNING) {
			DriverStation.reportWarning("[ironclaw_logs][WARNING]: " + message, false);
		} else if(type == URGENT) {
			System.out.println("**URGENT MESSAGE**");
			DriverStation.reportError("**URGENT** [ironclaw_logs] **URGENT** : " + message, false);
			System.out.println("**URGENT MESSAGE**");
		}
	}
}
