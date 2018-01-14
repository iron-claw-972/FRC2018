package org.usfirst.frc.team972.robot;

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
			System.out.println("[ironclaw_logs][WARNING]: " + message);
		} else if(type == URGENT) {
			System.out.println("**URGENT MESSAGE**");
			System.out.println("**URGENT** [ironclaw_logs] **URGENT** : " + message);
			System.out.println("**URGENT MESSAGE**");
		}
	}
}
