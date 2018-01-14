package org.usfirst.frc.team972.robot.ui;

import edu.wpi.first.wpilibj.Joystick;

public class UserInputGamepad {
	Joystick stick_a;
	
	public UserInputGamepad(int stick_a_id) {
		stick_a = new Joystick(stick_a_id);
	}
	
	public Joystick getStickA() {
		return stick_a;
	}
}
