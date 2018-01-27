package org.usfirst.frc.team972.robot.ui;

import edu.wpi.first.wpilibj.Joystick;

public class UserInputGamepad {
	Joystick stick_a;
	Joystick stick_b;
	
	
	public UserInputGamepad(int stick_a_id, int stick_b_id) {
		stick_a = new Joystick(stick_a_id);
		stick_b = new Joystick(stick_b_id);
	}
	
	public Joystick getStickA() {
		return stick_a;
	}
	
	public Joystick getStickB() {
		return stick_b;
	}
}
