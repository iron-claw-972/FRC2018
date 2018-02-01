package org.usfirst.frc.team972.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPicker {
	public static SendableChooser sideChooser;
	public static SendableChooser modeChooser;
	public static SendableChooser overrideMode;
	
	public static void setup() {
		sideChooser = new SendableChooser();
		sideChooser.addDefault("Start Center Side", "center");
		sideChooser.addObject("Start Right Side", "right");
		sideChooser.addObject("Start Left Side", "left");
		
		modeChooser = new SendableChooser();
		
		modeChooser.addDefault("Do Switch Any", "any_switch");
		modeChooser.addDefault("Do Switch If On Same Side", "side_switch");
		modeChooser.addDefault("Do Scale Any", "any_scale");
		modeChooser.addDefault("Do Scale If On Same Side", "side_scale");
		
		overrideMode = new SendableChooser();
		overrideMode.addDefault("Nothing", "");
		FileInput.addChoosers(overrideMode);
		
		SmartDashboard.putData("Side Choose", sideChooser);
		SmartDashboard.putData("Mode Chooser", modeChooser);
		SmartDashboard.putData("Override Chooser (Testing Use Only!)", overrideMode);
	}
	
	public static String getOverrideSelected() {
		return (String) overrideMode.getSelected();
	}
	
}