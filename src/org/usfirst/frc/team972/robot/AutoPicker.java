package org.usfirst.frc.team972.robot;

import org.usfirst.frc.team972.robot.executor.auto.AutoQuery;

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
		modeChooser.addDefault("Do SCALE or SWITCH if on SAME SIDE", "side_any");
		
		overrideMode = new SendableChooser();
		overrideMode.addDefault("NO override", "no_override");
		FileInput.addChoosers(overrideMode);
		
		SmartDashboard.putData("Side Choose", sideChooser);
		SmartDashboard.putData("Mode Chooser", modeChooser);
		SmartDashboard.putData("Override Chooser (Testing Use Only!)", overrideMode);
	}
	
	public static String getOverrideSelected() {
		return (String) overrideMode.getSelected();
	}

	public static String selectFile(AutoQuery query) {
		String side = (String)sideChooser.getSelected();
		String mode = (String)modeChooser.getSelected();
		String override = (String)overrideMode.getSelected();
		
		if(override.equals("no_override")) {
			if(mode.equals("side_switch")) {
				if((side.equals("right")) && (query.switchSide == 'R')) {
					return "right_to_right_switch";
				} else if((side.equals("left")) && (query.switchSide == 'L')) {
					return "left_to_left_switch";
				} else if((side.equals("center"))) {
					if(query.switchSide == 'L') {
						return "center_to_left_switch";
					} else if (query.switchSide == 'R') {
						return "center_to_right_switch";
					} else {
						RobotLogger.toast("Auto Side Failure, Start from Center: " + query.switchSide);
					}
				} else {
					RobotLogger.toast("Auto Side Failure: " + side + " " + query.switchSide);
					return null;
				} 
			} else if (mode.equals("side_scale")) {
				if((side.equals("right")) && (query.scaleSide == 'R')) {
					return "right_to_right_scale";
				} else if((side.equals("left")) && (query.scaleSide == 'L')) {
					return "left_to_left_scale";
				} else {
					RobotLogger.toast("Auto Scale Failure: " + side + " " + query.scaleSide);
					return null;
				}
			} else if (mode.equals("side_any")) {
				if((side.equals("right")) && (query.switchSide == 'R')) {
					return "right_to_right_switch";
				} else if((side.equals("left")) && (query.switchSide == 'L')) {
					return "left_to_left_switch";
				} else if((side.equals("right")) && (query.scaleSide == 'R')) {
					return "right_to_right_scale";
				} else if((side.equals("left")) && (query.scaleSide == 'L')) {
					return "left_to_left_scale";
				} else {
					RobotLogger.toast("No Scale/Switch on side, but want to do any. Performing Scale Defense: " + side);
					
					return side + "_scale_defense";
				}
			}
					
		} else {
			return override;
		}
		
		return null;
	}
	
}