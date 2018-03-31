package org.usfirst.frc.team972.robot;

import org.usfirst.frc.team972.robot.executor.auto.AutoQuery;

import edu.wpi.first.wpilibj.DriverStation;
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
		
		modeChooser.addDefault("Do Switch Regular", "switch");
		modeChooser.addDefault("Do Switch Regular, but do 2 block if we can!", "switch_2_block");

		
		overrideMode = new SendableChooser();
		overrideMode.addDefault("NO override", "no_override");
		overrideMode.addObject("do nothing", "nothing");
		FileInput.addChoosers(overrideMode);
		
		SmartDashboard.putData("Side Choose", sideChooser);
		SmartDashboard.putData("Mode Chooser", modeChooser);
		SmartDashboard.putData("Override Chooser (Testing Use Only!)", overrideMode);
	}
	
	public static String selectFile(AutoQuery query) {
		String side = (String)sideChooser.getSelected();
		String mode = (String)modeChooser.getSelected();
		String override = (String)overrideMode.getSelected();
		
		RobotLogger.toast(side);
		RobotLogger.toast(mode);
		RobotLogger.toast(override);
		
		if(override.equals("no_override")) {
			if(mode.equals("switch") || mode.equals("switch_2_block")) {
				if((side.equals("right")) && (query.switchSide == 'R')) {
					return "right_to_right_switch";
				} else if((side.equals("left")) && (query.switchSide == 'L')) {
					return "left_to_left_switch";
				} else if((side.equals("center"))) {
					if(mode.equals("switch_2_block")) { //2 block
						if(query.switchSide == 'L') {
							return "center_to_left_switch";
						} else if (query.switchSide == 'R') {
							return "center_to_right_switch_2_block";
						} else {
							RobotLogger.toast("FMS Fault!!! Auto Broke during Center" + query.switchSide);
						}
					} else {
						if(query.switchSide == 'L') { //easy block 1 block
							return "center_to_left_switch";
						} else if (query.switchSide == 'R') {
							return "center_to_right_switch";
						} else {
							RobotLogger.toast("FMS Fault!!! Auto Broke during Center" + query.switchSide);
						}
					}
				} else {
					RobotLogger.toast("Auto Side Failure: " + side + " " + query.switchSide);
					return "five_meters_foward";
				}
			}
					
		} else if (override == "nothing") {
			return "nothing";
		}else {
			RobotLogger.toast("Overrided Auto: " + override);
			return override;
		}
		
		return null;
	}
	
}