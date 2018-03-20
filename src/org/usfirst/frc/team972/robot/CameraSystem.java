package org.usfirst.frc.team972.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore. VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraSystem {
	public static void startCamera() {
		RobotLogger.toast("Spooling up Camera Server");
		UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture();
		//UsbCamera cam2 = CameraServer.getInstance().startAutomaticCapture(1);
		/*
		if(cam1.setFPS(15)) {
			RobotLogger.toast("Camera Config Success");
		} else {
			RobotLogger.toast("Camera Fail Config!", RobotLogger.URGENT);
		}
		*/
	}
}
