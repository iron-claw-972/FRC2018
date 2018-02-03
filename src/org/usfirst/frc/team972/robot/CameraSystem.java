package org.usfirst.frc.team972.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore. VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;

public class CameraSystem {
	public static void startCamera() {
		RobotLogger.toast("Spooling up Camera Server");
		UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(0);
		UsbCamera cam2 = CameraServer.getInstance().startAutomaticCapture(1);
		if(cam1.setFPS(30) && cam2.setFPS(30)) {
			RobotLogger.toast("Camera Config Success");
		} else {
			RobotLogger.toast("Camera Fail Config!", RobotLogger.URGENT);
		}
	}
}
