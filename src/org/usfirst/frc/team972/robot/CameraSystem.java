package org.usfirst.frc.team972.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore. VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;

public class CameraSystem {
	
	static boolean cameraSwitch = false;
	
	static UsbCamera cam0;
	static UsbCamera cam1;
	static MjpegServer server;
	
	public static void startCamera() {
		RobotLogger.toast("Spooling up MPJEG Camera Server");
		cam0 = new UsbCamera("cam0", 0);
		cam1 = new UsbCamera("cam1", 1);
		
		server = new MjpegServer("server", 1181);
		server.setSource(cam0);
	}
	
	public static void teleopUpdate(Joystick joy) {
		if(joy.getRawButtonPressed(7)) {
			cameraSwitch = !cameraSwitch;
		}
		if(cameraSwitch) {
			server.setSource(cam0);
		} else {
			server.setSource(cam1);
		}
	}
}
