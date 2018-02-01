package org.usfirst.frc.team972.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStreamWriter;
import java.nio.file.Files;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.util.Scanner;

import org.usfirst.frc.team972.robot.motionlib.Trajectory;
import org.usfirst.frc.team972.robot.motionlib.Trajectory.Segment;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class FileInput {
	
	final static String homeDir = "/home/lvuser/routes/";
	
	public String getMD5(File file) throws Exception {
		 MessageDigest md = MessageDigest.getInstance("MD5");
		 InputStream is = Files.newInputStream(file.toPath());
		 DigestInputStream dis = new DigestInputStream(is, md); 

		 byte[] digest = md.digest();
		 return digest.toString();
	}
	
	public static void addChoosers(SendableChooser chooser) {
		File dir = new File(homeDir);
		File[] listFiles = dir.listFiles();
		
		for(int i=0; i<listFiles.length; i++) {
			chooser.addObject(listFiles[i].getName(), listFiles[i].getName());
		}
	}
	
	public static Trajectory deserializeSplineTraj(String filename) throws Exception {
		RobotLogger.toast("Begin DeSerialization of Trajectory: " + filename);
		Scanner apCsLab = new Scanner(new File(homeDir + filename));
		
		int trajLength = 0;
		while(apCsLab.hasNextLine()) {
			apCsLab.nextLine();
			trajLength++;
		}
		
		Trajectory traj = new Trajectory(trajLength);

		Scanner scan = new Scanner(new File(homeDir + filename));
		
		int index = 0;
		
		while(scan.hasNextLine()) {
			String line = scan.nextLine();
			String elements[] = line.split(" ");
			//bw.write(seg.pos + " " + seg.vel + " " + seg.acc + " " + seg.dt + " " + seg.heading + " " + seg.jerk + " " + seg.x + " " + seg.y);
			double pos = Double.parseDouble(elements[0]);
			double vel = Double.parseDouble(elements[1]);
			double acc = Double.parseDouble(elements[2]);
			double dt = Double.parseDouble(elements[3]);
			double heading = Double.parseDouble(elements[4]);
			double jerk = Double.parseDouble(elements[5]);
			double x = Double.parseDouble(elements[6]);
			double y = Double.parseDouble(elements[7]);
			
			Segment seg = new Segment();
			seg.pos = pos;
			seg.vel = vel;
			seg.acc = acc;
			seg.dt = dt;
			seg.heading = heading;
			seg.jerk = jerk;
			seg.x = x;
			seg.y = y;
			
			traj.setSegment(index, seg);
			index++;
		}
		
        RobotLogger.toast("Trajectory de-serialized!");
            
        return traj;
	}
	
	public static void serializeSplineTraj(Trajectory traj, String filename) {
		try {
			RobotLogger.toast("Begin Serialization of Trajectory: " + filename);
			File realFile = new File(homeDir + filename);
			RobotLogger.toast("Creating New File: " + realFile.getAbsolutePath());
			realFile.createNewFile();

			FileOutputStream fos = new FileOutputStream(realFile); 
			BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(fos)); 
			
			for(int i=0; i<traj.getNumSegments(); i++) {
				Segment seg = traj.getSegment(i);
				bw.write(seg.pos + " " + seg.vel + " " + seg.acc + " " + seg.dt + " " + seg.heading + " " + seg.jerk + " " + seg.x + " " + seg.y);
				bw.write('\n');
			}
			
			bw.close();
		} catch(Exception e) {
			e.printStackTrace();
		}
        RobotLogger.toast("Trajectory has been serialized!");
	}
}
