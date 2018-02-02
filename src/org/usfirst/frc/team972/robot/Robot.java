package org.usfirst.frc.team972.robot;

import org.usfirst.frc.team972.robot.executor.IntakeSystemTask;
import org.usfirst.frc.team972.robot.executor.TaskExecutor;
import org.usfirst.frc.team972.robot.executor.TeleopArcadeDriveTask;
import org.usfirst.frc.team972.robot.executor.TeleopTankDriveTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoDrivePositionAngle;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveSimpleTime;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveVelocityProfileTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoQuery;
import org.usfirst.frc.team972.robot.executor.auto.AutoTurnAngleTask;
import org.usfirst.frc.team972.robot.executor.auto.TrajectoryExecutionTask;
import org.usfirst.frc.team972.robot.motionlib.Point;
import org.usfirst.frc.team972.robot.motionlib.PointsPath;
import org.usfirst.frc.team972.robot.motionlib.SplineGeneration;
import org.usfirst.frc.team972.robot.motionlib.Trajectory;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	//PDP: 30
	//PCM: 40
	
	public static final double REAL_TIME_LOOP_HZ = 200;
	public static final double MOTION_DT = 50;

	TaskExecutor taskExecutor = new TaskExecutor();
	MainDriveTrain driveTrain = new MainDriveTrain();
	MechanismActuators mechanismMotors = new MechanismActuators();
	Sensors sensors = new Sensors();
	AHRS ahrs;
	
	AutoQuery autoQuery;

	UserInputGamepad uig = new UserInputGamepad(0, 1);

	boolean firstTimeTeleop = false;
	double realStartTime = 0;

	public void robotInit() {
		RobotLogger.toast("Robot Init");
		
		autoQuery = new AutoQuery();
		ahrs = (AHRS) sensors.createAHRS();
		sensors.SetupEncoderDriveTrain(2, 3, 0, 1);
		
		//sensors.SetupIntakeSensors(0, 1);
		// mechanismMotors.SetupIntakeMotors(1, 3);
		
		driveTrain.SetupProcedure(1, 2, 3, 
								  4, 5, 6);// fill this out when we have our
		
		// driveTrain.SetupShift(0, 1);
		driveTrain.setTalonsPWM_follow();
		//driveTrain.setTalonsBrake();
		
		AutoPicker.setup();
	}

	public void autonomousInit() {
		RobotLogger.toast("Auto Init");
		
		sensors.resetDriveEncoders();
		ahrs.reset();
		ahrs.resetDisplacement();
		
		driveTrain.diagnosis();

		realStartTime = Timer.getFPGATimestamp();
		autoQuery.getData(); // retrieve the game data

		RobotLogger.toast("Begin Trajectory Generation");
		Trajectory splineTrajectory = new Trajectory(0);
		try {
			// FileInput.serializeSplineTraj(splineTrajectory, "test_route_1");
			String selectedFile = AutoPicker.selectFile(autoQuery);
			if(selectedFile != null) {
				RobotLogger.toast("Performing Auto File: " + selectedFile);
				splineTrajectory = FileInput.deserializeSplineTraj(selectedFile);
			} else {
				RobotLogger.toast("No Auto File!!!", RobotLogger.URGENT);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}

		RobotLogger.toast("Trajectory Generation Finished. Prepare for Execution");

		TrajectoryExecutionTask follower = new TrajectoryExecutionTask(0, driveTrain, sensors, ahrs);

		taskExecutor.addTask(new AutoDriveVelocityProfileTask(0,
				SplineGeneration.generateWheelTrajectories(splineTrajectory, 0.6096), sensors, follower, splineTrajectory));
		taskExecutor.addTask(follower);
		
		taskExecutor.autonomousStart();

		//TODO: spawn new thread to execute the realtime loop in
		autoRealTimeControlLoop();
	}

	// this control loop runs at 200hz, DO NOT ADD INTO THE CONTROL LOOP WITHOUT
	// ASKING JODY PLEASE (needs real time capabilities)
	public void autoRealTimeControlLoop() {
		while (this.isEnabled() && this.isAutonomous()) {
			double current_time = Timer.getFPGATimestamp() - realStartTime;
			taskExecutor.executeDT(current_time);
			try {
				Thread.sleep((long) (1000 / REAL_TIME_LOOP_HZ));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		taskExecutor.stop();
	}

	public void autonomousPeriodic() {
		// do nothing because all of our auto is done in the real time control loop
	}

	public void teleopInit() {
		RobotLogger.toast("Teleop Init");
		driveTrain.setTalonsPWM_follow();
		driveTrain.diagnosis();

		sensors.resetDriveEncoder();
		
		ahrs.reset();
		ahrs.resetDisplacement();
		
		taskExecutor.addTask(new TeleopArcadeDriveTask(0, uig, driveTrain, ahrs, sensors));
		// taskExecutor.addTask(new IntakeSystemTask(0, uig, mechanismMotors, sensors));
		taskExecutor.teleopStart();

		// EasyTeleop.teleopInit();
	}

	public void disabledPeriodic() {
		taskExecutor.stop();
		//driveTrain.stopCoast();
	}
	
	public void teleopPeriodic() {
		double current_time = Timer.getFPGATimestamp() - realStartTime;

		taskExecutor.executeDT(current_time);

		// EasyTeleop.teleopPeriodic();
	}

	public void testPeriodic() {

	}

}
