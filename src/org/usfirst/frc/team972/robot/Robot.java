package org.usfirst.frc.team972.robot;

import org.usfirst.frc.team972.robot.executor.IntakeSystemTask;
import org.usfirst.frc.team972.robot.executor.TaskExecutor;
import org.usfirst.frc.team972.robot.executor.TeleopTankDriveTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveSimpleTime;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveVelocityProfileTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoQuery;
import org.usfirst.frc.team972.robot.motionlib.Point;
import org.usfirst.frc.team972.robot.motionlib.PointsPath;
import org.usfirst.frc.team972.robot.motionlib.SplineGeneration;
import org.usfirst.frc.team972.robot.motionlib.Trajectory;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	static final double REAL_TIME_LOOP_HZ = 50;
	
	TaskExecutor taskExecutor = new TaskExecutor();
	MainDriveTrain driveTrain = new MainDriveTrain();
	MechanismActuators mechanismMotors = new MechanismActuators();
	Sensors sensors = new Sensors();
	
	AutoQuery autoQuery;

	UserInputGamepad uig = new UserInputGamepad(0);

	boolean firstTimeTeleop = false;
	double realStartTime = 0;
	
	public void robotInit() {
		RobotLogger.toast("Robot Init");
		autoQuery = new AutoQuery();
		sensors.SetupIntakeSensors(0,1);
		//mechanismMotors.SetupIntakeMotors(1, 3); //This creates two motors for the left and right motors of our intake mechanism
		driveTrain.SetupProcedure(1, 0, 0, 3, 0, 0);// fill this out when we have our
		//driveTrain.SetupShift(0, 1);
		driveTrain.setTalonsPWM_follow();
		driveTrain.setTalonsBrake();
		// cantalons left-right
	}

	public void autonomousInit() {
		RobotLogger.toast("Auto Init");
		driveTrain.diagnosis();
		
		realStartTime = Timer.getFPGATimestamp();
		autoQuery.getData(); // retrieve the game data

		/*taskExecutor.addTask(new AutoDriveSimpleTime(0  , 2, .321, driveTrain));
		taskExecutor.addTask(new AutoDriveSimpleTime(2, 1, .254, driveTrain));
		taskExecutor.addTask(new AutoDriveSimpleTime(4, 2, 0, driveTrain));
		taskExecutor.addTask(new AutoDriveSimpleTime(6, 1, 1.0, driveTrain));
		*/
		
        PointsPath pointsPath = new PointsPath();
        pointsPath.addPoint(new Point(0, 0, 0));
        pointsPath.addPoint(new Point(10, 0, 0));
        
        RobotLogger.toast("Begin Trajectory Generation");
        Trajectory splineTrajectory = SplineGeneration.generateSpline(0.5, 0.25, pointsPath, (1000/REAL_TIME_LOOP_HZ/1000));
        RobotLogger.toast("Trajectory Generation Finished");
        
        taskExecutor.addTask(new AutoDriveVelocityProfileTask(0, SplineGeneration.generateWheelTrajectories(splineTrajectory, 1), sensors, driveTrain));
		
		taskExecutor.autonomousStart();
		
		autoRealTimeControlLoop();
	}

	//this control loop runs at 200hz, DO NOT ADD INTO THE CONTROL LOOP WITHOUT ASKING JODY PLEASE (needs real time capabilities)
	public void autoRealTimeControlLoop() {
		while(this.isEnabled() && this.isAutonomous()) {
			double current_time = Timer.getFPGATimestamp() - realStartTime;
			
			taskExecutor.executeDT(current_time);
			try {
			Thread.sleep((long)(1000/REAL_TIME_LOOP_HZ));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
	
	public void autonomousPeriodic() {
		//do nothing because all of our auto is done in the real time control loop
	}

	public void teleopInit() {
		RobotLogger.toast("Teleop Init");
		driveTrain.diagnosis();
		
		taskExecutor.addTask(new TeleopTankDriveTask(0, uig, driveTrain));
		//taskExecutor.addTask(new IntakeSystemTask(0, uig, mechanismMotors, sensors));
		taskExecutor.teleopStart();
		
		//EasyTeleop.teleopInit();
	}

	public void teleopPeriodic() {
		double current_time = Timer.getFPGATimestamp() - realStartTime;

		taskExecutor.executeDT(current_time);
		
		//EasyTeleop.teleopPeriodic();
	}

	public void testPeriodic() {

	}

}
