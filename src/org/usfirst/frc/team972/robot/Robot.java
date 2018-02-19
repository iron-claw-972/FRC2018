package org.usfirst.frc.team972.robot;

import org.usfirst.frc.team972.robot.executor.IntakeSystemTask;
import org.usfirst.frc.team972.robot.executor.TaskExecutor;
import org.usfirst.frc.team972.robot.executor.TeleopArcadeDriveTask;
import org.usfirst.frc.team972.robot.executor.TeleopElevatorTask;
import org.usfirst.frc.team972.robot.executor.TeleopTankDriveTask;
import org.usfirst.frc.team972.robot.executor.TeleopWinchTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoDrivePositionAngle;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveSimpleTime;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveVelocityProfileTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoPathRoutines;
import org.usfirst.frc.team972.robot.executor.auto.AutoQuery;
import org.usfirst.frc.team972.robot.executor.auto.AutoTurnAngleTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlFlopTask;
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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	//PDP: 30
	//PCM: 40

	public static final double WHEEL_BASE_WIDTH = 0.6096;
	public static final double REAL_TIME_LOOP_HZ = 200;
	public static final double MOTION_DT = 50;

	AutoQuery autoQuery;
	
	TaskExecutor taskExecutor = new TaskExecutor();
	MainDriveTrain driveTrain = new MainDriveTrain();
	MechanismActuators mechanismMotors = new MechanismActuators();
	Sensors sensors = new Sensors();
	AHRS ahrs;
	
	UserInputGamepad uig = new UserInputGamepad(0, 1);

	AutoPathRoutines autoRoutine;
	
	boolean firstTimeTeleop = false;
	double realStartTime = 0;

	public void robotInit() {
		RobotLogger.toast("Robot Init");
		
		autoQuery = new AutoQuery();
		ahrs = (AHRS) sensors.createAHRS();
		sensors.SetupEncoderDriveTrain(2, 3, 0, 1);
		
		sensors.SetupIntakeSensors(5);
		mechanismMotors.SetupIntakeMotors(2, 3);
		
		mechanismMotors.SetupWinchMotors(11);
		
		sensors.SetupEncoderElevator(mechanismMotors.SetupElevatorLiftMotor(10));
		
		mechanismMotors.SetupElevatorFlopMotor(1);
		sensors.SetupEncoderFlop(6, 7);
		
		driveTrain.SetupProcedure(4, 5, 6, 
								  7, 8, 9);

		//driveTrain.SetupShift(40, 0, 1);
		driveTrain.setTalonsPWM_follow();
		//driveTrain.setTalonsBrake();
		
		AutoPicker.setup();
		CameraSystem.startCamera();

		autoRoutine = new AutoPathRoutines(taskExecutor,
				autoQuery, driveTrain, sensors, ahrs, mechanismMotors);
	}

	public void autonomousInit() {
		RobotLogger.toast("Auto Init");
		
		sensors.resetDriveEncoders();
		sensors.resetElevatorEncoder();
		sensors.resetFlopEncoder();
		
		ahrs.reset();
		ahrs.resetDisplacement();
		
		driveTrain.diagnosis();
		driveTrain.shiftSolenoidDown();
		 	
		realStartTime = Timer.getFPGATimestamp();
		autoQuery.getData(); // retrieve the game data
		
		if(autoRoutine.pickRoutine()) {
			RobotLogger.toast("Routine Picked Success: Starting Auto");
			taskExecutor.autonomousStart();
		} else {
			RobotLogger.toast("Failed Routine Pick: Do Nothing");
		}
		
		autoRealTimeControlLoop();
	}

	public void autonomousPeriodic() {
		// do nothing because all of our auto is done in the real time control loop
	}

	public void teleopInit() {
		sensors.resetDriveEncoders();
		
		// in a real match, we do not want to zero in teleop
		//sensors.resetElevatorEncoder();
		//sensors.resetFlopEncoder();
		
		RobotLogger.toast("Teleop Init");
		
		new Compressor(40).start();
		
		driveTrain.setTalonsPWM_follow();
		driveTrain.diagnosis();
		driveTrain.shiftSolenoidDown();

		sensors.resetDriveEncoder();
		
		ahrs.reset();
		ahrs.resetDisplacement();
		
		ControlFlopTask flopControl = new ControlFlopTask(0, mechanismMotors, sensors);
		ControlElevatorTask elevatorControl = new ControlElevatorTask(0, mechanismMotors, sensors, flopControl);

		elevatorControl.realtimeTask = true;
		flopControl.realtimeTask = true;
		
		taskExecutor.addTask(new TeleopArcadeDriveTask(0, uig, driveTrain, ahrs, sensors));
		taskExecutor.addTask(new TeleopElevatorTask(0, uig, mechanismMotors, elevatorControl, flopControl));
		taskExecutor.addTask(elevatorControl);
		taskExecutor.addTask(flopControl);
		taskExecutor.addTask(new IntakeSystemTask(0, uig, mechanismMotors, sensors));
		taskExecutor.addTask(new TeleopWinchTask(0, uig, mechanismMotors));
		taskExecutor.teleopStart();
		
		realStartTime = Timer.getFPGATimestamp();
		
		teleopRealTimeController(); //begin 200hz control loop
	}

	public void disabledPeriodic() {
		taskExecutor.stop();
		taskExecutor.forceClearTasks();
		driveTrain.stopCoast();
	}
	
	public void teleopPeriodic() {
		//do nothing
	}
	
	public void autoRealTimeControlLoop() {
		while (this.isEnabled() && this.isAutonomous()) {
			double current_time = Timer.getFPGATimestamp() - realStartTime;
			taskExecutor.executeDT(current_time, false); //dont care about realtime locks, just run at full speed
			try {
				Thread.sleep((long) (1000 / REAL_TIME_LOOP_HZ));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		taskExecutor.stop();
	}

	public void teleopRealTimeController() {
		while (this.isEnabled() && this.isOperatorControl()) {
			double current_time = Timer.getFPGATimestamp() - realStartTime;
			if(this.isNewDataAvailable()) {
				taskExecutor.executeDT(current_time, false);
			} else {
				taskExecutor.executeDT(current_time, true);
			}
			
			try {
				Thread.sleep((long) (1000 / REAL_TIME_LOOP_HZ));
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		taskExecutor.stop();
	}
	
	public void testPeriodic() {

	}

}
