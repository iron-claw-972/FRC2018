/*
 * tengamos code por la roboto
 * ser usa en 2018
 */

package org.usfirst.frc.team972.robot;

import org.usfirst.frc.team972.robot.executor.TeleopIntakeTask;
import org.usfirst.frc.team972.robot.executor.TaskExecutor;
import org.usfirst.frc.team972.robot.executor.TeleopArcadeDriveTask;
import org.usfirst.frc.team972.robot.executor.TeleopElevatorTask;
import org.usfirst.frc.team972.robot.executor.TeleopIntakeArmTask;
import org.usfirst.frc.team972.robot.executor.TeleopTankDriveTask;
import org.usfirst.frc.team972.robot.executor.TeleopWinchTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoDrivePositionAngle;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveSimpleTime;
import org.usfirst.frc.team972.robot.executor.auto.AutoDriveVelocityProfileTask;
import org.usfirst.frc.team972.robot.executor.auto.AutoPathRoutines;
import org.usfirst.frc.team972.robot.executor.auto.AutoQuery;
import org.usfirst.frc.team972.robot.executor.auto.AutoTurnAngleTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlIntakeArmTask;
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
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	//PDP: 30
	//PCM: 40
	
	public static final double WHEEL_BASE_WIDTH = 0.838;
	public static final double REAL_TIME_LOOP_HZ = 200;

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
	double lastTime = 0;
	
	long controlLoopCycle = 0;
	long moduloUpdateCycle = 200;
	
	public void robotInit() {
		RobotLogger.toast("Robot Init");
		
		autoQuery = new AutoQuery();
		ahrs = (AHRS) sensors.createAHRS();
		sensors.SetupEncoderDriveTrain(2, 3, 0, 1);

		mechanismMotors.SetupIntakeMotors(11, 12);
		
		sensors.SetupEncoderElevator(mechanismMotors.SetupElevatorLiftMotor(1));		

		mechanismMotors.SetupIntakeArmMotors(2, 10);
		sensors.SetupIntake(mechanismMotors.intakeArmMotorRight, mechanismMotors.intakeArmMotorLeft);
		
		driveTrain.SetupProcedure(4, 5, 6, 
								  7, 8, 9);

		driveTrain.SetupShift(40, 0, 1);
		driveTrain.setTalonsPWM_follow();
		
		AutoPicker.setup();

		autoRoutine = new AutoPathRoutines(taskExecutor,
				autoQuery, driveTrain, sensors, ahrs, mechanismMotors);
				
		CameraSystem.startCamera();
	}
	
	public void autonomousInit() {
		RobotLogger.toast("Auto Init");
		
		sensors.resetDriveEncoders();
		sensors.resetElevatorEncoder();
		sensors.resetIntakeEncoders();
		
		ahrs.reset();
		ahrs.resetDisplacement();
		
		driveTrain.diagnosis();
		driveTrain.setTalonsBrake();
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
		// in a real match, we do not want to zero in teleop
		//sensors.resetElevatorEncoder();
		//sensors.resetFlopEncoder();
		
		RobotLogger.toast("Teleop Init");
		
		sensors.resetDriveEncoders();
		new Compressor(40).start();
		
		driveTrain.setTalonsPWM_follow();
		driveTrain.diagnosis();
		driveTrain.shiftSolenoidDown();
		driveTrain.setTalonsBrake();

		ahrs.reset();
		ahrs.resetDisplacement();
		
		ControlElevatorTask elevatorControl = new ControlElevatorTask(0, mechanismMotors, sensors);
		ControlIntakeArmTask armControl = new ControlIntakeArmTask(0, sensors, mechanismMotors);
		
		elevatorControl.realtimeTask = true;
		armControl.realtimeTask = true;
		
		taskExecutor.addTask(new TeleopArcadeDriveTask(0, uig, driveTrain, ahrs, sensors));
		
		TeleopIntakeTask intakeTask = new TeleopIntakeTask(0, uig, mechanismMotors, sensors);
		taskExecutor.addTask(intakeTask);
		
		TeleopElevatorTask elevatorTask = new TeleopElevatorTask(0, uig, mechanismMotors, elevatorControl, armControl);
		taskExecutor.addTask(elevatorTask);
		
		TeleopIntakeArmTask intakeArm = new TeleopIntakeArmTask(0, uig, armControl, sensors, mechanismMotors, elevatorControl, intakeTask, elevatorTask);
		taskExecutor.addTask(intakeArm);
		elevatorTask.GiveIntakeArm(intakeArm);
				
		taskExecutor.addTask(elevatorControl);
		taskExecutor.addTask(armControl);
		
		taskExecutor.teleopStart(); //prepare for startup!
		
		realStartTime = Timer.getFPGATimestamp();
		
		teleopRealTimeController(); //begin 200hz control loop
	}

	public void disabledPeriodic() {
		taskExecutor.stop();
		taskExecutor.forceClearTasks();
		driveTrain.stopCoast();
		controlLoopCycle = 0;
	}
	
	public void teleopPeriodic() {
		//do nothing
	}
	
	public void updateLoopStat(double current_time) {
		if(controlLoopCycle % moduloUpdateCycle == 1) {
			SmartDashboard.putNumber("control loop delta", current_time - lastTime);
		}
		
		lastTime = current_time;
		controlLoopCycle++;
	}
	
	public void autoRealTimeControlLoop() {
		RobotLogger.toast("Control Loop (Autonomous) Starting");
		while (this.isEnabled() && this.isAutonomous()) {
			double current_time = Timer.getFPGATimestamp() - realStartTime;
			taskExecutor.executeDT(current_time, false); //dont care about realtime locks, just run at full speed
			try {
				Thread.sleep((long) (1000 / REAL_TIME_LOOP_HZ));
			} catch (Exception e) {
				e.printStackTrace();
			}
			
			updateLoopStat(current_time);
		}
		taskExecutor.stop();
	}

	public void teleopRealTimeController() {
		RobotLogger.toast("Control Loop (Teleop) Starting");
		while (this.isEnabled() && this.isOperatorControl()) {
			double current_time = Timer.getFPGATimestamp() - realStartTime;
			if(this.isNewDataAvailable()) {
				CameraSystem.teleopUpdate(uig.getStickB());
				taskExecutor.executeDT(current_time, false);
			} else {
				taskExecutor.executeDT(current_time, true);
			}
			
			try {
				Thread.sleep((long) (1000 / REAL_TIME_LOOP_HZ));
			} catch (Exception e) {
				e.printStackTrace();
			}
			
			updateLoopStat(current_time);
		}
		taskExecutor.stop();
	}
	
	public void testInit() {
		RobotLogger.toast("no test");
	}
}
