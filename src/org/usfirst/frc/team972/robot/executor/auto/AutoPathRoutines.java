package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.AutoPicker;
import org.usfirst.frc.team972.robot.FileInput;
import org.usfirst.frc.team972.robot.Robot;
import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.TaskExecutor;
import org.usfirst.frc.team972.robot.motionlib.SplineGeneration;
import org.usfirst.frc.team972.robot.motionlib.Trajectory;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

import com.kauailabs.navx.frc.AHRS;

public class AutoPathRoutines {
	
	TaskExecutor taskExecutor;
	AutoQuery autoQuery;
	MainDriveTrain driveTrain;
	Sensors sensors;
	AHRS ahrs;
	MechanismActuators mechanismMotors;
	
	final double SWITCH_AUTO_HEIGHT = 0.55;
	final double SCALE_AUTO_HEIGHT = 0.9;
	
	public AutoPathRoutines(TaskExecutor _taskExecutor, AutoQuery _autoQuery, MainDriveTrain _driveTrain, Sensors _sensors, AHRS _ahrs, MechanismActuators _mechanismMotors) {
		taskExecutor = _taskExecutor;
		autoQuery = _autoQuery;
		driveTrain = _driveTrain;
		sensors = _sensors;
		ahrs = _ahrs;
		mechanismMotors = _mechanismMotors;
	}
	
	public boolean pickRoutine() {
		String selectedFile = AutoPicker.selectFile(autoQuery);

		ControlIntakeArmTask intakeArm = new ControlIntakeArmTask(0, sensors, mechanismMotors);
		ControlElevatorTask elevatorControl = new ControlElevatorTask(0, mechanismMotors, sensors);
		
		taskExecutor.addTask(elevatorControl);
		taskExecutor.addTask(intakeArm);

		intakeArm.setPositionTarget(0, 0);
		
		switch(selectedFile) {
			case "five_meters_foward":
				performTraj(selectedFile);
				break;
			case "right_to_right_switch": // -- switches --
				intakeArm.setPositionTarget(0.5, 0.5);
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(SWITCH_AUTO_HEIGHT);
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.3, sensors, mechanismMotors));
				break;
			case "nothing": 
				RobotLogger.toast("Auto Do Nothing! Only Perform Zeroing!");
				break;
				
			default:
				RobotLogger.toast("Unhandled Auto Pick: " + selectedFile, RobotLogger.URGENT);
				performTraj(selectedFile);
				break;
		}
		
		return true;
	}
	
	public void performTrajWait(String selectedFile, double time, boolean invert) {
		RobotLogger.toast("Begin Trajectory Generation");
		Trajectory splineTrajectory = new Trajectory(0);
		try {
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

		TrajectoryExecutionTask follower = new TrajectoryExecutionTask(time, driveTrain, sensors, ahrs, 15);

		taskExecutor.addTask(new AutoDriveVelocityProfileTask(time,
				SplineGeneration.generateWheelTrajectories(splineTrajectory, Robot.WHEEL_BASE_WIDTH), sensors, follower, splineTrajectory, invert));
		taskExecutor.addTask(follower);
	}
	
	public void performTraj(String selectedFile, boolean invert) {
		performTraj(selectedFile, 15, invert);
	}
	
	public void performTraj(String selectedFile, double maxTime, boolean invert) {
		RobotLogger.toast("Begin Trajectory Generation");
		Trajectory splineTrajectory = new Trajectory(0);
		try {
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

		TrajectoryExecutionTask follower = new TrajectoryExecutionTask(0, driveTrain, sensors, ahrs, maxTime);

		taskExecutor.addTask(new AutoDriveVelocityProfileTask(0,
				SplineGeneration.generateWheelTrajectories(splineTrajectory, Robot.WHEEL_BASE_WIDTH), sensors, follower, splineTrajectory, invert));
		taskExecutor.addTask(follower);
	}
	
	
}
