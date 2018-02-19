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
	
	final double SWITCH_AUTO_HEIGHT = 4 * 12 * 0.0254;
	
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

		ControlFlopTask flopControl = new ControlFlopTask(0, mechanismMotors, sensors);
		ControlElevatorTask elevatorControl = new ControlElevatorTask(0, mechanismMotors, sensors, flopControl);
		
		taskExecutor.addTask(flopControl);
		taskExecutor.addTask(elevatorControl);
		
		flopControl.setFlopPositionTarget(0.05);
		
		switch(selectedFile) {
			case "right_to_right_switch": // -- switches --
				elevatorControl.setElevatorPositionTarget(SWITCH_AUTO_HEIGHT);
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 0.5, false, 0.7, sensors, mechanismMotors));
				break;
			case "left_to_left_switch":
				elevatorControl.setElevatorPositionTarget(SWITCH_AUTO_HEIGHT);
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
				break;
			case "center_to_right_switch":
				elevatorControl.setElevatorPositionTarget(SWITCH_AUTO_HEIGHT);
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
				break;
			case "center_to_left_switch":
				elevatorControl.setElevatorPositionTarget(SWITCH_AUTO_HEIGHT);
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
				break;	
			case "right_to_right_scale": // -- scales --
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
				break;
			case "left_to_left_scale":
				performTraj(selectedFile);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
				break;
			case "right_scale_defense": // -- defense --
				performTraj("baseline_defense");
				break;
			case "left_scale_defense":
				performTraj("baseline_defense");
				break;
			default:
				RobotLogger.toast("Unhandled Auto Pick: " + selectedFile, RobotLogger.URGENT);
				performTraj(selectedFile);
				break;
		}
		
		return true;
	}
	
	public void performTraj(String selectedFile) {
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

		TrajectoryExecutionTask follower = new TrajectoryExecutionTask(0, driveTrain, sensors, ahrs);

		taskExecutor.addTask(new AutoDriveVelocityProfileTask(0,
				SplineGeneration.generateWheelTrajectories(splineTrajectory, Robot.WHEEL_BASE_WIDTH), sensors, follower, splineTrajectory));
		taskExecutor.addTask(follower);
	}
	
}
