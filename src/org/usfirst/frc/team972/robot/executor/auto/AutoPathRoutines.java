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
				
			//--ESPECIAL--
			case "left_to_right_switch":
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 5.51, 0, driveTrain, 10, sensors, ahrs));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 90, 3, driveTrain, ahrs));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 4.7, 90, driveTrain, 10, sensors, ahrs));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 180, 3, driveTrain, ahrs));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 0.5, 180, driveTrain, 10, sensors, ahrs));
				break;
			case "right_to_left_switch":
				
				break;
			case "left_to_left_switch_outside":
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 3.7, 0, driveTrain, 5, sensors, ahrs));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 90, 3, driveTrain, ahrs));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 0.45, 90, driveTrain, 6, sensors, ahrs));
				break;
			case "left_to_left_switch_behind":
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 5.6, 0, driveTrain, 6, sensors, ahrs, 0.75));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 90 + 45, 3, driveTrain, ahrs));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 1, 90 + 10, driveTrain, 6, sensors, ahrs));
				taskExecutor.addTask(new AutoWaitTask(0, 1));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, -0.45, 90 + 45, driveTrain, 6, sensors, ahrs, 0.5));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 125, 1.5, driveTrain, ahrs));
				taskExecutor.addTask(new AutoWaitTask(0, 1));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 0.8, 125, driveTrain, 6, sensors, ahrs, 0.6));
				taskExecutor.addTask(new AutoWaitTask(0, 1));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 90 + 45, 1.5, driveTrain, ahrs));
				break;
			case "right_to_right_switch_outside":
				
				break;
			case "center_to_right_switch_2_block":
				elevatorControl.setElevatorPositionTarget(SWITCH_AUTO_HEIGHT);
				performTraj("center_to_right_switch");
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, -1, 0, driveTrain, 5, sensors, ahrs));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 60, 3, driveTrain, ahrs));
				taskExecutor.addTask(new AutoIntakeMechanism(0, 4, true, 0.5, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 0.5, 60, driveTrain, 3, sensors, ahrs));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, -0.5, 60, driveTrain, 3, sensors, ahrs));
				taskExecutor.addTask(new AutoTurnAngleTask(0, 0, 3, driveTrain, ahrs));
				taskExecutor.addTask(new AutoDrivePositionAngle(0, 1, 0, driveTrain, 3, sensors, ahrs));
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.7, sensors, mechanismMotors));
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
