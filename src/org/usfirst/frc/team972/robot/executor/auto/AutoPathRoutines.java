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
	
	final double CLAMP_CUBE_AUTO_POS = 0.5;
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
		
		intakeArm.allowedControl = false;
		elevatorControl.allowedControl = false;
		
		switch(selectedFile) {
			case "center_to_right_2_block_1":
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(0.1);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.9, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoElevatorTargetTask(3, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				performTraj("center_to_right_2_block_1", false);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.8, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoElevatorTargetTask(1, 0, elevatorControl, intakeArm));
				performTraj("center_to_right_2_block_2", true);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(0, 0.7, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoIntakeMechanism(0, 2, true, 0.6, sensors, mechanismMotors));
				performTraj("1_meter", false);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(0, 0.9, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoElevatorTargetTask(0.5, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				performTraj("1_meter", true);
				performTraj("center_to_right_2_block_5", false);
				taskExecutor.addTask(new AutoIntakeMechanism(0, 1, false, 0.8, sensors, mechanismMotors));
				break;
			case "scale_drive":
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(0.1);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.9, elevatorControl, intakeArm));
				performTraj(selectedFile, false);
				taskExecutor.addTask(new AutoElevatorTargetTask(3, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				break;
			case "five_meters_foward":
				performTraj(selectedFile, false);
				break;
			case "three_meters_foward":
				performTraj(selectedFile, false);
				break;
			case "right_to_right_switch": // -- switches --
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(0.1);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.9, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoElevatorTargetTask(3, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				
				performTrajWait("right_to_right_switch", 1, false, false);
				taskExecutor.addTask(new AutoIntakeMechanism(1, 1, false, 0.9, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.5, elevatorControl, intakeArm));
				//performTrajWait("right_backout", 1, true, true);
				break;
			case "left_to_left_switch":
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(0.1);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.9, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoElevatorTargetTask(3, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				
				performTrajWait("left_to_left_switch", 1, false, false);
				taskExecutor.addTask(new AutoIntakeMechanism(1, 1, false, 0.9, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.5, elevatorControl, intakeArm));
				//performTrajWait("left_backout", 1, true, true);
				break;	
			case "center_to_right_switch": // -- center switches --
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(0.1);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.9, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoElevatorTargetTask(3, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				
				performTraj("center_to_right_switch", false);
				taskExecutor.addTask(new AutoIntakeMechanism(1, 1, false, 0.9, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoIntakeArmTargetTask(10, 0.5, elevatorControl, intakeArm));
				break;	
			case "center_to_left_switch": //
				elevatorControl.setControl(true);
				elevatorControl.setElevatorPositionTarget(0.1);
				taskExecutor.addTask(new AutoIntakeArmTargetTask(2, 0.9, elevatorControl, intakeArm));
				taskExecutor.addTask(new AutoElevatorTargetTask(3, SWITCH_AUTO_HEIGHT, elevatorControl, intakeArm));
				
				performTraj("center_to_left_switch", false);
				taskExecutor.addTask(new AutoIntakeMechanism(1, 1, false, 0.9, sensors, mechanismMotors));
				taskExecutor.addTask(new AutoIntakeArmTargetTask(10, 0.5, elevatorControl, intakeArm));
				break;		
			case "nothing": 
				RobotLogger.toast("Auto Do Nothing! Only Perform Zeroing!");
				intakeArm.allowedControl = false;
				elevatorControl.allowedControl = false;
				break;
			default:
				RobotLogger.toast("Unhandled Auto Pick: " + selectedFile, RobotLogger.URGENT);
				performTraj(selectedFile, false);
				break;
		}
		
		return true;
	}
	
	public void performTrajWait(String selectedFile, double time, boolean invert, boolean deltaWait) {
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

		if(deltaWait) {
			RobotLogger.toast("Traj using Delta wait : " + time);
			AutoDriveVelocityProfileTask avp = new AutoDriveVelocityProfileTask(0,
					SplineGeneration.generateWheelTrajectories(splineTrajectory, Robot.WHEEL_BASE_WIDTH), sensors, follower, splineTrajectory, invert);
			avp.deltaWait(time);
			taskExecutor.addTask(avp);	
		} else {
			RobotLogger.toast("Traj using Real wait : " + time);
			taskExecutor.addTask(new AutoDriveVelocityProfileTask(time,
					SplineGeneration.generateWheelTrajectories(splineTrajectory, Robot.WHEEL_BASE_WIDTH), sensors, follower, splineTrajectory, invert));
		}
		
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
