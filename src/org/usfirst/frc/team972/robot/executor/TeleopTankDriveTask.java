package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopTankDriveTask extends Task {

	final int LEFT_DRIVE_AXIS = 1;
	final int RIGHT_DRIVE_AXIS = 3;
	final int SHIFT_BUTTON = 6;
	final int TURBO_BUTTON = 8;
	final double DEAD_BAND_THROTTLE = 0.005;

	MainDriveTrain driveTrain;
	UserInputGamepad uig;
	
	int driveGearMode = 0;
	
	double leftDrive = 0;
	double rightDrive = 0;
	double easingValue = 0.1;
	
	public TeleopTankDriveTask(double _executionTime, UserInputGamepad _uig, MainDriveTrain _driveTrain) {
		super(_executionTime);
		super.autoRemove = false;
		uig = _uig;
		driveTrain = _driveTrain;
	}
	
	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}
	
	//this is teleopPeriodic
	public void execute(double dt) {
		double wantLeftDrive = uig.getStickA().getRawAxis(LEFT_DRIVE_AXIS);
		double wantRightDrive = uig.getStickA().getRawAxis(RIGHT_DRIVE_AXIS);

		wantLeftDrive = handleDeadband(wantLeftDrive, DEAD_BAND_THROTTLE);
		wantRightDrive = handleDeadband(wantRightDrive, DEAD_BAND_THROTTLE);
		
		if(uig.getStickA().getRawButtonPressed(SHIFT_BUTTON)) {
			if(driveGearMode == 0) {
				driveGearMode = 1; //do high gear
				driveTrain.shiftSolenoidUp();
			} else {
				driveGearMode = 0; //do low gear
				driveTrain.shiftSolenoidDown();
			}
		}
		
		if(uig.getStickA().getRawButton(TURBO_BUTTON)) {
			driveTrain.voltageUnlock();
		} else {
			driveTrain.voltageCompensation();
		}
		
		//leftDrive = wantLeftDrive;
		//rightDrive = wantRightDrive;
		
		leftDrive = interpolateValues(wantLeftDrive, leftDrive);
		rightDrive = interpolateValues(wantRightDrive, rightDrive);
		
		leftDrive = handleDeadband(leftDrive, DEAD_BAND_THROTTLE);
		rightDrive = handleDeadband(rightDrive, DEAD_BAND_THROTTLE);

		RobotLogger.toast(leftDrive + " " + rightDrive);
		driveTrain.driveSidesPWM(leftDrive,rightDrive);
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}
	
    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

}
