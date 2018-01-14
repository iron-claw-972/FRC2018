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
	final double DEAD_BAND_THROTTLE = 0.05;
	final double INERTIA_TO_POWER = 0.2;
	
	MainDriveTrain driveTrain;
	UserInputGamepad uig;
	
	int driveGearMode = 0;

	double leftSideInertiaAccum = 0;
	double rightSideInertiaAccum = 0;
	double maxInertia = 50;
	
	public TeleopTankDriveTask(double _executionTime, UserInputGamepad _uig, MainDriveTrain _driveTrain) {
		super(_executionTime);
		super.autoRemove = false;
		uig = _uig;
		driveTrain = _driveTrain;
	}
	
	//this is teleopPeriodic
	public void execute(double dt) {
		double leftDrive = uig.getStickA().getRawAxis(LEFT_DRIVE_AXIS);
		double rightDrive = uig.getStickA().getRawAxis(RIGHT_DRIVE_AXIS);

		leftDrive = handleDeadband(leftDrive, DEAD_BAND_THROTTLE);
		rightDrive = handleDeadband(rightDrive, DEAD_BAND_THROTTLE);
		
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
		
		leftSideInertiaAccum = leftSideInertiaAccum + leftDrive;
		rightSideInertiaAccum = rightSideInertiaAccum + rightDrive;
		
		if(Math.abs(leftSideInertiaAccum) > maxInertia) {
			leftSideInertiaAccum = Math.signum(leftSideInertiaAccum) * maxInertia;
		} if(Math.abs(rightSideInertiaAccum) > maxInertia) {
			rightSideInertiaAccum = Math.signum(rightSideInertiaAccum) * maxInertia;
		}
		
		if(leftDrive == 0 && (Math.abs(leftSideInertiaAccum) > 0)) {
			leftSideInertiaAccum = leftSideInertiaAccum -Math.signum(leftSideInertiaAccum) * 1;
			leftDrive = (leftSideInertiaAccum/maxInertia) * INERTIA_TO_POWER;
		} if(rightDrive == 0 && (Math.abs(rightSideInertiaAccum) > 0)) {
			rightSideInertiaAccum = rightSideInertiaAccum -Math.signum(rightSideInertiaAccum) * 1;
			rightDrive = (rightSideInertiaAccum/maxInertia) * INERTIA_TO_POWER;
		}
		
		leftDrive = handleDeadband(leftDrive, DEAD_BAND_THROTTLE);
		rightDrive = handleDeadband(rightDrive, DEAD_BAND_THROTTLE);

		driveTrain.driveSidesPWM(leftDrive,rightDrive);
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub
		
	}
	
    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

}
