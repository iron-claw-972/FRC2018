package org.usfirst.frc.team972.robot.executor;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motionlib.PIDControl;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.ui.Sensors;
import org.usfirst.frc.team972.robot.ui.UserInputGamepad;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopArcadeDriveTask extends Task {

	final int LEFT_DRIVE_AXIS = 1;
	final int RIGHT_DRIVE_AXIS = 4;
	final int SHIFT_BUTTON = 5;
	final int TURBO_BUTTON = 6;

	int currentSpeedMode = 0;
	
	double speedModes[] = {0.25, 0.5, 0.75, 1};
	
	double z_alpha = Math.PI * 0.4;
	
	final int SPEED_MODE_0 = 3; //these are button id's
	final int SPEED_MODE_1 = 1;
	final int SPEED_MODE_2 = 2;
	final int SPEED_MODE_3 = 4;
	
	MainDriveTrain driveTrain;
	UserInputGamepad uig;
	
	AHRS ahrs;
	int driveGearMode = 0;
	
	double leftDrive = 0;
	double rightDrive = 0;
	double easingValue = 0.1;

	Sensors sensors;
	PIDControl pidAngle;
	
	boolean calibratingAngle = false;
	double desiredAngle = 0;
	double fakeDesiredAngle = 0;
	double lastDesiredAngle = 0;
	
	public TeleopArcadeDriveTask(double _executionTime, UserInputGamepad _uig, MainDriveTrain _driveTrain, AHRS _ahrs, Sensors _sensors) {
		super(_executionTime);
		super.autoRemove = false;
		uig = _uig;
		driveTrain = _driveTrain;
		ahrs = _ahrs;
		sensors = _sensors;
		
		pidAngle = new PIDControl(0.025, 0, 0.05);
		pidAngle.setOutputLimits(-0.08, 0.08);
		pidAngle.setSetpointRange(2);
		pidAngle.setOutputFilter(0.1);
	}
	
	
	private double interpolateValues(double want, double actual) {
		double error = (want - actual) * easingValue;
		return actual + error;
	}
	
	double last_steering_set = 0;
	double inertia_counter = 0;
	double turn_stop_counter = 0;
	
	//this is teleopPeriodic
	public void execute(double dt) {
		double currentAngle = ahrs.getAngle();

		if(uig.getStickA().getRawButton(TURBO_BUTTON)) {
			driveTrain.voltageUnlock();
		} else {
			driveTrain.voltageCompensation();
		}
		
		if(uig.getStickA().getRawButton(SPEED_MODE_0)) {
			currentSpeedMode = 0;
		} else if(uig.getStickA().getRawButton(SPEED_MODE_1)) {
			currentSpeedMode = 1;
		} else if(uig.getStickA().getRawButton(SPEED_MODE_2)) {
			currentSpeedMode = 2;
		} else if(uig.getStickA().getRawButton(SPEED_MODE_3)) {
			currentSpeedMode = 3;
		}
		
		if(uig.getStickA().getRawButtonPressed(SHIFT_BUTTON)) {
			currentSpeedMode = 0;
			if(driveGearMode == 0) {
				driveGearMode = 1; //do high gear
				driveTrain.shiftSolenoidUp();
			} else {
				driveGearMode = 0; //do low gear
				driveTrain.shiftSolenoidDown();
			}
		}
		
		double throttle = -uig.getStickA().getRawAxis(LEFT_DRIVE_AXIS);
		double steer_set = uig.getStickA().getRawAxis(RIGHT_DRIVE_AXIS);
		double correctionAngleOutput = 0;
		
		throttle = handleDeadband(throttle, 0.05);
		steer_set = handleDeadband(steer_set, 0.05);

		SmartDashboard.putNumber("left speed real", driveTrain.pulseToMetersLinear(sensors.getLeftDriveEncoderSpeed()));
		
		if(steer_set == 0) {
			if(calibratingAngle) {
				fakeDesiredAngle = ahrs.getAngle();
				double deltaDesired = fakeDesiredAngle - lastDesiredAngle;
				if(Math.abs(deltaDesired) < 0.05) {
					RobotLogger.toast("Teleop Arcade Close Loop Set: " + fakeDesiredAngle);
					calibratingAngle = false;
				}
				desiredAngle = fakeDesiredAngle;
				lastDesiredAngle = fakeDesiredAngle;
			}
			
			correctionAngleOutput = pidAngle.getOutput(currentAngle, desiredAngle);
		} else {
			calibratingAngle = true;
		}
		
		double turn_in_place = 1;

		double steering_compensate_inertia = steer_set - last_steering_set;
		last_steering_set = steer_set;

		double steering_inertia_scale = 0;

		if ((Math.abs(throttle) > 0.05) || (steer_set == 0)) {
		        turn_in_place = 0;
		}

		steer_set = Math.sin(z_alpha * steer_set) / Math.sin(z_alpha);
		steer_set = Math.sin(z_alpha * steer_set) / Math.sin(z_alpha);

		steer_set = (2 * steer_set) - steer_set;

		if(steer_set * steering_compensate_inertia > 0) {
		        steering_inertia_scale = 4;
		} else {
		        steering_inertia_scale = 5;
		}

		inertia_counter = inertia_counter + (steering_compensate_inertia * steering_inertia_scale);

		steer_set = steer_set + inertia_counter;

		if(inertia_counter > 1) {
		        inertia_counter = inertia_counter - 1;
		} else if (inertia_counter < -1) {
		        inertia_counter = inertia_counter + 1;
		} else {
		        inertia_counter = 0;
		}

		double left_p = throttle;
		double right_p = throttle;

		double steering_force = 0;

		if(steer_set > 1) {
		        steer_set = 1;
		} else if (steer_set < -1) {
		        steer_set = -1;
		}

		if(turn_in_place == 1) {
		        double z1 = 0.1;
		        turn_stop_counter = ((1 - z1) * turn_stop_counter) + (z1 * clamp(steer_set, 0, 1.0) * 3);
		        steering_force = steer_set;
		} else {
		        steering_force = Math.abs(throttle) * steer_set - turn_stop_counter;
		        if (turn_stop_counter > 1) {
		                turn_stop_counter = turn_stop_counter - 1;
		        } else if (turn_stop_counter < -1) {
		                turn_stop_counter = turn_stop_counter + 1;
		        } else {
		                turn_stop_counter = 0;
				}
		}

		left_p = left_p + steering_force;
		right_p = right_p - steering_force;

		if(left_p > 1) {
		        right_p = right_p - (turn_in_place * (left_p - 1));
		        left_p = 1;
		} else if(right_p > 1) {
		        left_p = left_p - (turn_in_place * (right_p - 1));
		        right_p = 1;
		} else if(left_p < -1) {
		        right_p = right_p + (turn_in_place * (-1 - left_p));
		        left_p = -1;
		} else if(right_p < -1) {
		        left_p = left_p + (turn_in_place * (-1 - right_p));
		        right_p = -1;
		}

		leftDrive = interpolateValues(left_p * speedModes[currentSpeedMode], leftDrive);
		rightDrive = interpolateValues(right_p * speedModes[currentSpeedMode], rightDrive);
		
		double leftRealDist = driveTrain.pulseToMetersLinear(sensors.getLeftDriveEncoder());
		double rightRealDist = -driveTrain.pulseToMetersLinear(sensors.getRightDriveEncoder());
		
		SmartDashboard.putNumber("left real", leftRealDist);
		SmartDashboard.putNumber("right real", rightRealDist);
		SmartDashboard.putNumber("angle ahrs", currentAngle);
		
		driveTrain.driveSidesPWM(leftDrive + correctionAngleOutput, rightDrive - correctionAngleOutput);
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}
	
    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

	public double clamp(double in, double low, double high) {
		if(in < low) {
			return low;
		} else if(in > high ) {
			return high;
		} else {
			return in;
		}
	}
    
}