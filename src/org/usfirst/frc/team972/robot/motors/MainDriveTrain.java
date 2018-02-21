package org.usfirst.frc.team972.robot.motors;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.motionlib.CoolMath;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class MainDriveTrain {
	
	final int ENCODER_PULSES_PER_REV = 2 * 1024;
	final int WHEEL_DIAMETER_INCHES = 6;
	final double METER_CONVERSION_INCHES = 0.0254;
	final double GEARBOX_RATIO = 1; //4.8925, real bot is direct driven
	
	final int MOTOR_MAX_CURRENT = 40;
	
	final double MOTOR_VOLTAGE_SATURATION = 10;
	boolean voltageCompensation = false;
	
	DoubleSolenoid ShiftSolenoid;
	
	WPI_TalonSRX Right_1;
	WPI_TalonSRX Right_2;
	WPI_TalonSRX Right_3;
	
	WPI_TalonSRX Left_1;
	WPI_TalonSRX Left_2;
	WPI_TalonSRX Left_3;
	
	WPI_TalonSRX talons[] = new WPI_TalonSRX[6];
	
	public void SetupRight(int f, int m, int b) {
		Right_1 = new WPI_TalonSRX(f);
		Right_2= new WPI_TalonSRX(m);
		Right_3 = new WPI_TalonSRX(b);
	}
	
	public void SetupLeft(int f, int m, int b) {
		Left_1 = new WPI_TalonSRX(f);
		Left_2 = new WPI_TalonSRX(m);
		Left_3 = new WPI_TalonSRX(b);
	}
	
	public void SetupShift(int id, int a, int b) {
		ShiftSolenoid = new DoubleSolenoid(id, a, b);
		RobotLogger.toast("DriveTrain Solenoid Set");
	}
	
	public void SetupProcedure(int a, int b, int c, int d, int e, int f) {
		SetupLeft(a, b, c);
		SetupRight(d, e, f);
		
		talons[3] = Right_1;
		talons[4] = Right_2;
		talons[5] = Right_3;
		
		talons[0] = Left_1;
		talons[1] = Left_2;
		talons[2] = Left_3;
		
		setTalonsFastRate();

		RobotLogger.toast("DriveTrain Talons Set, Prepare Diagnosis");
		diagnosis();
	}
	
	public void diagnosis() {
		int count = 0;
		double averageBusVoltage = 0;
		
		for(int i=0; i<talons.length; i++) {
			try {
				if(talons[i].getBusVoltage() > 5) {
					count++;
					averageBusVoltage = averageBusVoltage + talons[i].getBusVoltage();
				}
			} catch(Exception e) {
				RobotLogger.toast(e.getMessage());
			}
		}
		
		if(count != 6) {
			RobotLogger.toast("Abnormal Talon Count Detected for Drive Train! Talon Count Value = " + count, RobotLogger.URGENT);
		} else {
			RobotLogger.toast("Nominal Alive Count for Drive Train");
		}
		
		averageBusVoltage = averageBusVoltage/6;
		
		RobotLogger.toast("Average Drive Train Input Voltage:" + averageBusVoltage);
	}
	
	public double encoderPulseToRadians(int encoderPulse) {
		return (Math.PI * ((double)encoderPulse/ENCODER_PULSES_PER_REV)) / GEARBOX_RATIO;
	}
	
	public double radiansToLinearMeters(double radians) {
		return radians * WHEEL_DIAMETER_INCHES * (METER_CONVERSION_INCHES);
	}
	
	public double pulseToMetersLinear(int encoderPulse) {
		return radiansToLinearMeters(encoderPulseToRadians(encoderPulse));
	}
	
	public void shiftSolenoidUp() {
		
		ShiftSolenoid.set(DoubleSolenoid.Value.kReverse);
		RobotLogger.toast("DriveTrain Shift Up");
		SmartDashboard.putString("Gear Mode", "high gear");
		
	}
	
	public void shiftSolenoidDown() {
		
		ShiftSolenoid.set(DoubleSolenoid.Value.kForward);
		RobotLogger.toast("DriveTrain Shift Down");
		SmartDashboard.putString("Gear Mode", "low gear");
		
	}
	
	public void setTalonsPWM_follow() {
		//switch all talons to followers
		
		Right_1.set(ControlMode.PercentOutput, 0);
		Right_2.set(ControlMode.PercentOutput, 0);
		Right_3.set(ControlMode.PercentOutput, 0);
		
		Left_1.set(ControlMode.PercentOutput, 0);
		Left_2.set(ControlMode.PercentOutput, 0);
		Left_3.set(ControlMode.PercentOutput, 0);
				
		setTalonsBrake();
		voltageCompensation();
	}
	
	public void voltageCompensation() {
		if(voltageCompensation == false) {
			for(int i=0; i<talons.length; i++) {
				talons[i].enableVoltageCompensation(true);
				talons[i].configVoltageCompSaturation(MOTOR_VOLTAGE_SATURATION, 0);
			}
			voltageCompensation = true;
			RobotLogger.toast("DriveTrain Power Locked to: " + MOTOR_VOLTAGE_SATURATION);
		}
	}
	
	public void voltageUnlock() {
		if(voltageCompensation) {
			for(int i=0; i<talons.length; i++) {
				talons[i].enableVoltageCompensation(false);
			}
			voltageCompensation = false;
			RobotLogger.toast("DriveTrain FULL Power Unlocked");
		}
	}
	
	public void setTalonsBrake() {
		RobotLogger.toast("Setting Drive Talons to Brake");
		Left_1.setNeutralMode(NeutralMode.Brake);
		Left_2.setNeutralMode(NeutralMode.Brake);
		Left_3.setNeutralMode(NeutralMode.Brake);
		
		Right_1.setNeutralMode(NeutralMode.Brake);
		Right_2.setNeutralMode(NeutralMode.Brake);
		Right_3.setNeutralMode(NeutralMode.Brake);
	}
	
	public void setTalonsCoast() {
		RobotLogger.toast("Setting Drive Talons to Coast");
		Left_1.setNeutralMode(NeutralMode.Coast);
		Left_2.setNeutralMode(NeutralMode.Coast);
		Left_3.setNeutralMode(NeutralMode.Coast);
		
		Right_1.setNeutralMode(NeutralMode.Coast);
		Right_2.setNeutralMode(NeutralMode.Coast);
		Right_3.setNeutralMode(NeutralMode.Coast);
	}
	
	public void setTalonsFastRate() {
		RobotLogger.toast("Setting Drive Talons to Fast Update Rate");
		for(int i=0; i<talons.length; i++) {
			talons[i].changeMotionControlFramePeriod(5);
		}
	}
	
	public void stopHard() {
		Left_1.set(0);
		Right_1.set(0);
		Left_2.set(0);
		Right_2.set(0);
		Left_3.set(0);
		Right_3.set(0);
	}
	
	public void stopCoast() {
		Left_1.setNeutralMode(NeutralMode.Coast);
		Right_1.setNeutralMode(NeutralMode.Coast);
		Left_1.set(0);
		Right_1.set(0);
		Left_2.setNeutralMode(NeutralMode.Coast);
		Right_2.setNeutralMode(NeutralMode.Coast);
		Left_2.set(0);
		Right_2.set(0);
		Left_3.setNeutralMode(NeutralMode.Coast);
		Right_3.setNeutralMode(NeutralMode.Coast);
		Left_3.set(0);
		Right_3.set(0);
	}
	
	public void logOutputCurrent() {
		double currentSum = 0;
		for(int i=0; i<talons.length; i++) {
			currentSum = currentSum + talons[i].getOutputCurrent();
		}
		SmartDashboard.putNumber("average drivetrain current", currentSum/6);
	}
	
	public void driveSidesPWM(double d, double e) {
		SmartDashboard.putNumber("lo", d);
		SmartDashboard.putNumber("ro", e);
		
		logOutputCurrent();
		
		/*
		 * our drive_train gears are weird... top motor is INVERTED.
		 */
		
		Left_1.set(-d); //invert this
		Left_2.set(d);
		Left_3.set(d);
		
		Right_1.set(e);
		Right_2.set(-e); //invert this
		Right_3.set(-e); //invert this
		
	}
}
