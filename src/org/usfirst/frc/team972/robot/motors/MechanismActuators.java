package org.usfirst.frc.team972.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class MechanismActuators {
	
	WPI_TalonSRX intakeMotorLeft;
	WPI_TalonSRX intakeMotorRight;
	
	WPI_TalonSRX winchLiftMotor;
	
	WPI_TalonSRX elevatorLiftMotor;
	
	public void SetupElevatorLiftMotor(int motorId) {
		elevatorLiftMotor = new WPI_TalonSRX(motorId);
		elevatorLiftMotor.setNeutralMode(NeutralMode.Brake);
		elevatorLiftMotor.set(ControlMode.PercentOutput, 0);
		
		elevatorLiftMotor.configPeakCurrentLimit(30, 0);
		elevatorLiftMotor.enableCurrentLimit(true);
	}
	
	public void SetupIntakeMotors(int left, int right) {
		intakeMotorLeft = new WPI_TalonSRX(left);
		intakeMotorRight = new WPI_TalonSRX(right);
		
		intakeMotorLeft.setNeutralMode(NeutralMode.Coast);
		intakeMotorRight.setNeutralMode(NeutralMode.Coast);
		
		intakeMotorLeft.configPeakCurrentLimit(10, 0);
		intakeMotorRight.configPeakCurrentLimit(10, 0);
		intakeMotorLeft.enableCurrentLimit(true);
		intakeMotorRight.enableCurrentLimit(true);
		
		intakeMotorLeft.set(ControlMode.PercentOutput, 0);
		intakeMotorRight.set(ControlMode.PercentOutput, 0);
	}
	
	public void SetupWinchMotors(int motorId) {
		winchLiftMotor = new WPI_TalonSRX(motorId);
		winchLiftMotor.set(ControlMode.PercentOutput, 0);
		winchLiftMotor.setNeutralMode(NeutralMode.Brake);
	}
	
	public void RunIntakeMotors(double power) {
		intakeMotorLeft.set(power);
		intakeMotorRight.set(-power);
	}
	
	public void RunWinchMotor(double power) {
		winchLiftMotor.set(power);
	}
	
	public void RunElevatorLiftMotor(double power) {
		elevatorLiftMotor.set(power);
	}
}