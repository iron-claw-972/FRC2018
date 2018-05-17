package org.usfirst.frc.team972.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MechanismActuators {
	
	final double MAX_INTAKE_DRAW = 15;
	
	public WPI_TalonSRX intakeMotorLeft;
	public WPI_TalonSRX intakeMotorRight;
	
	WPI_TalonSRX winchLiftMotor;	//do not use
	WPI_TalonSRX elevatorFlopMotor; //do not use
	
	public WPI_TalonSRX elevatorLiftMotor;
	
	public WPI_TalonSRX intakeArmMotorLeft;
	public WPI_TalonSRX intakeArmMotorRight;
	
	public void SetupIntakeArmMotors(int left, int right) {
		intakeArmMotorLeft = new WPI_TalonSRX(left);
		intakeArmMotorRight = new WPI_TalonSRX(right);
		
		intakeArmMotorLeft.set(ControlMode.PercentOutput, 0);
		intakeArmMotorRight.set(ControlMode.PercentOutput, 0);
		
		intakeArmMotorLeft.setNeutralMode(NeutralMode.Coast);
		intakeArmMotorRight.setNeutralMode(NeutralMode.Coast);
	}
	
	public WPI_TalonSRX SetupElevatorLiftMotor(int motorId) {
		elevatorLiftMotor = new WPI_TalonSRX(motorId);
		elevatorLiftMotor.setNeutralMode(NeutralMode.Brake);
		elevatorLiftMotor.set(ControlMode.PercentOutput, 0);
		
		return elevatorLiftMotor;
	}
	
	public void SetupIntakeMotors(int left, int right) {
		intakeMotorLeft = new WPI_TalonSRX(left);
		intakeMotorRight = new WPI_TalonSRX(right);
		
		intakeMotorLeft.setNeutralMode(NeutralMode.Brake);
		intakeMotorRight.setNeutralMode(NeutralMode.Brake);
		
		intakeMotorLeft.set(ControlMode.PercentOutput, 0);
		intakeMotorRight.set(ControlMode.PercentOutput, 0);
	}
	
	public void SetupWinchMotors(int motorId) {
		winchLiftMotor = new WPI_TalonSRX(motorId);
		winchLiftMotor.set(ControlMode.PercentOutput, 0);
		winchLiftMotor.setNeutralMode(NeutralMode.Brake);
	}
	
	public void RunIntakeMotors(double power) {
		//power = -power;
		intakeMotorLeft.set(power);
		intakeMotorRight.set(power);
	}
	
	public void RunWinchMotor(double power) {
		//winchLiftMotor.set(power);
	}
	
	public void RunElevatorLiftMotor(double power) {
		
		if(power > 0.95) {
			power = 0.95;
		} else if(power < -0.95) {
			power = -0.95;
		}
		
		SmartDashboard.putNumber("elevator output", power);
		elevatorLiftMotor.set(power);
	}

	public void RunIntakeArmMotors(double left, double right) {
		if(left > 1) {
			left = 1;
		} else if(left < -1) {
			left = -1;
		}
		
		if(right > 1) {
			right = 1;
		} else if(right < -1) {
			right = -1;
		}
		
		left = -left * 0.315;
		right = -right * 0.315;

		intakeArmMotorLeft.set(left); //target 3 volts
		intakeArmMotorRight.set(right);
		
		SmartDashboard.putString("intake arm motor", left + " - " + right);
	}
	
	public void RunFlopMotor(double power) {
		//elevatorFlopMotor.set(power);
	}
	
	public boolean IntakeMotorOverdraw() {
		return (intakeMotorLeft.getOutputCurrent() > MAX_INTAKE_DRAW) || (intakeMotorRight.getOutputCurrent() > MAX_INTAKE_DRAW);  
	}
	
	//public WPI_TalonSRX SetupElevatorFlopMotor(int i) {
		//elevatorFlopMotor = new WPI_TalonSRX(i);
		//elevatorFlopMotor.setNeutralMode(NeutralMode.Brake);
		//elevatorFlopMotor.set(0);
		
		//return elevatorFlopMotor;
	//}
}