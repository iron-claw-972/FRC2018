package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;

public class AutoIntakeArmTargetTask extends Task {
	
	public AutoIntakeArmTargetTask(double _executionTime, double _position, ControlElevatorTask _elevator, ControlIntakeArmTask _arm) {
		super(_executionTime);
		// TODO Auto-generated constructor stub
		position = _position;
		elevator = _elevator;
		arm = _arm;
	}
	
	double position;
	ControlElevatorTask elevator;
	ControlIntakeArmTask arm;
	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void execute(double dt) {
		if(Math.abs(elevator.getPosition()) > -1) { //OVERRIDE SAFETY
			RobotLogger.toast("Auto Setting Arm to: " + position);
			if(arm.allowedControl == false) {
				RobotLogger.toast("Auto Task force-steal arm control!", RobotLogger.URGENT);
				arm.allowedControl = true;
			}
			arm.setPositionTarget(position, -position);
		} else {
			arm.allowedControl = false;
			RobotLogger.toast("Auto Cancel Arm: " + elevator.getPosition());
			//TODO: turn off the arms because we failed to do somthing cool with it and we want it to go floppy so no damage
		}
		super.destroy();
	}

	
}
