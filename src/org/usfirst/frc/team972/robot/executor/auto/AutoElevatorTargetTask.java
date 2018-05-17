package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;

public class AutoElevatorTargetTask extends Task {

	double position;
	ControlElevatorTask elevator;
	ControlIntakeArmTask arm;
	
	public AutoElevatorTargetTask(double _executionTime, double _position, ControlElevatorTask _elevator, ControlIntakeArmTask _arm) {
		super(_executionTime);
		// TODO Auto-generated constructor stub
		position = _position;
		elevator = _elevator;
		arm = _arm;
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void execute(double dt) {
		if(arm.isSafeOutwards()) {
			RobotLogger.toast("Auto Setting Elevator to: " + position);
			if(elevator.allowedControl == false) {
				RobotLogger.toast("Auto Task force-steal elevator control!", RobotLogger.URGENT);
				elevator.setControl(true);
			}
			elevator.setElevatorPositionTarget(position);
		} else {
			arm.allowedControl = false;
			RobotLogger.toast("Auto Cancel Elevator: " + arm.getLeftPos() + " " + arm.getRightPos());
			//TODO: turn off the arms because we failed to do somthing cool with it and we want it to go floppy so no damage
		}
		super.destroy();
	}

}
