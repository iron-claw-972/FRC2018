package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;

public class AutoElevatorTargetTask extends Task {

	double position;
	ControlElevatorTask elevator;
	
	public AutoElevatorTargetTask(double _executionTime, double _position, ControlElevatorTask _elevator) {
		super(_executionTime);
		// TODO Auto-generated constructor stub
		position = _position;
		elevator = _elevator;
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void execute(double dt) {
		RobotLogger.toast("Auto Setting Elevator to: " + position);
		if(elevator.allowedControl == false) {
			RobotLogger.toast("Auto Task force-steal elevator control!", RobotLogger.URGENT);
			elevator.setControl(true);
		}
		elevator.setElevatorPositionTarget(position);
		super.destroy();
	}

}
