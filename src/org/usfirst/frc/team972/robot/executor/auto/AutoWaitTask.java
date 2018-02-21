package org.usfirst.frc.team972.robot.executor.auto;

import org.usfirst.frc.team972.robot.RobotLogger;
import org.usfirst.frc.team972.robot.executor.Task;

public class AutoWaitTask extends Task {

	double wait = 0;
	double start = 0;
	
	public AutoWaitTask(double _executionTime, double _wait) {
		super(_executionTime);
		wait = _wait;
	}

	@Override
	public void init(double dt) {
		// TODO Auto-generated method stub
		this.block();
		start = dt;
	}

	@Override
	public void execute(double dt) {
		if((dt - start) > wait) {
			RobotLogger.toast("Wait Finished");
			super.free();
			super.destroy();
		}
	}

}
