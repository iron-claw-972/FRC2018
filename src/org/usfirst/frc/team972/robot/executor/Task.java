package org.usfirst.frc.team972.robot.executor;

public abstract class Task {
	double executionTime = 0;
	boolean executed = false;
	boolean autoRemove = false;
	boolean allowedRun = false;
	
	boolean finished = false;
	
	public Task(double _executionTime) {
		this.executionTime = _executionTime;
		allowedRun = true;
		finished = false;
	}
	
	public abstract void init();
	
	public abstract void execute(double dt);
	
	public void destroy() {
		allowedRun = false;
		finished = true;
	}
	
	
	
	public void setExecuted() {
		executed = true;
	}
	
	public void finish() {
		finished = true;
	}
	
}
