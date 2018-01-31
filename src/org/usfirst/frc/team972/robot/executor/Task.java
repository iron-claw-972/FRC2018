package org.usfirst.frc.team972.robot.executor;

public abstract class Task {
	double executionTime = 0;
	double realExecutionTime = 0;
	boolean executed = false;
	boolean autoRemove = false;
	boolean allowedRun = false;
	private boolean blocking = false;
	
	boolean finished = false;
	
	public Task(double _executionTime) {
		this.executionTime = _executionTime;
		allowedRun = true;
		finished = false;
	}
	
	public abstract void init(double dt);
	
	public abstract void execute(double dt);
	
	public void destroy() {
		allowedRun = false;
		finished = true;
	}
	
	public void block() {
		blocking = true;
	}
	
	public void free() {
		blocking = false;
	}
	
	public boolean blocking() {
		return blocking;
	}
	
	public void setExecuted() {
		executed = true;
	}
	
}
