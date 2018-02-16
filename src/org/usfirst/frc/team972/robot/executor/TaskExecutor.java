package org.usfirst.frc.team972.robot.executor;

import java.util.ArrayList;

import org.usfirst.frc.team972.robot.RobotLogger;

public class TaskExecutor {
	public int currentMode = 0;
	public Task[] taskList;
	public ArrayList<Task> taskSetupList;
	
	private void start() {
		//dump our taskSetupList into our actual task list, prepare for actual task execution phase
		taskList = null;
		if(taskSetupList == null) {
			taskSetupList = new ArrayList();
		}
		taskList = taskSetupList.toArray(new Task[taskSetupList.size()]);
		taskSetupList = null;
	}
	
	public void stop() {
		clearTasks();
	}
	
	public void addTask(Task task) {
		if(taskSetupList == null) {
			taskSetupList = new ArrayList();
		}
		taskSetupList.add(task);
		RobotLogger.toast("Adding New Task: " + task.toString());
	}
	
	private void clearTasks() {
		if(taskList != null) {
			for(int i=0; i<taskList.length; i++) {
				if(taskList[i] != null) {
					taskList[i].destroy(); //let task perform any cleanup it needs
					if(taskList[i].executed) {
						RobotLogger.toast("removing task: " + taskList[i].toString() + " due to force clear!");
						taskList[i] = null;
					}
				}
			}
		}
	}

	public void forceClearTasks() {
		taskList = null;
	}
	
	public void resetTasks() {
		if(taskList != null) {
			for(int i=0; i<taskList.length; i++) {
				if(taskList[i] != null) {
					taskList[i].finished = false;
				}
			}
		}
	}
	
	
	public void autonomousStart() {
		if(currentMode != 1) {
			stop();
			currentMode = 1;
		}
		resetTasks();
		start();
	}
	
	public void teleopStart() {
		if(currentMode != 2) {
			stop();
			currentMode = 2;
		}
		resetTasks();
		start();
	}
	
	public void executeDT(double timestamp, boolean rt) {
		for(int i=0; i<taskList.length; i++) {
			Task task = taskList[i];
			if(task != null) {
				if((task.executionTime < timestamp) && task.allowedRun) {
					if((rt == false) || (rt && task.realtimeTask)) {
						boolean blocking = false;
						
						if(task.executed == false) {
							task.realExecutionTime = timestamp;
							task.init(timestamp - task.realExecutionTime);
						}
		
						task.execute(timestamp - task.realExecutionTime);
						
						task.setExecuted();
						blocking = task.blocking();
						
						if(task.autoRemove || task.finished) {
							taskList[i] = null;
							RobotLogger.toast("Task: " + task + " is finished/removed at " + timestamp);
						}
						if(blocking) {
							break;
						}
					}
				}
			}
		}
	}
	
}
