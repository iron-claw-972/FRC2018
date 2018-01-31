package org.usfirst.frc.team972.robot.motionlib;

public class TrapezoidalMotionProfile {
	private float maxVelocity = 0;
	private float maxAcceleration = 0;
	private float position = 0;
	private float oldPosition = 0;
	private float velocity = 0;
	private float oldVelocity = 0;
	private float acceleration = 0;
	private float lastTime = 0;
	private float delta = 0;
	
	public TrapezoidalMotionProfile(float _maxVelocity, float _maxAcceleration) {
		maxVelocity = _maxVelocity;
		maxAcceleration = _maxAcceleration;
	}
	
	private boolean timeCalculation(float time) {
		if(this.lastTime == 0) {
			lastTime = time;
			return false;
		} else {
			this.delta = (time - this.lastTime);
			this.lastTime = time;
			return true;
		}
	}
	
	public void update(float setpoint, float time) {
		boolean timeValid = timeCalculation(time);
		
		this.oldPosition = this.position;
		this.oldVelocity = this.velocity;
		
		if(timeValid) {
			this.timeCalculation(setpoint);
		}
		
		stateCalculation();
	}
	
	private void stateCalculation() {
		this.velocity = (this.position - this.oldPosition) / this.delta;
		this.acceleration = (this.velocity - this.oldVelocity) / this.delta;
	}
	
	private void calculateTrapezodialProfile(float setpoint) {
		// Check if we need to de-accelerate
		if (((velocity * velocity) / maxAcceleration) / 2 >= Math.abs(setpoint - position)) {
			if (velocity < 0) {
				position += (velocity + maxAcceleration * delta) * delta;
			}
			else if (velocity > 0) {
				position += (velocity - maxAcceleration * delta) * delta;
			}
		}
		else {
			// We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
			if (Math.abs(velocity) < maxVelocity || (setpoint < position && velocity > 0) || (setpoint > position && velocity < 0)) {
				// We need to accelerate, do so but check the maximum acceleration.
				// Keep velocity constant at the maximum
				float suggestedVelocity = 0.0f;
				if (setpoint > position) {
					suggestedVelocity = velocity + maxAcceleration * delta;
					if (suggestedVelocity > maxVelocity) {
						suggestedVelocity = maxVelocity;
					}
				}
				else {
					suggestedVelocity = velocity - maxAcceleration * delta;
					if (Math.abs(suggestedVelocity) > maxVelocity) {
						suggestedVelocity = -maxVelocity;
					}				
				}
				position += suggestedVelocity * delta;
			}
			else {
				// Keep velocity constant at the maximum
				if (setpoint > position) {
					position += maxVelocity * delta;
				}
				else {
					position += -maxVelocity * delta;
				}
			}
		}
	}
	
}
