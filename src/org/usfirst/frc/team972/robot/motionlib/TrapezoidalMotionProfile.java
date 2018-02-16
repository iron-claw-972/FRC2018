package org.usfirst.frc.team972.robot.motionlib;

public class TrapezoidalMotionProfile {
	
  private double maxVelocity = 0;
  private double maxAcceleration = 0;
  public double position = 0;
  private double oldPosition = 0;
  public double velocity = 0;
  private double oldVelocity = 0;
  public double acceleration = 0;
  private double lastTime = -1; //start at -1 so we dont have weird timing calc error
  private double delta = 0;
  
  public TrapezoidalMotionProfile(double _maxVelocity, double _maxAcceleration) {
    maxVelocity = _maxVelocity;
    maxAcceleration = _maxAcceleration;
  }
  
  public void setRealPositions(double p) {
	  position = p;
	  oldPosition = p;
	  velocity = 0;
	  oldVelocity = 0;
  }
  
  private boolean timeCalculation(double time) {
    if(this.lastTime == 0) {
      lastTime = time;
      return false;
    } else {
      this.delta = (time - this.lastTime);
      this.lastTime = time;
      return true;
    }
  }
  
  public void update(double setpoint, double dt) {
    boolean timeValid = timeCalculation(dt);
    
    this.oldPosition = this.position;
    this.oldVelocity = this.velocity;
    
    if(timeValid) {
      this.calculateTrapezodialProfile(setpoint);
    }
    
    stateCalculation();
  }
  
  public void setTime(double t) {
	  lastTime = t;
  }
  
  private void stateCalculation() {
    this.velocity = (this.position - this.oldPosition) / this.delta;
    this.acceleration = (this.velocity - this.oldVelocity) / this.delta;
  }
  
  private void calculateTrapezodialProfile(double setpoint) {
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
        double suggestedVelocity = 0.0f;
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
