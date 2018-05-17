package org.usfirst.frc.team972.robot;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.usfirst.frc.team972.robot.executor.auto.ControlElevatorTask;
import org.usfirst.frc.team972.robot.executor.auto.ControlIntakeArmTask;
import org.usfirst.frc.team972.robot.motors.MainDriveTrain;
import org.usfirst.frc.team972.robot.motors.MechanismActuators;
import org.usfirst.frc.team972.robot.ui.Sensors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class SystemTest {

    private boolean inTest = false;
    Sensors sensors;
    MainDriveTrain mdt;
    MechanismActuators motors;

    ControlIntakeArmTask intakeArmTask;
    ControlElevatorTask elevatorTask;

    double intakeMotorCurrent = 1;
    double armForwardPosition = 0.8;
    double elevatorUpPosition = 0.25;

    //weird cap conventions
    int DRIVE_TRAIN_ENCODER_LIMIT = 2048;
    int MAX_ARM_CURRENT_COMBINED = 10;
    int MAX_ELEVATOR_CURRENT = 10;
    double MAX_ARM_ERROR = 0.1;
    double MAX_ELEVATOR_ERROR = 0.1;
    int IN_THE_GOOD_MAX = 250;

    Map<String, Test> tests;

    public SystemTest(Sensors _sensors, MainDriveTrain _mdt, MechanismActuators _motors) {
    	sensors = _sensors;
    	mdt = _mdt;
    	motors = _motors;
    	
        elevatorTask = new ControlElevatorTask(0, motors, sensors);
        intakeArmTask = new ControlIntakeArmTask(0, sensors, motors); //this is great code practice (it's not)

        tests = new LinkedHashMap<>();

        RobotLogger.toast("Adding Tests...");
        		
        tests.put("drive forward", new Test() {
            public boolean run() {
                sensors.resetDriveEncoder();
                mdt.driveSidesPWM(0.5, 0.5);
                sleep(500);
                if ((sensors.getLeftDriveEncoder() > DRIVE_TRAIN_ENCODER_LIMIT) && (-sensors.getRightDriveEncoder() > DRIVE_TRAIN_ENCODER_LIMIT)) {
                    return true;
                } else {
                    return false;
                }
            }
        });

        tests.put("drive backward", new Test() {
            public boolean run() {
                sensors.resetDriveEncoder();
                mdt.driveSidesPWM(-0.5, -0.5);
                sleep(500);
                if ((sensors.getLeftDriveEncoder() < -DRIVE_TRAIN_ENCODER_LIMIT) && (-sensors.getRightDriveEncoder() < -DRIVE_TRAIN_ENCODER_LIMIT)) {
                    return true;
                } else {
                    return false;
                }
            }
        });    
        
        tests.put("turn on intake", new Test() {
            public boolean run() {
                motors.RunIntakeMotors(0.5);
                return true; //don't care about result
            }
        });
        
        tests.put("move arm forward", new Test() {
            double realStartTime = Timer.getFPGATimestamp();
            int inTheGood = 0;
            
            public boolean run() {
                intakeArmTask.setPositionTarget(0, 0);
                for(int i=0; i<1000; i++) {
                	if(DriverStation.getInstance().isDisabled()) break;
                    if((motors.intakeArmMotorLeft.getOutputCurrent() + motors.intakeArmMotorRight.getOutputCurrent()) > MAX_ARM_CURRENT_COMBINED) {
                        this.abort = true;
                        return false;
                    }
                    if(i > 50) {
                    	intakeArmTask.setPositionTarget(armForwardPosition, -armForwardPosition);
                    }
                    double current_time = Timer.getFPGATimestamp() - realStartTime;
                    intakeArmTask.execute(current_time);
                    
                    if(intakeArmTask.isSafeOutwards()) {
                        inTheGood++;
                    } else {
                        inTheGood = 0;
                    }

                    if(inTheGood > IN_THE_GOOD_MAX) { //we don't want to end this test immediate when we are in a good range
                        return true;
                    }

                    sleep((long) (1000 / Robot.REAL_TIME_LOOP_HZ));
                }
                return false; //we did not get there in time
            }
        });
        
        tests.put("measure intake", new Test() {
            public boolean run() {
                double currentLeft = motors.intakeMotorLeft.getOutputCurrent();
                double currentRight = motors.intakeMotorRight.getOutputCurrent();
                motors.RunIntakeMotors(0);
                if((currentLeft > intakeMotorCurrent) && (currentRight > intakeMotorCurrent)) {
                    return true;
                } else {
                    return false;
                }
            }
        });
        
        tests.put("move arm backward", new Test() {
            double realStartTime = Timer.getFPGATimestamp();
            int inTheGood = 0;
            public boolean run() {
                for(int i=0; i<1000; i++) {
                	if(DriverStation.getInstance().isDisabled()) break;
                    if((motors.intakeArmMotorLeft.getOutputCurrent() + motors.intakeArmMotorRight.getOutputCurrent()) > MAX_ARM_CURRENT_COMBINED) {
                        this.abort = true;
                        return false;
                    }
                    
                    if(i > 50) {
                    	intakeArmTask.setPositionTarget(0, 0);
                    }
                    
                    double current_time = Timer.getFPGATimestamp() - realStartTime;
                    intakeArmTask.execute(current_time);
                    
                    if(intakeArmTask.isSafeOutwards() == false) {
                        inTheGood++;
                    } else {
                        inTheGood = 0;
                    }

                    if(inTheGood > IN_THE_GOOD_MAX) { //we don't want to end this test immediate when we are in a good range
                        return true;
                    }

                    sleep((long) (1000 / Robot.REAL_TIME_LOOP_HZ));
                }
                return false; //we did not get there in time
            }
        });

        /*
        //fake boilerplate code
        tests.put("move elevator up", new Test() {
            double startTime = System.currentTimeMillis();
            int inTheGood = 0;
            public boolean run() {
                elevatorTask.setElevatorPositionTarget(elevatorUpPosition);
                for(int i=0; i<5000; i++) {
                    if(motors.elevatorLiftMotor.getOutputCurrent() > MAX_ELEVATOR_CURRENT) {
                        this.abort = true;
                        return false;
                    }
                    elevatorTask.execute(System.currentTimeMillis() - startTime);

                    if(Math.abs(elevatorUpPosition - elevatorTask.getPosition()) < MAX_ELEVATOR_ERROR) {
                        inTheGood++;
                    } else {
                        inTheGood = 0;
                    }

                    if(inTheGood > IN_THE_GOOD_MAX) { //we don't want to end this test immediate when we are in a good range
                        return true;
                    }

                    sleep(1);
                }
                return false; //we did not get there in time
            }
        });
		*/
	
        //TODO: write ELEVATOR DOWN and then ARM BACK!!!
        //TODO: not writing ATM because i want to see if my tests actually work

    }

    public void sleep(long time) {
        try {
            Thread.sleep(time);
        } catch (Exception e) {
            RobotLogger.toast("Sleep Interrupted during System Test. This should be fine?");
        }
    }

    public void beginTest() {
        try {
        	
            //sensors.resetDriveEncoder();
            //sensors.resetElevatorEncoder();
            sensors.resetIntakeEncoders();

            sleep(500);

            int currentTest = 0;
            int numOfTests = tests.size();

            int testPassed = 0;
            int testFailed = 0;
            long testTimeStart = System.currentTimeMillis();
            for(Map.Entry<String, Test> testMapEntry : tests.entrySet()) {
                String testName = testMapEntry.getKey();
                Test test = testMapEntry.getValue();
                sleep(100);
                RobotLogger.toast("Performing Test (" + currentTest + "/" + numOfTests + "): " + testName);
                boolean result = test.run();
                if(result) {
                    RobotLogger.toast(testName + " PASSED!");
                    test.result = "PASSED";
                    testPassed++;
                } else {
                    RobotLogger.toast(testName + " FAILED!");
                    test.result = "FAILED";
                    testFailed++;
                }

                if(test.abort) {
                    //abort because one of our tests failed and if we continue we risk damaging stuff
                    RobotLogger.toast("!!!TEST ABORT!!!", RobotLogger.URGENT);
                    RobotLogger.toast("!!!TEST ABORT!!!");
                    RobotLogger.toast("!!!TEST ABORT!!!");

                    mdt.driveSidesPWM(0, 0);
                    motors.RunElevatorLiftMotor(0);
                    motors.RunIntakeArmMotors(0, 0);
                    motors.RunIntakeMotors(0);

                    RobotLogger.toast("ABORTED DUE TO: " + testName);

                    //if things are really bad ... then we can System.exit(0);

                    break;
                }

                currentTest++;
            }
            sleep(100);
            System.out.println("\n\n\n--------------------\n\n\n"); //show test results
            System.out.println(currentTest + " tests executed in " + (System.currentTimeMillis() - testTimeStart) + " ms");
            System.out.println(testPassed + " passes");
            System.out.println(testFailed + " fails");
            System.out.println(numOfTests - currentTest + " ? untested");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

abstract class Test {
    String result = "NOT_RAN";
    boolean abort = false;
    abstract public boolean run();
}