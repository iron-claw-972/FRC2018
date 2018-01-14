package org.usfirst.frc.team972.robot.motionlib;

import java.util.ArrayList;

public class TrajectoryGeneration {

    public static Trajectory generate(double max_velo, double max_accel, double finalPosition, double init_heading, double final_heading, double DT) {
        double POSITION = finalPosition;
        double MAX_VELOCITY = max_velo;
        double ACCEL = max_accel;

        double JERK = Math.pow(ACCEL, 1.25);
        double JOUNCE = Math.pow(JERK, 1.25);
        double CRACKLE = Math.pow(JOUNCE, 1.25);

        double Xpeak[] = {POSITION, MAX_VELOCITY, ACCEL, JERK, JOUNCE, CRACKLE};
        double T[] = {0, 0, 0, 0, 0, 0};

        SCurveGeneration.computePeriods(Xpeak, T);

        ArrayList<Double> values = new ArrayList<>();
        ArrayList<Double> velocity = new ArrayList<>();
        ArrayList<Double> acceleration = new ArrayList<>();
        ArrayList<Double> jerk = new ArrayList<>();
        ArrayList<Double> jounce = new ArrayList<>();
        ArrayList<Double> crackle = new ArrayList<>();

        double lastValueVelo = 0;
        double lastValueAccel = 0;
        double lastValueJerk = 0;
        double lastValueJounce = 0;
        double lastValueCrackle = 0;
        int time_in_dt = 0;

        for (int i = 1; i <= 50000; i++) {
            double point = SCurveGeneration.getSetpoint(Xpeak, T, DT * i);
            double delta = (point - lastValueVelo)/DT;
            double delta_accel = ((delta - lastValueAccel)/DT);
            double delta_jerk = ((delta_accel - lastValueJerk)/DT);
            double delta_jounce = ((delta_jerk - lastValueJounce)/DT);
            double delta_crackle = ((delta_jounce - lastValueCrackle)/DT);
            time_in_dt++;
            values.add(point);
            velocity.add(delta);
            acceleration.add(delta_accel);
            jerk.add(delta_jerk);
            jounce.add(delta_jounce);
            crackle.add(delta_crackle);

            lastValueVelo = point;
            lastValueAccel = delta;
            lastValueJerk = delta_accel;
            lastValueJounce = delta_jerk;
            lastValueCrackle = delta_jounce;

            if(delta == 0) {
                break;
            }
        }

        Trajectory trajectory = new Trajectory(time_in_dt);
        Trajectory.Segment[] segments = new Trajectory.Segment[time_in_dt];
        double total_heading_change = final_heading - init_heading;
        for(int i=0; i<time_in_dt; i++) {
            segments[i] = new Trajectory.Segment();
            segments[i].vel = velocity.get(i);
            segments[i].acc = acceleration.get(i);
            segments[i].jerk = jerk.get(i);
            segments[i].pos = values.get(i);
            segments[i].dt = DT;
            segments[i].heading = init_heading + total_heading_change * (segments[i].pos) / finalPosition;
        }
        trajectory.segments_ = segments;

        return trajectory;
    }
}
