package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import java.util.ArrayList;

public class PathFollower {
    public Drivetrain drivetrain;
    public Path path;
    ArrayList<Point> path_to_follow;
    Point currentPoint;
    int i;
    double currentVelocity;
    double yPower;
    double xPower;
    double turnPower;
    double angle_to_point;
    PIDController pidX = new PIDController(0.087, 0.041, 0.015);
    PIDController pidY = new PIDController(0.082, 0.041, 0.0175);
    PIDController pidH = new PIDController(0.05, 0.005, 0);

    public PathFollower (Drivetrain drivetrain, Path path) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.i=0;
        this.path_to_follow = path.get_Path();
        this.currentPoint = path_to_follow.get(i);
    }

    public boolean followForward (double [] current_pos, double endHeading, ElapsedTime totalRuntime) {
        angle_to_point = Math.atan2(path_to_follow.get(i).x - current_pos[0], path_to_follow.get(i).y - current_pos[1]);
        return update(current_pos, Math.toDegrees(angle_to_point), endHeading, totalRuntime);
    }

    public boolean followBackward (double [] current_pos, double endHeading, ElapsedTime totalRuntime) {
        angle_to_point = Math.atan2(path_to_follow.get(i).x - current_pos[0], path_to_follow.get(i).y - current_pos[1]);
        return update(current_pos, normalizeDegrees(Math.toDegrees(angle_to_point)-180), endHeading, totalRuntime);
    }

    public boolean followHoldHeading (double [] current_pos, double holdHeading, double endHeading, ElapsedTime totalRuntime) {
        return update(current_pos, holdHeading, endHeading, totalRuntime);
    }

    public boolean update (double [] current_pos,double holdHeading, double endHeading, ElapsedTime totalRuntime) {
        if(currentPoint.dist_to_point(current_pos[0], current_pos[1]) < 3.5 && i < path_to_follow.size()-1) {
            i = i+1;
            currentPoint = path_to_follow.get(i);
        }

        if(i < path_to_follow.size()-1) {
            currentVelocity = currentPoint.first_derivative.magnitude/path.getMaxVelocity();
            angle_to_point = Math.atan2(path_to_follow.get(i).x - current_pos[0], path_to_follow.get(i).y - current_pos[1]);
            yPower = Math.cos(angle_to_point) * 0.8;
            xPower = Math.sin(angle_to_point) * 0.8;
            turnPower = pidH.calculate(current_pos[2], holdHeading);
        } else {
            xPower = pidX.calculate(current_pos[0], currentPoint.x);
            yPower = pidY.calculate(current_pos[1], currentPoint.y);
            turnPower = pidH.calculate(current_pos[2], endHeading);
        }

        drivetrain.moveRobot(xPower, -yPower, turnPower, -Math.toRadians(current_pos[2]));
        if((currentPoint.dist_to_point(current_pos[0], current_pos[1]) < 0.5 && i == path_to_follow.size()-1 && Math.abs(current_pos[2] - endHeading) < 1) || totalRuntime.seconds()>30) {
           return true;
        } else {
            return false;
        }
    }
}
