package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class PathFollower {
    public Drivetrain drivetrain;
    public Path path;
    ArrayList<Point> path_to_follow;
    Point currentPoint;
    int i;

    double yPower;
    double xPower;
    double turnPower;
    double angle_to_point;
    PIDController pidX;
    PIDController pidY;
    PIDController pidH;

    public PathFollower (Drivetrain drivetrain, Path path) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.i=0;
        this.path_to_follow = path.get_Path();
        this.currentPoint = path_to_follow.get(i);
        pidX = new PIDController(0.083, 0.041, 0.016);
        pidY = new PIDController(0.082, 0.041, 0.0175);
        pidH = new PIDController(0.05, 0.005, 0);
    }

    public boolean update (double [] current_pos, ElapsedTime totalRuntime) {
        if(currentPoint.dist_to_point(current_pos[0], current_pos[1]) < 3.5 && i < path_to_follow.size()-1) {
            i = i+1;
            currentPoint = path_to_follow.get(i);
        }

        if(i < path_to_follow.size()-1) {
            angle_to_point = Math.atan2(path_to_follow.get(i).x - current_pos[0], path_to_follow.get(i).y - current_pos[1]);
            yPower = Math.cos(angle_to_point)*0.8;
            xPower = Math.sin(angle_to_point)*0.8;
        } else {
            xPower = pidX.calculate(current_pos[0], currentPoint.x);
            yPower = pidY.calculate(current_pos[1], currentPoint.y);
        }
        turnPower = pidH.calculate(current_pos[2], 0);

        drivetrain.moveRobot(xPower, -yPower, turnPower, -Math.toRadians(current_pos[2]));
        if((currentPoint.dist_to_point(current_pos[0], current_pos[1]) < 0.75 && i == path_to_follow.size()-1) || totalRuntime.seconds()>30) {
           return true;
        } else {
            return false;
        }
    }
}
