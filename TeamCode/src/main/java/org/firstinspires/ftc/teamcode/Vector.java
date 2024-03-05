package org.firstinspires.ftc.teamcode;

public class Vector {
    public double theta;
    public double magnitude;

    Vector(double theta, double magnitude) {
        this.theta = theta;
        this.magnitude = magnitude;
    }

    Vector(double x, double y, double magnitude) {
        this.theta = Math.atan2(x,y);
        this.magnitude = magnitude;
    }
}
