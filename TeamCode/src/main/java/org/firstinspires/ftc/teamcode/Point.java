package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;

public class Point {
    public double x;
    public double y;
    public Vector first_derivative;

    public Vector second_derivative;

    public void setFirst_derivative (Vector d_vec) {this.first_derivative = d_vec;}

    public void setSecond_derivative(Vector d2_vec) {
        this.second_derivative = d2_vec;
    }

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, Vector first_derivative){
        this.x = x;
        this.y = y;
        this.first_derivative = first_derivative;
    }

    public Point(double x, double y, Vector first_derivative, Vector second_derivative){
        this.x = x;
        this.y = y;
        this.first_derivative = first_derivative;
        this.second_derivative = second_derivative;
    }

    public double dist_to_point (double currentX, double currentY) {
        return Math.sqrt(Math.pow(this.x - currentX, 2) + Math.pow(this.y - currentY, 2));
    }
}
