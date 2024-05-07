package org.firstinspires.ftc.teamcode;

import java.util.*;

public class Path {
    private final ArrayList<Point> controlPoints = new ArrayList<>();
    public Path(int numPoints, double x, double y) {
        this.numPoints = numPoints;
        this.addControlPoint(x,y);
    }
    private int numPoints;
    public void addControlPoint(double x, double y) {
        controlPoints.add(new Point(x, y));
    }

    private Point calculatePointOnSpline(double t, Point p0, Point p1, Point p2, Point p3) {
        double t2 = t * t;
        double t3 = t2 * t;
        double c0 = -0.5 * t3 + t2 - 0.5 * t;
        double c1 = 1.5 * t3 - 2.5 * t2 + 1;
        double c2 = -1.5 * t3 + 2 * t2 + 0.5 * t;
        double c3 = 0.5 * t3 - 0.5 * t2;
        double x = (c0 * p0.x + c1 * p1.x + c2 * p2.x + c3 * p3.x);
        double y = (c0 * p0.y + c1 * p1.y + c2 * p2.y + c3 * p3.y);
        return new Point(x, y);
    }

    public ArrayList<Point> get_Path() {
        ArrayList<Point> Spline = new ArrayList<>();
        if (controlPoints.size() >= 2) {
            // Add control points for smooth connections at the beginning and end

            ArrayList<Point> extendedPoints = new ArrayList<>();
            extendedPoints.add(controlPoints.get(0));
            extendedPoints.addAll(controlPoints);
            extendedPoints.add(controlPoints.get(controlPoints.size() - 1));

            for (int i = 0; i < extendedPoints.size() - 3; i++) {
                Point p0 = extendedPoints.get(i);
                Point p1 = extendedPoints.get(i + 1);
                Point p2 = extendedPoints.get(i + 2);
                Point p3 = extendedPoints.get(i + 3);
                for (int j = 0; j <= numPoints-1; j++) {
                    double t = (double) j / numPoints;
                    Point p = calculatePointOnSpline(t, p0, p1, p2, p3);
                    double t1 = (double) (j + 1) / numPoints;
                    Point pNext = calculatePointOnSpline(t1, p0, p1, p2, p3);
                    double t2 = (double) (j + 2) / numPoints;
                    Point pNextNext = calculatePointOnSpline(t2, p0, p1, p2, p3);
                    double dx = pNext.x - p.x;
                    double dy = pNext.y - p.y;
                    double theta1 = Math.atan2(dx, dy);
                    double mag1 = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
                    p.setFirst_derivative(new Vector(theta1 ,mag1));
                    double d2x = pNextNext.x - 2 * pNext.x + p.x;
                    double d2y = pNextNext.y - 2 * pNext.y + p.y;
                    double theta2 = Math.atan2(d2x,d2y);
                    double mag2 = Math.sqrt(Math.pow(d2x, 2) + Math.pow(d2y, 2));
                    p.setSecond_derivative(new Vector(theta2, mag2));
                    Spline.add(p);
                }
            }
        }
        Spline.add(controlPoints.get(controlPoints.size() - 1));
        return Spline;
    }
}
