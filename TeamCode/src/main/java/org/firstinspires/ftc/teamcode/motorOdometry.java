package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class motorOdometry {
    final private double xMultiplier = 1.0;
    final private double yMultiplier = 1.0;
    final private double wheel_theta = Math.atan2(xMultiplier, yMultiplier);
    private double x;
    private double y;
    private double heading;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    Telemetry telemetry;
    private int current_left_front_pos = 0;
    private int current_right_front_pos = 0;
    private int current_left_back_pos = 0;
    private int current_right_back_pos = 0;
    private int past_left_front_pos = current_left_front_pos;
    private int past_right_front_pos = current_right_front_pos;
    private int past_left_back_pos = current_left_back_pos;
    private int past_right_back_pos = current_right_back_pos;
    public static double track_width = 10;//fill in value with horizontal distance from center of robot's left and right wheels
    private double delta_L_F;
    private double delta_R_F;
    private double delta_L_B;
    private double delta_R_B;
    private double deltaHeading;
    private static double ENCODER_COUNTS_PER_INCH = 537.7/(2*Math.PI*1.88976);
    private BHI260IMU imu;

    public motorOdometry (DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB, double starting_x, double starting_y, double starting_heading, Telemetry telemetry, BHI260IMU imu) {
        this.leftFront = LF;
        this.leftBack = LB;
        this.rightFront = RF;
        this.rightBack = RB;
        this.telemetry = telemetry;
        this.x = starting_x;
        this.y = starting_y;
        this.heading = Math.toRadians(starting_heading);
        this.imu = imu;
    }

    public void update () {
        current_left_front_pos = leftFront.getCurrentPosition();
        current_right_front_pos = rightFront.getCurrentPosition();
        current_left_back_pos = leftBack.getCurrentPosition();
        current_right_back_pos = rightBack.getCurrentPosition();

        delta_L_F = current_left_front_pos - past_left_front_pos;
        delta_L_B = current_left_back_pos - past_left_back_pos;
        delta_R_F = current_right_front_pos - past_right_front_pos;
        delta_R_B = current_right_back_pos - past_right_back_pos;

        deltaHeading = (delta_L_F+delta_L_B-delta_R_F-delta_R_B)/(2.0*track_width*ENCODER_COUNTS_PER_INCH);

        heading = normalizeRadians(heading+deltaHeading);

        double turn_ticks = deltaHeading*track_width*ENCODER_COUNTS_PER_INCH*2.0;

        double deltaStr = ((delta_L_F - turn_ticks)*Math.sin(wheel_theta) + (delta_L_B - turn_ticks)*Math.sin(wheel_theta) +
                (delta_R_F - turn_ticks)*Math.sin(wheel_theta) + (delta_R_B - turn_ticks)*Math.sin(wheel_theta)) / 4;

        double deltaFwd = ((delta_L_F - turn_ticks)*Math.cos(wheel_theta) + (delta_L_B - turn_ticks)*Math.cos(wheel_theta) +
                (delta_R_F - turn_ticks)*Math.cos(wheel_theta) + (delta_R_B - turn_ticks)*Math.cos(wheel_theta)) / 4;

        double r0 = deltaFwd/(deltaHeading);
        double r1 = deltaStr/(deltaHeading);

        double relDeltaX = r0*sin(deltaHeading) - r1*(1-cos(deltaHeading));
        double relDeltaY = r1*sin(deltaHeading) + r0*(1-cos(deltaHeading));

        x+=relDeltaX*cos(heading) - relDeltaY*sin(heading);
        y+=relDeltaY*cos(heading) + relDeltaX*sin(heading);

        past_left_front_pos = current_left_front_pos;
        past_left_back_pos = current_left_back_pos;
        past_right_front_pos = current_right_front_pos;
        past_right_back_pos = current_right_back_pos;
    }

    public double[] getPose () {
        double[] pos = {x/ENCODER_COUNTS_PER_INCH, y/ENCODER_COUNTS_PER_INCH, -(Math.toDegrees(heading)-90)};//return the values corrected for inches and degrees
        return pos;
    }
}



