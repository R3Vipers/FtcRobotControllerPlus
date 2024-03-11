package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.E;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class odometry {
    private double x;
    private double y;
    private double heading;
    private DcMotor left_encoder;
    private DcMotor right_encoder;
    private DcMotor strafe_encoder;
    Telemetry telemetry;
    private int current_left_pos = 0;
    private int current_right_pos = 0;
    private int current_strafe_pos = 0;
    private int past_left_pos = current_left_pos;
    private int past_right_pos = current_right_pos;
    private int past_strafe_pos = current_strafe_pos;
    private double track_width = 10.16;//fill in value with horizontal distance from center of robot to the left encoder
    private double distance_to_strafe_encoder = -4.95;//fill in value with vertical distance from center of robot to the strafe encoder
    private double delta_L;
    private double delta_R;
    private double delta_S;
    private double deltaHeading;
    private static double ENCODER_COUNTS_PER_INCH = 2000/(2*Math.PI*0.945);
    private BHI260IMU imu;

    public odometry(DcMotor left_encoder, DcMotor right_encoder, DcMotor strafe_encoder, double starting_x, double starting_y, double starting_heading, Telemetry telemetry, BHI260IMU imu) {
        this.left_encoder = left_encoder;
        this.right_encoder = right_encoder;
        this.strafe_encoder = strafe_encoder;
        this.telemetry = telemetry;
        this.x = starting_x;
        this.y = starting_y;
        this.heading = starting_heading;
        this.imu = imu;
    }

    public void update () {
        current_left_pos = -left_encoder.getCurrentPosition();
        current_right_pos = right_encoder.getCurrentPosition();
        current_strafe_pos = -strafe_encoder.getCurrentPosition();

        delta_L = current_left_pos - past_left_pos;
        delta_R = current_right_pos - past_right_pos;
        delta_S = current_strafe_pos - past_strafe_pos;

        deltaHeading = (delta_R-delta_L)/(track_width*ENCODER_COUNTS_PER_INCH);

        heading = normalizeRadians(heading+deltaHeading);

        double deltaFwd = (delta_R+delta_L)/2;
        double deltaStr = (delta_S - distance_to_strafe_encoder*deltaHeading*ENCODER_COUNTS_PER_INCH);
// use for tuning
//        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//        double angle = angles.firstAngle;
        //telemetry.addData("angle", "%f", angle);
//        telemetry.addData("track width", "%f", (current_right_pos - current_left_pos)/(angle*ENCODER_COUNTS_PER_INCH));
//        telemetry.addData("dist. to back encoder", "%f", current_strafe_pos/(angle*ENCODER_COUNTS_PER_INCH));

        if(abs(deltaHeading)!=0) {

            double r0 = deltaFwd/(deltaHeading);
            double r1 = deltaStr/(deltaHeading);

            double relDeltaX = r0*sin(deltaHeading) - r1*(1-cos(deltaHeading));
            double relDeltaY = r1*sin(deltaHeading) + r0*(1-cos(deltaHeading));

            x+=relDeltaX*cos(heading) - relDeltaY*sin(heading);
            y+=relDeltaY*cos(heading) + relDeltaX*sin(heading);
        } else {
            double relDeltaX = deltaFwd;
            double relDeltaY = deltaStr;

            x+=relDeltaX*cos(heading) - relDeltaY*sin(heading);
            y+=relDeltaY*cos(heading) + relDeltaX*sin(heading);
        }

        past_left_pos = current_left_pos;
        past_right_pos = current_right_pos;
        past_strafe_pos = current_strafe_pos;
    }

    public double[] getPose () {
        double[] pos = {x/ENCODER_COUNTS_PER_INCH, -y/ENCODER_COUNTS_PER_INCH, -Math.toDegrees(heading)};
        return pos;
    }
}