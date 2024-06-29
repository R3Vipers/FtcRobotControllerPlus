package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

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
    public static double track_width = 10.16;//fill in value with horizontal distance from center of the robot's two parallel encoders
    public static double distance_to_strafe_encoder = -4.95;//fill in value with vertical distance from center of robot to the strafe encoder
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
        this.heading = Math.toRadians(starting_heading);
        this.imu = imu;
    }

    public void update () {
        current_left_pos = -left_encoder.getCurrentPosition();//left encoder is negative because it is technically mounted backwards, but this may not always be true
        current_right_pos = right_encoder.getCurrentPosition();
        current_strafe_pos = -strafe_encoder.getCurrentPosition();//strafe encoder is negative because of mounting reasons, but this may not be true for all robots. This will impact the direction of the heading.

        delta_L = current_left_pos - past_left_pos;// delta is the same as saying change in so we are looking for the change in the encoder values which is equal to current - past
        delta_R = current_right_pos - past_right_pos;
        delta_S = current_strafe_pos - past_strafe_pos;

        deltaHeading = (delta_R-delta_L)/(track_width*ENCODER_COUNTS_PER_INCH); //theta = s/r, where s = arc length and r = radius of circle
                                                                                //since delta_L is negative because the left encoder returns a negative position, deltaR-deltaL = 2*deltaR
        //delta_R is the distance traveled by right encoder therefore we will set this to s
        //track width is equal to 2r since it is the diameter of the turing circle of the parallel encoders
        //so we have 2s/2r which is equal to s/r and we must cancel out all other units and multiply by 2*Pi to get the heading in radians
        //one radian is the circumference of a circle with a radius of 1 (2*Pi*r, where r = 1, equals 2*Pi)

        heading = normalizeRadians(heading+deltaHeading); // by normalizing the heading we make it so that heading goes from + or - 180 degrees
        // this is not needed, but if the heading is not normalized the heading will keep accumulating

        double deltaFwd = (delta_R+delta_L)/2; // if you add the two together and divided by two you get the average distance traveled by each encoder
        //if the robot traveled completely straight delta_r would be equal to delta_L thus the numerator would be equal to 2*delta_R
        //thus th equations would equal delta-R or the distance traveled forward
        double deltaStr = (delta_S - distance_to_strafe_encoder*deltaHeading*ENCODER_COUNTS_PER_INCH); // The strafe encoder is affected by strafe movement and
        // turning movement. therefore to find the distance strafed it is necessary to subtract the amount of change due to turning.
        //from before we know the s or distance traveled in a circle is equal to r*theta. We have calculated theta before, which is delta heading,
        //and we have r from distance_to_strafe_encoder. therefore with some unit conversions we can calculate how much the strafe encode has moved
        //due to turning and thus find out how much the strafe encoder has moved due to strafing movement.
// use for tuning
        //The odometry works off of the measurements of the position of the wheels. first get a rough measurement of the distance between the
        //two parallel encoders. Next determine the center of rotation of the robot or the point where your spins about. Measure the vertical
        //distance from the horizontal encoder to the center of rotation. Put these numbers into the variables track_width, and
        //distance_to_strafe_encoder. Now we need to tune these distances. The measurements before were rough estimates no we need to determine
        //the exact distances to make the math accurate to real values. To start uncomment the telemetry output below and the imu code in
        // the init function, then follow the steps below.
        //first use the imu to tune the track width. Turn about 90 degrees and read the number and input that into the variable named track_width.
        //Next repeat the same tuning process until the track_width stays constant and the heading given by the imu and the heading given by the odometry
        //start to line up. Then tune the distance to the back encoder repeat the same process of turning and changing the number. You will know
        //if everything is tuned because when turing the robot, the x,y position given by the odometry should not change.
//        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//        double angle = angles.firstAngle;
//        telemetry.addData("angle", "%f", angle);
//        telemetry.addData("track width", "%f", (current_right_pos - current_left_pos)/(angle*ENCODER_COUNTS_PER_INCH));
//        telemetry.addData("dist. to back encoder", "%f", current_strafe_pos/(angle*ENCODER_COUNTS_PER_INCH));

        if(abs(deltaHeading) > 0.001) { // there are two types of odometry, one is linear odometry and the other is arc based odometry.
            //acr based odometry work on the idea that robot moves in arcs rater than lines.
            //while line and arc based odometry both work fine, it actually turns out that arc based odometry converges to zero error faster
            //than line based odometry. So why not just use arc based odometry? well if we only used arc based odometry, then we run into divide
            //by zero errors if there is no change in heading, i.e. moving in a straight line. This is because some of the odometry math use delta
            //heading in the denominator, so if there is no change in heading, the math return a NaN error.
            // to find derivations of math got to https://youtu.be/ixsxDn_ddLE?feature=shared and https://youtu.be/qqODIdvSGac?feature=shared

            double r0 = deltaFwd/(deltaHeading);
            double r1 = deltaStr/(deltaHeading);

            double relDeltaX = r0*sin(deltaHeading) - r1*(1-cos(deltaHeading));
            double relDeltaY = r1*sin(deltaHeading) + r0*(1-cos(deltaHeading));

            x+=relDeltaX*cos(heading) - relDeltaY*sin(heading);
            y+=relDeltaY*cos(heading) + relDeltaX*sin(heading);
        } else {
            // linear odometry works as its name implies. It assumes that the robot moves in straight lines with little to no rotation
            // since heading is constant we can use deltaFwd and deltaStr to determine how the robot has moved.
            // lets for example say that we are moving 10 inches to the right and ten inches forward and through out translation our heading is forward or 90 degrees.
            // Our heading is 90 degrees because 0 degrees is in the +x direction when using radian (refer to the unit circle), therefore positive 90 is in the +y direction
            // using the equations below x = change in forward (10) * cosine of 90, which is 0, minus change in strafe (10) * sin of 90, which is 1. this means,
            // that x would be equal to 0+10 = 10, which happens to be how much we strafed in the x direction.
            // When solving the equation for y with the example described above we find that y = 10, which is the same as the distance that we moved forward
            //This proves that the below equation can be used to describe the motion of the robot in straight lines.
            x+=deltaFwd*cos(heading) - deltaStr*sin(heading);
            y+=deltaStr*cos(heading) + deltaFwd*sin(heading);
        }

        past_left_pos = current_left_pos;//update path variable s with current variables, because they have already been used
        past_right_pos = current_right_pos;
        past_strafe_pos = current_strafe_pos;
    }

    public double[] getPose () {
        double[] pos = {x/ENCODER_COUNTS_PER_INCH, y/ENCODER_COUNTS_PER_INCH, -(Math.toDegrees(heading)-90)};//return the values corrected for inches and degrees
        return pos;
    }
}