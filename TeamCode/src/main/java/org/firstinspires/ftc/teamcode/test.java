package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="test")
@Config
//@Disabled
public class test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private odometry odo = null;
    private Path path = null;
    ArrayList<Point> path_to_follow;
    List<LynxModule> allHubs;
    double [] current_pos;
    double yPower;
    double xPower;
    double turnPower;
    double angle_to_point;
    int i = 0;
    PIDController pidX;
    PIDController pidY;
    PIDController pidH;
    Point temp = new Point(0, 24);
    Point temp1 = new Point(24, 24);
    Point temp2 = new Point(24, 48);
    Point temp3 = new Point(0, 0);

    Point currentPoint;
    boolean wait = false;

    ArrayList<Point> Points = new ArrayList<>();

    // The IMU sensor object
    BHI260IMU imu;
    IMU.Parameters myIMUparameters;

    @Override
    public void init() {

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
            myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            )
        );

        allHubs = hardwareMap.getAll(LynxModule.class); // bulk reading hubs for faster loop speeds

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetry for ftc dashboard and driver station

        //hardware declarations
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor2");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motor1");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // odometry object takes motors as parameters
        odo = new odometry(leftBackDrive, rightBackDrive, rightFrontDrive, 0, 0, 90, telemetry, imu);
        //creating new path
        path = new Path(10, 0, 0); // number of points, starting x, starting y;
        path.addControlPoint(0, 24); // new point x, y
        path.addControlPoint(24, 24); // new point x, y
        path.addControlPoint(24, 48); // new point x, y
        path.addControlPoint(0, 0); // new point x, y
        path_to_follow = path.get_Path();
        pidX = new PIDController(0.083, 0.041, 0.016);
        pidY = new PIDController(0.082, 0.041, 0.0175);
        pidH = new PIDController(0.05, 0.005, 0);
        Points.add(temp);
        Points.add(temp1);
        Points.add(temp2);
        Points.add(temp3);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        currentPoint = path_to_follow.get(i);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //clearing the cache form all huds
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        current_pos = odo.getPose();
        telemetry.addData("x", "%.2f",current_pos[0]);//output of odometry for x
        telemetry.addData("y","%.2f", current_pos[1]);//output of odometry for y
        telemetry.addData("heading","%.2f", current_pos[2]);//output of odometry for heading
        //telemetry.addData("loop speed", "%f", (1/runtime.seconds()));//output of loop speed

        if(currentPoint.dist_to_point(current_pos[0], current_pos[1]) < 1 && i < path_to_follow.size()-1) {
            if(!wait) {
                wait = true;
                runtime.reset();
            } else {
                //if(runtime.seconds() > 3) {
                    i = i+1;
                    currentPoint = path_to_follow.get(i);
                    wait = false;
                //}
            }
        }

        if(i < path_to_follow.size()-1) {
            angle_to_point = Math.atan2(path_to_follow.get(i).x - current_pos[0], path_to_follow.get(i).y - current_pos[1]);
            yPower = Math.cos(angle_to_point)/2;
            xPower = Math.sin(angle_to_point)/2;
        } else {
            xPower = pidX.calculate(current_pos[0], currentPoint.x);
            yPower = pidY.calculate(current_pos[1], currentPoint.y);
        }
        turnPower = pidH.calculate(current_pos[2], 0);


        moveRobot(xPower, -yPower, turnPower, -Math.toRadians(current_pos[2]));

        //moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, -Math.toRadians(current_pos[2])); //freight frenzy driver code
        odo.update();//call the odometry to update the current position
        telemetry.update();//update the telemetry to display the most recent values
        //runtime.reset();//reset the runtime for loop timing
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        for (LynxModule hub : allHubs) { //clear out the cache from the hubs at the end of the program to stop values from carrying over through programs.
                                         //do not know if this works perfectly may have to restart the robot to clear values
            hub.clearBulkCache();
        }
        telemetry.update();
    }

    //drive code from freight frenzy
    public void moveRobot(double x, double y, double yaw, double angle) {

        /**gets squared values from the driver's stick input**/
        double r = Math.hypot(-y, -x);//get the hypotenuse of the x and y inputs (r = sqrt(y^2 + x^2) or C^2 = A^2 + B^2)
        /**finds the desired angle that the driver wants to move the robot**/
        double robotAngle = Math.atan2(-x, y) - Math.PI / 4;// use the arc-tangent function to determine the angle, in radians, that the robot desires to move
        /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
         * the offset value is set by the the driver if the imu does not reset after auto*/
        robotAngle = robotAngle + angle;// add the current robot angle and the desired movement angle to find the final movement angle

        double rightX = yaw;// turn power

        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;

        // Output the safe vales to the motor drives.++
        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftBackDrive.setPower(v3);
        rightBackDrive.setPower(v4);
    }

    public void wait_sec (int seconds) {
        runtime.reset();
        while(runtime.seconds() < seconds) {}
    }
}
