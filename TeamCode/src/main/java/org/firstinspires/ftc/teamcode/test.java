package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="test")
@Config
@Disabled
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

    // The IMU sensor object
    BHI260IMU imu;
    @Override
    public void init() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BHI260IMU.class, "imu");
//        imu.initialize();

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
        odo = new odometry(leftBackDrive, rightBackDrive, rightFrontDrive, 0, 0, 0, telemetry, imu);

        path = new Path(5, 0, 0); // number of points, starting x, starting y;
        path.addControlPoint(0, 10); // new point x, y
        path.addControlPoint(10,10);
        path.addControlPoint(0,0);
        path_to_follow = path.get_Path();

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
        telemetry.addData("y", "%.2f",current_pos[0]);
        telemetry.addData("x","%.2f", current_pos[1]);
        telemetry.addData("heading","%.2f", current_pos[2]);
        telemetry.addData("loop speed", "%f", (1/runtime.seconds()));
        // pseudo code for possible path following

//        for (int i = 0; i< path_to_follow.size()-1; i++) {
//            Point temp = path_to_follow.get(i);
//            while(temp.dist_to_point(current_pos[1], current_pos[0]) > 0.1) {
//                double yPower = (temp.y - current_pos[0]) * 0.15;
//                double xPower = (temp.x - current_pos[1]) * 0.15;
//                double turnPower = (0 - current_pos[0]) * 0.003;
//
//                //Normalize wheel powers to be less than 1.0
//                double max = Math.max(Math.abs(yPower), Math.abs(xPower));
//                max = Math.max(max, Math.abs(turnPower));
//
//                if (max > 1.0) {
//                    yPower /= max;
//                    xPower /= max;
//                    turnPower /= max;
//                }
//                moveRobot(xPower, yPower, turnPower, -Math.toRadians(current_pos[2]));
//            }
//        }

        moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, -Math.toRadians(current_pos[2]));
        odo.update();
        telemetry.update();
        runtime.reset();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void moveRobot(double x, double y, double yaw, double angle) {

        /**gets squared values from the driver's stick input**/
        double r = Math.hypot(-y, -x);
        /**finds the desired angle that the driver wants to move the robot**/
        double robotAngle = Math.atan2(-x, y) - Math.PI / 4;
        /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
         * the offset value is set by the the driver if the imu does not reset after auto*/
        robotAngle = robotAngle + angle;

        double rightX = yaw;

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
}
