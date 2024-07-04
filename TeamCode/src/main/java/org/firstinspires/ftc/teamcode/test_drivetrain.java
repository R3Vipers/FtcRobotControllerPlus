package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="test_drivetrain")
@Config
//@Disabled
public class test_drivetrain extends OpMode {
    // Declare OpMode members.
    private RobotHardware robot = new RobotHardware(this);
    double[] current_pos;
    Telemetry telemetry;
    motorOdometry odo;
    @Override
    public void init() {
        robot.init();
        robot.clearCache();
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetry for ftc dashboard and driver station
        odo = new motorOdometry(robot.drivetrain.leftFrontDrive, robot.drivetrain.rightFrontDrive, robot.drivetrain.leftBackDrive, robot.drivetrain.rightBackDrive, 0, 0, 90, telemetry, robot.imu);
        robot.drivetrain.reset();
        robot.clearCache();
        updateAll();
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
        robot.clearCache();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //clearing the cache form all huds
        robot.clearCache();
        telemetry.addData("status", "TeleOp Period");
        robot.drivetrain.moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, -Math.toRadians(current_pos[2])); //freight frenzy driver code
        updateAll();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.clearCache();
    }

    public void updateAll() {
        odo.update();//call the odometry to update the current position
        current_pos = odo.getPose();
        //telemetry.addData("change in heading", "%f", robot.motor_odo.deltaHeading);
        Orientation angles = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        telemetry.addData("angle", "%f", angle);
        telemetry.addData("track width", "%f", (robot.motor_odo.current_left_front_pos+robot.motor_odo.current_left_back_pos-robot.motor_odo.current_right_front_pos-robot.motor_odo.current_right_back_pos)/(2.0*angle*robot.motor_odo.ENCODER_COUNTS_PER_INCH));
        telemetry.addData("FL", -robot.drivetrain.leftFrontDrive.getCurrentPosition());
        telemetry.addData("FR", robot.drivetrain.rightFrontDrive.getCurrentPosition());
        telemetry.addData("BL", robot.drivetrain.leftBackDrive.getCurrentPosition());
        telemetry.addData("BR", -robot.drivetrain.rightBackDrive.getCurrentPosition());
        telemetry.addData("x", "%.2f", current_pos[0]);//output of odometry for x
        telemetry.addData("y", "%.2f", current_pos[1]);//output of odometry for y
        telemetry.addData("heading", "%.2f", current_pos[2]);//output of odometry for heading
        telemetry.update();//update the telemetry to display the most recent values
    }
}
