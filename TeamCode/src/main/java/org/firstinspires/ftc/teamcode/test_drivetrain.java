package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="test_drivetrain")
@Config
//@Disabled
public class test_drivetrain extends OpMode {
    // Declare OpMode members.
    private RobotHardware robot = new RobotHardware(this);
    double[] current_pos;

    @Override
    public void init() {
        robot.init();
        robot.clearCache();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetry for ftc dashboard and driver station
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
    }

    public void updateAll() {
        robot.motor_odo.update();//call the odometry to update the current position
        current_pos = robot.motor_odo.getPose();
        telemetry.addData("FL", robot.drivetrain.leftFrontDrive.getCurrentPosition());
        telemetry.addData("FR", robot.drivetrain.rightFrontDrive.getCurrentPosition());
        telemetry.addData("BL", robot.drivetrain.leftBackDrive.getCurrentPosition());
        telemetry.addData("BR", robot.drivetrain.rightBackDrive.getCurrentPosition());
        telemetry.addData("x", "%.2f", current_pos[0]);//output of odometry for x
        telemetry.addData("y", "%.2f", current_pos[1]);//output of odometry for y
        telemetry.addData("heading", "%.2f", current_pos[2]);//output of odometry for heading
        telemetry.update();//update the telemetry to display the most recent values
    }
}
