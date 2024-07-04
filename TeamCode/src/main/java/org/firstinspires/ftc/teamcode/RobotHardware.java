package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
@Config
public class RobotHardware {

    /* Declare OpMode members. */
    private OpMode myOpMode;   // gain access to methods in the calling OpMode.
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    public odometry odo = null;
    public motorOdometry motor_odo = null;
    public Drivetrain drivetrain = null;
    List<LynxModule> allHubs;
    BHI260IMU imu;
    IMU.Parameters myIMUparameters;
    Telemetry telemetry;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetry for ftc dashboard and driver station

        imu = myOpMode.hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
                myIMUparameters = new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        allHubs = myOpMode.hardwareMap.getAll(LynxModule.class); // bulk reading hubs for faster loop speeds

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //hardware declarations
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "motor3");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "motor2");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "motor1");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "motor");

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
        motor_odo = new motorOdometry(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, 0, 0, 90, telemetry, imu);
        drivetrain = new Drivetrain(rightBackDrive, leftBackDrive, rightFrontDrive, leftFrontDrive);
    }

    public void clearCache () {
        for (LynxModule hub : allHubs) { //clear out the cache from the hubs at the end of the program to stop values from carrying over through programs.
            //do not know if this works perfectly may have to restart the robot to clear values
            hub.clearBulkCache();
        }
    }
}

