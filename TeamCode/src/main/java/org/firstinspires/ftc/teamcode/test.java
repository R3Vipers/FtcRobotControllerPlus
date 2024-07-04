package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="test")
@Config
//@Disabled
public class test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime totalRuntime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware(this);
    private PathFollower follower = null;
    private PathFollower follower2 = null;
    private Path path = null;
    private Path path2 = null;
    double [] current_pos;
    boolean wait = false;

    @Override
    public void init() {
        robot.init();
        robot.clearCache();
        //creating new path
        path = new Path(30, 0, 0); // number of points, starting x, starting y;
        path.addControlPoint(0, 24); // new point x, y
        path.addControlPoint(24, 24); // new point x, y
        path.addControlPoint(24, 48); // new point x, y
        path2 = new Path(30, 24, 48);
        path2.addControlPoint(0, 0); // new point x, y
        follower = new PathFollower(robot.drivetrain, path);
        follower2 = new PathFollower(robot.drivetrain, path2);
        // Tell the driver that initialization is complete.
        robot.telemetry.addData("Status", "Initialized");
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
        totalRuntime.reset();
        while(totalRuntime.seconds() < 30) {
            //clearing the cache form all hubs
            robot.clearCache();
            updateAll();
            if(follower.followForward(current_pos, 0, totalRuntime)) {
                robot.drivetrain.stop();
                break;
            }
        }
        while(totalRuntime.seconds() < 30) {
            //clearing the cache form all hubs
            robot.clearCache();
            updateAll();
            if(waitSeconds(5))
                break;
        }
        while(totalRuntime.seconds() < 30) {
            //clearing the cache form all hubs
            robot.clearCache();
            updateAll();
            if(follower2.followHoldHeading(current_pos, 90, 90, totalRuntime)) {
                robot.drivetrain.stop();
                break;
            }
        }
        telemetry.update();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //clearing the cache form all huds
        robot.clearCache();
        robot.telemetry.addData("status", "TeleOp Period");
        robot.drivetrain.moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, -Math.toRadians(current_pos[2])); //freight frenzy driver code
        updateAll();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void updateAll (){
        robot.odo.update();//call the odometry to update the current position
        current_pos = robot.odo.getPose();
        robot.telemetry.addData("x", "%.2f",current_pos[0]);//output of odometry for x
        robot.telemetry.addData("y","%.2f", current_pos[1]);//output of odometry for y
        robot.telemetry.addData("heading","%.2f", current_pos[2]);//output of odometry for heading
        robot.telemetry.update();//update the telemetry to display the most recent values
    }

    public boolean waitSeconds(int seconds) {
        if(!wait) {
            wait = true;
            runtime.reset();
        } else {
            if(runtime.seconds() > seconds) {
                wait = false;
                return true;
            }
        }
        return false;
    }
}
