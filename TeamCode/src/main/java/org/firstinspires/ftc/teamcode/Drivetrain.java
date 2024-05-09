package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    public DcMotor rightBackDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftFrontDrive;

    public Drivetrain (DcMotor rightBackDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive, DcMotor leftFrontDrive) {
        this.rightBackDrive = rightBackDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.leftFrontDrive = leftFrontDrive;
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
}
