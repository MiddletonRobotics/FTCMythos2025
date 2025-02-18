package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDrive extends RobotDrive {
    DcMotorEx frontLeft, frontRight,backLeft, backRight;
    DcMotorEx[] motors;

    /**
     * The constructor for the mecanum drive.
     *
     * @param aHardwareMap Passing through HardwareMap from OpMode
     */

    public MecanumDrive(HardwareMap aHardwareMap) {
        frontLeft = aHardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = aHardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = aHardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = aHardwareMap.get(DcMotorEx.class, "rightBack");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

    }

    public void setRange(double min, double max) {
        super.setRange(min, max);
    }

    public void setMaxSpeed(double value) {
        super.setMaxSpeed(value);
    }

    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param frontLeftSpeed    the speed of the front left motor
     * @param frontRightSpeed   the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed    the speed of the back right motor
     *
     */

    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        motors[MotorType.kFrontLeft.value].setPower(frontLeftSpeed * maxOutput);
        motors[MotorType.kFrontRight.value].setPower(frontRightSpeed * maxOutput);
        motors[MotorType.kBackLeft.value].setPower(backLeftSpeed * maxOutput);
        motors[MotorType.kBackRight.value].setPower(backRightSpeed * maxOutput);
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param rotationalSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     */

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed, double gyroAngle) {
        forwardSpeed = clipRange(forwardSpeed);
        strafeSpeed = clipRange(strafeSpeed);
        rotationalSpeed = clipRange(rotationalSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);
        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[MotorType.kFrontLeft.value] += rotationalSpeed;
        wheelSpeeds[MotorType.kFrontRight.value] -= rotationalSpeed;
        wheelSpeeds[MotorType.kBackLeft.value] += rotationalSpeed;
        wheelSpeeds[MotorType.kBackRight.value] -= rotationalSpeed;

        normalize(wheelSpeeds);

        driveWithMotorPowers(
                wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value],
                wheelSpeeds[MotorType.kBackLeft.value],
                wheelSpeeds[MotorType.kBackRight.value]
        );
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param xSpeed       the horizontal speed of the robot, derived from input
     * @param ySpeed       the vertical speed of the robot, derived from input
     * @param rotationalSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     * @param squareInputs Square the value of the input to allow for finer control
     */

    public void driveFieldCentric(double xSpeed, double ySpeed, double rotationalSpeed, double gyroAngle, boolean squareInputs) {
        xSpeed = squareInputs ? clipRange(squareInput(xSpeed)) : clipRange(xSpeed);
        ySpeed = squareInputs ? clipRange(squareInput(ySpeed)) : clipRange(ySpeed);
        rotationalSpeed = squareInputs ? clipRange(squareInput(rotationalSpeed)) : clipRange(rotationalSpeed);

        driveFieldCentric(xSpeed, ySpeed, rotationalSpeed, gyroAngle);
    }

    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param rotationalSpeed    the turn speed of the robot, derived from input
     */

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed) {
        driveFieldCentric(strafeSpeed, forwardSpeed, rotationalSpeed, 0.0);
    }

    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param rotationalSpeed    the turn speed of the robot, derived from input
     * @param squareInputs Square joystick inputs for finer control
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed, boolean squareInputs) {
        forwardSpeed = squareInputs ? clipRange(squareInput(forwardSpeed)) : clipRange(forwardSpeed);
        strafeSpeed = squareInputs ? clipRange(squareInput(strafeSpeed)) : clipRange(strafeSpeed);
        rotationalSpeed = squareInputs ? clipRange(squareInput(rotationalSpeed)) : clipRange(rotationalSpeed);

        driveRobotCentric(strafeSpeed, forwardSpeed, rotationalSpeed);
    }

    @Override
    public void stop() {
        for(DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
}
