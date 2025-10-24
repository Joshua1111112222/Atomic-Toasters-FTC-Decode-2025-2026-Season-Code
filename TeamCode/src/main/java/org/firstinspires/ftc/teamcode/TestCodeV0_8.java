/*
 * Copyright (c) 2025 Team 21518 - The Atomic Toasters
 * Licensed under the No Copy, No Modify License (see LICENSE-TeamAtomicToasters)
 * You MAY NOT copy, modify, or redistribute this file without permission.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "TestCodeV0_8", group = "Linear OpMode")
public class TestCodeV0_8 extends LinearOpMode {

    // --- Intake ---
    private DcMotor intakeMotor;

    // --- Flywheels ---
    private DcMotorEx flyLeft, flyRight;
    private boolean flywheelOn = false;
    private boolean flywheelReady = false;
    private boolean lastBumper = false;

    // PIDF constants
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;
    private static final double targetRPM = 5000;
    private static final double tolerance = 100;
    private double targetVelocity;

    // --- Vision ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean autoAim = false;
    private boolean lastA = false;
    private static final double CHASSIS_KP = 0.03; // higher gain for rotation correction

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drivetrain ---
        DcMotor frontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("motorBackRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // --- Intake ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Flywheels ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Vision ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;

        while (opModeIsActive()) {

            // --- Drive Control ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Normal driving (field-oriented off)
            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.abs(bl)));
            max = Math.max(max, Math.max(Math.abs(fr), Math.abs(br)));

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // --- Intake Control ---
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) intakePower = 1.0;
            else if (gamepad1.left_trigger > 0.2) intakePower = -1.0;
            intakeMotor.setPower(intakePower);

            // --- Flywheel Toggle ---
            if (gamepad1.right_bumper && !lastBumper) {
                flywheelOn = !flywheelOn;
            }
            lastBumper = gamepad1.right_bumper;

            if (flywheelOn) {
                double velL = flyLeft.getVelocity();
                double velR = flyRight.getVelocity();

                double errorL = targetVelocity - velL;
                double errorR = targetVelocity - velR;

                double powerL = (kP * errorL) + (kF * targetVelocity);
                double powerR = (kP * errorR) + (kF * targetVelocity);

                flyLeft.setPower(powerL);
                flyRight.setPower(powerR);

                flywheelReady = (Math.abs(errorL) < tolerance && Math.abs(errorR) < tolerance);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                flywheelReady = false;
            }

            // --- Auto Aim Toggle ---
            if (gamepad1.a && !lastA) {
                autoAim = !autoAim;
            }
            lastA = gamepad1.a;

            // --- Auto Aim (Rotate Chassis Only) ---
            if (autoAim) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                if (!detections.isEmpty()) {
                    AprilTagDetection tag = detections.get(0);
                    AprilTagPoseFtc pose = tag.ftcPose;

                    // Turn in place toward tag
                    double bearingError = pose.bearing;
                    double turnPower = CHASSIS_KP * bearingError;

                    frontLeft.setPower(-turnPower);
                    backLeft.setPower(-turnPower);
                    frontRight.setPower(turnPower);
                    backRight.setPower(turnPower);

                    telemetry.addData("AutoAim", "Rotating | Tag ID: %d | Bearing: %.1f°",
                            tag.id, pose.bearing);
                } else {
                    telemetry.addData("AutoAim", "Active | No tag found");
                }
            } else {
                telemetry.addData("AutoAim", "Inactive");
            }

            // --- Telemetry ---
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading (deg)", botHeading);
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Flywheel Vel L", flyLeft.getVelocity());
            telemetry.addData("Flywheel Vel R", flyRight.getVelocity());
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }
}
//github things