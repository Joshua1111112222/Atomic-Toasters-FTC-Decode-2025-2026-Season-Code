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

@TeleOp
public class TestCodeV0_5 extends LinearOpMode {

    // --- Intake (single motor) ---
    DcMotor intakeMotor;

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    boolean flywheelOn = false;
    boolean flywheelReady = false;
    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    // Flywheel PIDF constants
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0; // goBILDA 5202/5203 encoder
    private static final double gearRatio = 1.0;    // 1:1
    private static final double closeRPM = 2500;    // Close shot
    private static final double farRPM = 2700;      // Far shot
    private static final double tolerance = 100;    // RPM tolerance

    private double targetRPM = 0;                   // Current active target
    private double targetVelocity = 0;              // in ticks/sec

    // --- Vision (Webcam + AprilTag) ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean autoAim = false;
    boolean lastA = false;
    private static final double CHASSIS_KP = 0.02; // base turning gain
    private static final double SMOOTH_FACTOR = 0.3; // smoothing for correction
    Double targetHeading = null; // locked heading

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drivetrain motors ---
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- IMU setup ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        // --- Intake motor ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Flywheel motors ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Webcam + AprilTag setup ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        double smoothCorrection = 0; // used for easing turn motion

        while (opModeIsActive()) {

            // === Field-centric drive ===
            double y = -gamepad1.left_stick_y;  // Forward
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // --- AprilTag Detection ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();

            // === Auto Align Toggle ===
            if (gamepad1.a && !lastA) {
                autoAim = !autoAim;
                if (!autoAim) targetHeading = null;
            }
            lastA = gamepad1.a;

            if (autoAim) {
                if (tagDetected) {
                    AprilTagDetection tag = detections.get(0);
                    AprilTagPoseFtc pose = tag.ftcPose;
                    double desiredHeading = botHeading + Math.toRadians(pose.bearing);
                    targetHeading = desiredHeading;
                    telemetry.addData("AutoAlign", "Tracking Tag ID: %d | Bearing: %.2f°", tag.id, pose.bearing);
                }

                if (targetHeading != null) {
                    double headingError = targetHeading - botHeading;

                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    double correction = CHASSIS_KP * headingError;

                    // --- Smooth correction (easing filter) ---
                    smoothCorrection = smoothCorrection * (1 - SMOOTH_FACTOR) + correction * SMOOTH_FACTOR;

                    // Disable driver rotation stick during auto-aim
                    rx = smoothCorrection;
                }
            } else {
                smoothCorrection = 0;
            }

            // Apply movement with possible auto-align rotation correction
            double fl = rotatedY + rotatedX + rx;
            double bl = rotatedY - rotatedX + rx;
            double fr = rotatedY - rotatedX - rx;
            double br = rotatedY + rotatedX - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
            motorFrontLeft.setPower(fl / max);
            motorBackLeft.setPower(bl / max);
            motorFrontRight.setPower(fr / max);
            motorBackRight.setPower(br / max);

            // === Intake Control ===
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) {
                intakePower = 1.0;
            } else if (gamepad1.left_trigger > 0.2) {
                intakePower = -1.0;
            }
            intakeMotor.setPower(intakePower);

            // === Flywheel Controls ===
            if (gamepad1.left_bumper && !lastLeftBumper) {
                if (flywheelOn && targetRPM == closeRPM) {
                    flywheelOn = false;
                } else {
                    targetRPM = closeRPM;
                    flywheelOn = true;
                }
            }
            if (gamepad1.right_bumper && !lastRightBumper) {
                if (flywheelOn && targetRPM == farRPM) {
                    flywheelOn = false;
                } else {
                    targetRPM = farRPM;
                    flywheelOn = true;
                }
            }

            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            if (flywheelOn) {
                targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;
                double velL = flyLeft.getVelocity();
                double velR = flyRight.getVelocity();
                double rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
                double rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);
                double errorL = targetVelocity - velL;
                double errorR = targetVelocity - velR;
                double powerL = (kP * errorL) + (kF * targetVelocity);
                double powerR = (kP * errorR) + (kF * targetVelocity);
                flyLeft.setPower(powerL);
                flyRight.setPower(powerR);
                flywheelReady = (Math.abs(rpmL - targetRPM) < tolerance &&
                        Math.abs(rpmR - targetRPM) < tolerance);
                telemetry.addData("Flywheel Mode", targetRPM == closeRPM ? "CLOSE (2500)" : "FAR (2700)");
                telemetry.addData("Flywheel L RPM", rpmL);
                telemetry.addData("Flywheel R RPM", rpmR);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                flywheelReady = false;
            }

            // === Telemetry ===
            telemetry.addData("AutoAlign Active", autoAim);
            telemetry.addData("AprilTag Detected", tagDetected);
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
        }
    }
}//github things
