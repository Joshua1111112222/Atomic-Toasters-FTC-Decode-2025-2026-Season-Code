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
import com.qualcomm.robotcore.hardware.Servo;
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
public class TestCodeV0_67 extends LinearOpMode {

    // --- Intake (single motor) ---
    DcMotor intakeMotor;

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    boolean flywheelOn = false;
    boolean flywheelReady = false;
    boolean lastBumper = false;

    // Flywheel PIDF
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
    boolean autoAim = false;
    boolean lastA = false;
    private static final double CHASSIS_KP = 0.02; // gain per degree of bearing

    // --- Auto Align ---
    Double targetHeading = null; // stores the heading that should be maintained when locked on

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // Intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheels
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;

        while (opModeIsActive()) {
            // --- Drivetrain control ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.abs(bl)));
            max = Math.max(max, Math.max(Math.abs(fr), Math.abs(br)));

            motorFrontLeft.setPower(fl / max);
            motorBackLeft.setPower(bl / max);
            motorFrontRight.setPower(fr / max);
            motorBackRight.setPower(br / max);

            // --- Intake control ---
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) {
                intakePower = 1.0;
            } else if (gamepad1.left_trigger > 0.2) {
                intakePower = -1.0;
            }
            intakeMotor.setPower(intakePower);

            // --- Flywheel toggle ---
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

            // --- Auto-aim toggle ---
            if (gamepad1.a && !lastA) {
                autoAim = !autoAim;
                if (!autoAim) targetHeading = null; // clear lock when auto aim off
            }
            lastA = gamepad1.a;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();

            if (autoAim) {
                if (tagDetected) {
                    AprilTagDetection tag = detections.get(0);
                    AprilTagPoseFtc pose = tag.ftcPose;

                    // Convert AprilTag bearing (degrees) to radians and add to current heading
                    double desiredHeading = botHeading + Math.toRadians(pose.bearing);
                    targetHeading = desiredHeading; // lock onto tag heading

                    telemetry.addData("AutoAim", "Locking on to Tag ID: %d | Bearing: %.2f°", tag.id, pose.bearing);
                }

                // If we already locked onto a tag, maintain facing that direction
                if (targetHeading != null) {
                    double headingError = targetHeading - botHeading;
                    // Normalize error between -π and π
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    double correction = CHASSIS_KP * headingError;

                    // Apply correction to maintain facing the locked heading
                    motorFrontLeft.setPower(motorFrontLeft.getPower() - correction);
                    motorBackLeft.setPower(motorBackLeft.getPower() - correction);
                    motorFrontRight.setPower(motorFrontRight.getPower() + correction);
                    motorBackRight.setPower(motorBackRight.getPower() + correction);

                    telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
                    telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
                }
            } else {
                telemetry.addData("AutoAim", "Inactive");
            }

            // --- Telemetry ---
            telemetry.addData("AprilTag Detected", tagDetected);
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Flywheel Vel L", flyLeft.getVelocity());
            telemetry.addData("Flywheel Vel R", flyRight.getVelocity());
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
        }
    }
}
//github things