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
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp
public class TestCodeV0_99 extends LinearOpMode {

    // --- Intake ---
    DcMotor intakeMotor;

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    boolean flywheelOn = false;
    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    // PIDF constants
    private static final double kP = 0.002;
    private static final double kI = 0.0000009;   // small integral term
    private static final double kD = 0.0001;      // derivative term for damping
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;
    private static final double closeRPM = 2300;
    private static final double farRPM = 2500;
    private static final double tolerance = 250; // ±250 RPM for "ready"

    private double targetRPM = 0;
    private double targetVelocity = 0;

    // PID state
    private double errorSumL = 0;
    private double errorSumR = 0;
    private double lastErrorL = 0;
    private double lastErrorR = 0;

    // --- Vision ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean autoAim = false;
    boolean lastA = false;

    // Camera toggle
    private boolean cameraEnabled = true;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    // Auto-align tuning (TURN ONLY)
    private static final double ROT_KP = 0.02;
    private static final double ROT_MAX = 0.4;
    private static final double ROT_DEADZONE = 2.0; // widened for stability
    private static final double MIN_TURN_POWER = 0.15;

    // --- Conveyor ---
    CRServo conveyorLeft, conveyorRight, conveyorLeft2;
    boolean conveyorOn = false;
    private static final double CONVEYOR_POWER = 1.0;

    // IMU reset offset
    double imuOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Drivetrain ---
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        // --- Intake ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Flywheels ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Conveyor ---
        conveyorLeft = hardwareMap.get(CRServo.class, "conveyorLeft");
        conveyorRight = hardwareMap.get(CRServo.class, "conveyorRight");
        conveyorLeft2 = hardwareMap.get(CRServo.class, "conveyorLeft2");
        conveyorLeft.setDirection(CRServo.Direction.REVERSE);
        conveyorLeft2.setDirection(CRServo.Direction.REVERSE);
        conveyorRight.setDirection(CRServo.Direction.REVERSE);

        // --- Vision / AprilTag ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        double lastTime = getRuntime();

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // === CAMERA TOGGLE (DPAD UP) ===
            boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            if (dpadUp && !lastDpadUp) {
                cameraEnabled = !cameraEnabled;
                if (cameraEnabled) visionPortal.resumeStreaming();
                else visionPortal.stopStreaming();
            }
            lastDpadUp = dpadUp;

            // === IMU RESET (DPAD DOWN) ===
            boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDown && !lastDpadDown) {
                imuOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            lastDpadDown = dpadDown;

            // === DRIVE ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - imuOffset;
            double botHeading = Math.toRadians(botHeadingDeg);

            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // === APRILTAG AUTO ALIGN (UPDATED) ===
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();

            boolean aPressed = gamepad1.a || gamepad2.a;
            if (aPressed && !lastA) autoAim = !autoAim;
            lastA = aPressed;

            if (autoAim && cameraEnabled && tagDetected) {
                AprilTagDetection tag = detections.get(0);
                if (tag.ftcPose != null) {
                    double yawError = tag.ftcPose.bearing; // positive = tag is right of center

                    // if tag is to the right, turn right (negative rx = clockwise)
                    double rotPower = -yawError * ROT_KP;

                    // clamp
                    rotPower = Math.max(-ROT_MAX, Math.min(ROT_MAX, rotPower));

                    // deadzone
                    if (Math.abs(yawError) < ROT_DEADZONE) rotPower = 0;

                    // min power if not zero
                    if (rotPower != 0) {
                        rotPower = Math.copySign(Math.max(Math.abs(rotPower), MIN_TURN_POWER), rotPower);
                    }

                    // stop translation while aligning
                    rotatedX = 0;
                    rotatedY = 0;

                    rx = rotPower;

                    telemetry.addData("AutoAlign Yaw Error", yawError);
                    telemetry.addData("AutoAlign Turn Power", rotPower);
                }
            }

            // === CONVEYOR CONTROL === (Hold to run)
            boolean bPressed = gamepad1.b || gamepad2.b;
            conveyorOn = bPressed;

            double conveyorPower = 0.0;
            boolean yPressed = gamepad1.y || gamepad2.y;
            if (yPressed) conveyorPower = -CONVEYOR_POWER;
            else if (conveyorOn) conveyorPower = CONVEYOR_POWER;

            conveyorLeft.setPower(conveyorPower);
            conveyorRight.setPower(conveyorPower);
            conveyorLeft2.setPower(conveyorPower);

            // === X BUTTON: INTAKE + CONVEYOR ===
            boolean xPressed = gamepad1.x || gamepad2.x;
            if (xPressed) {
                conveyorLeft2.setPower(1.0);
                conveyorRight.setPower(1.0);
                intakeMotor.setPower(1.0);
            } else {
                if (!conveyorOn && !yPressed) {
                    conveyorLeft2.setPower(0);
                    conveyorRight.setPower(0);
                }
                if (gamepad1.right_trigger < 0.2 && gamepad1.left_trigger < 0.2
                        && gamepad2.right_trigger < 0.2 && gamepad2.left_trigger < 0.2) {
                    intakeMotor.setPower(0);
                }
            }

            // === DRIVETRAIN ===
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

            // === INTAKE CONTROL ===
            double intakePower = 0.0;
            double rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            double leftTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            if (rightTrigger > 0.2) intakePower = 1.0;
            else if (leftTrigger > 0.2) intakePower = -1.0;
            if (!xPressed) intakeMotor.setPower(intakePower);

            // === FLYWHEEL CONTROL ===
            boolean leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

            if (leftBumper && !lastLeftBumper) {
                targetRPM = (flywheelOn && targetRPM == closeRPM) ? 0 : closeRPM;
                flywheelOn = targetRPM > 0;
            }
            if (rightBumper && !lastRightBumper) {
                targetRPM = (flywheelOn && targetRPM == farRPM) ? 0 : farRPM;
                flywheelOn = targetRPM > 0;
            }

            lastLeftBumper = leftBumper;
            lastRightBumper = rightBumper;

            if (flywheelOn) {
                targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;
                double velL = flyLeft.getVelocity();
                double velR = flyRight.getVelocity();
                double rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
                double rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);

                double errorL = targetVelocity - velL;
                double errorR = targetVelocity - velR;

                errorSumL += errorL * dt;
                errorSumR += errorR * dt;
                double derivativeL = (errorL - lastErrorL) / dt;
                double derivativeR = (errorR - lastErrorR) / dt;
                lastErrorL = errorL;
                lastErrorR = errorR;

                double powerL = (kP * errorL) + (kI * errorSumL) + (kD * derivativeL) + (kF * targetVelocity);
                double powerR = (kP * errorR) + (kI * errorSumR) + (kD * derivativeR) + (kF * targetVelocity);

                flyLeft.setPower(powerL);
                flyRight.setPower(powerR);

                boolean flywheelReady = Math.abs(rpmL - targetRPM) <= tolerance &&
                        Math.abs(rpmR - targetRPM) <= tolerance;

                // Telemetry
                String statusText;
                if (!flywheelOn) statusText = "🔴 OFF";
                else if (flywheelReady) statusText = "🟢 READY";
                else statusText = "🟡 SPINNING UP";

                String rpmLText = String.format("%s %.1f", flywheelReady ? "🟢" : "🟡", rpmL);
                String rpmRText = String.format("%s %.1f", flywheelReady ? "🟢" : "🟡", rpmR);
                String powerLText = String.format("%.3f", powerL);
                String powerRText = String.format("%.3f", powerR);

                telemetry.addLine("=== FLYWHEEL STATUS ===");
                telemetry.addData("Mode", targetRPM == closeRPM ? "CLOSE (2500)" : "FAR (2700)");
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("L RPM", rpmLText);
                telemetry.addData("R RPM", rpmRText);
                telemetry.addData("Power L", powerLText);
                telemetry.addData("Power R", powerRText);
                telemetry.addData("Status", statusText);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                errorSumL = 0;
                errorSumR = 0;
                lastErrorL = 0;
                lastErrorR = 0;

                telemetry.addLine("=== FLYWHEEL STATUS ===");
                telemetry.addData("Mode", "OFF");
                telemetry.addData("Target RPM", 0);
                telemetry.addData("L RPM", "🔴 0");
                telemetry.addData("R RPM", "🔴 0");
                telemetry.addData("Power L", "0.000");
                telemetry.addData("Power R", "0.000");
                telemetry.addData("Status", "🔴 OFF");
            }

            // === TELEMETRY: ROBOT HEADING, AUTOALIGN, CAMERA, APRILTAG, CONVEYOR ===
            telemetry.addData("Heading (deg)", botHeadingDeg);
            telemetry.addData("AutoAlign Active", autoAim);
            telemetry.addData("Camera Enabled", cameraEnabled);
            telemetry.addData("AprilTag Detected", tagDetected);
            telemetry.addData("Conveyor Active (B)", conveyorOn);

            telemetry.update();
        }
    }
}