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
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestCodeV0_6 extends LinearOpMode {

    // --- Intake (single motor) ---
    DcMotor intakeMotor;

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    boolean flywheelOn = false;
    boolean flywheelReady = false;
    boolean lastBumper = false;

    // --- Blinkin LED driver ---
    RevBlinkinLedDriver blinkin;

    // Flywheel PIDF constants
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0; // Typical goBILDA 5202/5203 motor encoder
    private static final double gearRatio = 1.0;    // 1:1 ratio
    private static final double targetRPM = 3000;   // Flywheel target speed
    private static final double tolerance = 100;    // RPM tolerance
    private static final double ledTolerance = 200; // LED indication tolerance
    private double targetVelocity; // in ticks/sec

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

        // --- Blinkin LED Driver ---
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        waitForStart();

        // Convert target RPM → ticks/sec
        targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;

        while (opModeIsActive()) {

            // === Field-centric drive ===
            double y = -gamepad1.left_stick_y;  // Forward
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Get current robot heading (radians)
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate joystick input for field-centric control
            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate motor powers
            double fl = rotatedY + rotatedX + rx;
            double bl = rotatedY - rotatedX + rx;
            double fr = rotatedY - rotatedX - rx;
            double br = rotatedY + rotatedX - rx;

            // Normalize powers
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
            motorFrontLeft.setPower(fl / max);
            motorBackLeft.setPower(bl / max);
            motorFrontRight.setPower(fr / max);
            motorBackRight.setPower(br / max);

            // === Intake Control (Triggers) ===
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) {
                intakePower = 1.0;   // Intake in
            } else if (gamepad1.left_trigger > 0.2) {
                intakePower = -1.0;  // Intake out
            }
            intakeMotor.setPower(intakePower);

            // === Flywheel Toggle (Right Bumper) ===
            if (gamepad1.right_bumper && !lastBumper) {
                flywheelOn = !flywheelOn;
            }
            lastBumper = gamepad1.right_bumper;

            if (flywheelOn) {
                double velL = flyLeft.getVelocity();  // ticks/sec
                double velR = flyRight.getVelocity(); // ticks/sec

                // Convert encoder velocity → RPM
                double rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
                double rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);

                // PIDF control
                double errorL = targetVelocity - velL;
                double errorR = targetVelocity - velR;

                double powerL = (kP * errorL) + (kF * targetVelocity);
                double powerR = (kP * errorR) + (kF * targetVelocity);

                flyLeft.setPower(powerL);
                flyRight.setPower(powerR);

                flywheelReady = (Math.abs(rpmL - targetRPM) < tolerance &&
                        Math.abs(rpmR - targetRPM) < tolerance);

                // --- LED Control ---
                if (Math.abs(rpmL - targetRPM) < ledTolerance &&
                        Math.abs(rpmR - targetRPM) < ledTolerance) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                } else {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

                // --- Flywheel Telemetry ---
                telemetry.addData("Flywheel L RPM", rpmL);
                telemetry.addData("Flywheel R RPM", rpmR);
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Flywheel Power L", powerL);
                telemetry.addData("Flywheel Power R", powerR);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                flywheelReady = false;
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            // === Telemetry (IMU + Intake) ===
            telemetry.addData("Heading (deg)", Math.toDegrees(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }
}
//github things