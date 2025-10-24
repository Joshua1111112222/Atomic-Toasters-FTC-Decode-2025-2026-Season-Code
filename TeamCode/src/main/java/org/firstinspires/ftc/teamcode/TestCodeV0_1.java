/*
 * Copyright (c) 2025 Team 21518 - The Atomic Toasters
 * Licensed under the No Copy, No Modify License (see LICENSE-TeamAtomicToasters)
 * You MAY NOT copy, modify, or redistribute this file without permission.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestCodeV0_1 extends LinearOpMode {

    // Intake motor
    DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drivetrain ---
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- IMU for field-centric ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // --- Intake (GoBILDA motor) ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // --- Field-centric drive ---
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotation

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate joystick input by robot heading
            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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

            // --- Intake control (triggers) ---
            double intakePower = 0.0;

            if (gamepad1.right_trigger > 0.2) {
                intakePower = 1.0;   // intake in
            } else if (gamepad1.left_trigger > 0.2) {
                intakePower = -1.0;  // intake out
            }

            intakeMotor.setPower(intakePower);

            // Telemetry
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }
}// github things
