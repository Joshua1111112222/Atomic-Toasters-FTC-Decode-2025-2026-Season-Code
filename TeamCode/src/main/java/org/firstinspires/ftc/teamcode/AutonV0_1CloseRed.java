/*
 * Copyright (c) 2025 Team 21518 - The Atomic Toasters
 * Licensed under the No Copy, No Modify License (see LICENSE-TeamAtomicToasters)
 * You MAY NOT copy, modify, or redistribute this file without permission.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonV0_1CloseRed", group = "Main")
public class AutonV0_1CloseRed extends LinearOpMode {

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;

    // --- Conveyor + Intake ---
    CRServo conveyorLeft, conveyorRight, conveyorLeft2;
    DcMotor intakeMotor;

    // --- Drive Motors ---
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;

    // --- Constants ---
    private static final double TARGET_RPM = 2300;
    private static final double RPM_TOLERANCE = 200;    // ±200 RPM window
    private static final double MIN_RPM_DROP = 100;     // detect ball shot
    private static final double CONVEYOR_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;

    // --- Shot control ---
    private static final int MAX_SHOTS = 2;
    private int shotsFired = 0;
    private boolean feeding = false;
    private double lastAvgRPM = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware map ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorEx.Direction.REVERSE);

        conveyorLeft = hardwareMap.get(CRServo.class, "conveyorLeft");
        conveyorRight = hardwareMap.get(CRServo.class, "conveyorRight");
        conveyorLeft2 = hardwareMap.get(CRServo.class, "conveyorLeft2");
        conveyorLeft.setDirection(CRServo.Direction.REVERSE);
        conveyorLeft2.setDirection(CRServo.Direction.REVERSE);
        conveyorRight.setDirection(CRServo.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized. Ready to run Auto BackUp + Shoot + Strafe Right.");
        telemetry.update();

        waitForStart();

        double targetVelocity = (TARGET_RPM / 60.0) * ticksPerRev * gearRatio;

        // --- Backup using straight motor power ---
        telemetry.addLine("Backing up for 1.5 seconds...");
        telemetry.update();

        double basePower = -0.5;
        motorFrontLeft.setPower(basePower);
        motorFrontRight.setPower(basePower);
        motorBackLeft.setPower(basePower);
        motorBackRight.setPower(basePower);
        sleep(1500); // backup duration (milliseconds)

        // Stop drive motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        telemetry.addLine("✅ Backup complete. Starting shooting sequence...");
        telemetry.update();

        // --- Shooting phase with timeout ---
        shotsFired = 0;
        feeding = false;
        lastAvgRPM = 0;

        long lastShotTime = System.currentTimeMillis();
        long TIMEOUT_MS = 10000; // 10 seconds timeout

        while (opModeIsActive() && shotsFired < MAX_SHOTS) {

            // --- Flywheel control ---
            double velL = flyLeft.getVelocity();
            double velR = flyRight.getVelocity();
            double rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
            double rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);
            double avgRPM = (rpmL + rpmR) / 2.0;

            double errorL = targetVelocity - velL;
            double errorR = targetVelocity - velR;

            double powerL = (kP * errorL) + (kF * targetVelocity);
            double powerR = (kP * errorR) + (kF * targetVelocity);

            flyLeft.setPower(powerL);
            flyRight.setPower(powerR);

            // --- Start feeding once flywheel stable ---
            if (!feeding && Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                feeding = true;
                conveyorLeft.setPower(CONVEYOR_POWER);
                conveyorRight.setPower(CONVEYOR_POWER);
                conveyorLeft2.setPower(CONVEYOR_POWER);
                intakeMotor.setPower(INTAKE_POWER);
            }

            // --- Detect shot using delta RPM ---
            if (feeding && lastAvgRPM - avgRPM > MIN_RPM_DROP) {
                shotsFired++;
                feeding = false;
                lastShotTime = System.currentTimeMillis();

                // Stop conveyors and intake
                conveyorLeft.setPower(0);
                conveyorRight.setPower(0);
                conveyorLeft2.setPower(0);
                intakeMotor.setPower(0);
            }

            // --- Timeout: if no shot in last 10 sec, break ---
            if (System.currentTimeMillis() - lastShotTime > TIMEOUT_MS) {
                telemetry.addLine("⚠️ Timeout reached. Moving to strafe.");
                telemetry.update();
                break;
            }

            lastAvgRPM = avgRPM;

            telemetry.addData("Flywheel Avg RPM", avgRPM);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Feeding", feeding);
            telemetry.update();
        }

        // Stop flywheels and intake after shooting or timeout
        flyLeft.setPower(0);
        flyRight.setPower(0);
        conveyorLeft.setPower(0);
        conveyorRight.setPower(0);
        conveyorLeft2.setPower(0);
        intakeMotor.setPower(0);

        // --- Strafe right (mecanum) ---
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(-0.5);
        motorBackLeft.setPower(-0.5);
        motorBackRight.setPower(0.5);
        sleep(1000);

        // Stop after strafing
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        telemetry.addLine("✅ Finished Strafe Right!");
        telemetry.update();
        sleep(500);
    }
}
