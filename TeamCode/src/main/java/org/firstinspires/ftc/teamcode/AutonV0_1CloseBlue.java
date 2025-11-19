/*
 * Copyright (c) 2025 Team 21518 - The Atomic Toasters
 * Licensed under the No Copy, No Modify License (see LICENSE-TeamAtomicToasters)
 * You MAY NOT copy, modify, or redistribute this file without permission.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonV0_1CloseBlueRR", group = "Main")
public class AutonV0_1CloseBlue extends LinearOpMode {

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;

    // --- Conveyor + Intake ---
    CRServo conveyorLeft, conveyorRight, conveyorLeft2;
    DcMotor intakeMotor;

    // --- Road Runner Drive ---
    private SampleMecanumDrive drive;

    // --- Dashboard ---
    private FtcDashboard dashboard;

    // --- Constants ---
    private static final double TARGET_RPM = 2400;
    private static final double RPM_TOLERANCE = 200;
    private static final double MIN_RPM_DROP = 200;
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

        // --- Road Runner drive ---
        drive = new SampleMecanumDrive(hardwareMap);

        // --- Setup Dashboard ---
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Initialized. Ready to run Auto BackUp + Shoot + Strafe Left.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- Starting Pose ---
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // --- BACKUP trajectory ---
        Trajectory backupTrajectory = drive.trajectoryBuilder(startPose)
                .back(80)
                .build();

        // --- STRAFE trajectory ---
        Trajectory strafeTrajectory = drive.trajectoryBuilder(backupTrajectory.end())
                .strafeLeft(80)
                .build();

        telemetry.addLine("🚀 Following backup trajectory...");
        telemetry.update();
        drive.followTrajectory(backupTrajectory);

        telemetry.addLine("✅ Backup complete. Starting shooting sequence...");
        telemetry.update();

        double targetVelocity = (TARGET_RPM / 60.0) * ticksPerRev * gearRatio;
        shotsFired = 0;
        feeding = false;
        lastAvgRPM = 0;
        long lastShotTime = System.currentTimeMillis();
        long TIMEOUT_MS = 10000;

        // --- Shooting loop ---
        while (opModeIsActive() && shotsFired < MAX_SHOTS) {
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

            // Feed control
            if (!feeding && Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                feeding = true;
                conveyorLeft.setPower(CONVEYOR_POWER);
                conveyorRight.setPower(CONVEYOR_POWER);
                conveyorLeft2.setPower(CONVEYOR_POWER);
                intakeMotor.setPower(INTAKE_POWER);
            }

            if (feeding && lastAvgRPM - avgRPM > MIN_RPM_DROP) {
                shotsFired++;
                feeding = false;
                lastShotTime = System.currentTimeMillis();
                conveyorLeft.setPower(0);
                conveyorRight.setPower(0);
                conveyorLeft2.setPower(0);
                intakeMotor.setPower(0);
            }

            if (System.currentTimeMillis() - lastShotTime > TIMEOUT_MS) {
                telemetry.addLine("⚠️ Timeout reached. Moving to strafe.");
                telemetry.update();
                break;
            }

            lastAvgRPM = avgRPM;

            // --- Update drive for live dashboard pose ---
            drive.update();
            Pose2d pose = drive.getPoseEstimate();

            // Send live robot pose to dashboard field
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // Draw robot
            field.setStroke("#3F51B5"); // Blue color
            field.strokeCircle(pose.getX(), pose.getY(), 9); // robot radius ~9 in
            field.strokeLine(
                    pose.getX(),
                    pose.getY(),
                    pose.getX() + 9 * Math.cos(pose.getHeading()),
                    pose.getY() + 9 * Math.sin(pose.getHeading())
            );
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Flywheel Avg RPM", avgRPM);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Feeding", feeding);
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }

        // Stop everything
        flyLeft.setPower(0);
        flyRight.setPower(0);
        conveyorLeft.setPower(0);
        conveyorRight.setPower(0);
        conveyorLeft2.setPower(0);
        intakeMotor.setPower(0);

        // --- Strafe trajectory follow ---
        telemetry.addLine("🚗 Strafing left...");
        telemetry.update();
        drive.followTrajectory(strafeTrajectory);

        drive.update();
        Pose2d finalPose = drive.getPoseEstimate();

        // Send final pose to dashboard
        TelemetryPacket finalPacket = new TelemetryPacket();
        Canvas finalField = finalPacket.fieldOverlay();
        finalField.setStroke("#00FF00");
        finalField.strokeCircle(finalPose.getX(), finalPose.getY(), 9);
        dashboard.sendTelemetryPacket(finalPacket);

        telemetry.addLine("✅ Finished Strafe Left!");
        telemetry.addData("Final X", finalPose.getX());
        telemetry.addData("Final Y", finalPose.getY());
        telemetry.addData("Final Heading (deg)", Math.toDegrees(finalPose.getHeading()));
        telemetry.update();

        sleep(5000);
    }
}
