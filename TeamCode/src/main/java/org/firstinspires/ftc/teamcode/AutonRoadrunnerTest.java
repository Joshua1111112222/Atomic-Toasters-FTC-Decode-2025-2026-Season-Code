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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonRoadrunnerTest", group = "Test")
public class AutonRoadrunnerTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Road Runner drive ---
        drive = new SampleMecanumDrive(hardwareMap);

        // --- Dashboard setup ---
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // --- Starting pose ---
        Pose2d startPose = new Pose2d(60, 52.5, Math.toRadians(48));
        drive.setPoseEstimate(startPose);

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- Trajectory 1: Move to (0,0,48) ---
        Pose2d targetPose1 = new Pose2d(0, 0, Math.toRadians(48));
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(targetPose1)
                .build();

        // Follow Traj 1
        drive.followTrajectory(traj1);
        updateDashboardPose();

        // --- Rotate to -90 degrees ABSOLUTELY ---
        double currentHeading = Math.toDegrees(drive.getPoseEstimate().getHeading());
        double turnAmount = -90 - currentHeading;  // absolute target - current heading
        drive.turn(Math.toRadians(turnAmount));

        // Set new pose after turn
        Pose2d rotatedPose = new Pose2d(
                drive.getPoseEstimate().getX(),
                drive.getPoseEstimate().getY(),
                Math.toRadians(-90)
        );
        drive.setPoseEstimate(rotatedPose);

        updateDashboardPose();

        // --- Trajectory 3: Move to (54, 11, -90) ---
        Pose2d targetPose2 = new Pose2d(54, 11, Math.toRadians(-90));
        Trajectory traj3 = drive.trajectoryBuilder(rotatedPose)
                .lineToLinearHeading(targetPose2)
                .build();

        drive.followTrajectory(traj3);
        updateDashboardPose();

        telemetry.addLine("✅ Finished all movements!");
        telemetry.update();
        sleep(5000);
    }

    private void updateDashboardPose() {
        drive.update();
        Pose2d pose = drive.getPoseEstimate();

        // Draw on dashboard
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        field.setStroke("#FF0000"); // red robot
        field.strokeCircle(pose.getX(), pose.getY(), 9);
        field.strokeLine(
                pose.getX(),
                pose.getY(),
                pose.getX() + 9 * Math.cos(pose.getHeading()),
                pose.getY() + 9 * Math.sin(pose.getHeading())
        );

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
