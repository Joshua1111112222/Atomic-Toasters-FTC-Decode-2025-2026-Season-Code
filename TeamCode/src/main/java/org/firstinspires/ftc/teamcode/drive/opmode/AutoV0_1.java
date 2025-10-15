package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name = "AutoV0_1", group = "drive")
public class AutoV0_1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;

        // Explicitly slow constraints (not just half of max, but small numbers)
        double slowVel = 15;   // 15 in/s
        double slowAccel = 10; // 10 in/s^2

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .forward(24,
                                SampleMecanumDrive.getVelocityConstraint(
                                        slowVel,
                                        Math.toRadians(180),
                                        DriveConstants.TRACK_WIDTH
                                ),
                                SampleMecanumDrive.getAccelerationConstraint(slowAccel)
                        )
                        .back(24,
                                SampleMecanumDrive.getVelocityConstraint(
                                        slowVel,
                                        Math.toRadians(180),
                                        DriveConstants.TRACK_WIDTH
                                ),
                                SampleMecanumDrive.getAccelerationConstraint(slowAccel)
                        )
                        .build()
        );

        telemetry.addLine("Finished Forward + Backward @ slowed speed");
        telemetry.update();
        sleep(2000);
    }
}
