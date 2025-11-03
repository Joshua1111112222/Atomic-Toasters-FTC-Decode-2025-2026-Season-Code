package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "AutonPatherRR", group = "Utility")
public class AutonPatherRR extends LinearOpMode {

    private SampleMecanumDrive drive;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize Road Runner drive ---
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, drive));

        // --- Vision (AprilTag) ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            // === Road Runner Pose ===
            drive.update(); // update localizer
            Pose2d pose = drive.getPoseEstimate();

            // === AprilTag Detection ===
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();

            telemetry.addData("X (in)", pose.getX());
            telemetry.addData("Y (in)", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));

            if (tagDetected) {
                AprilTagDetection tag = detections.get(0);
                AprilTagPoseFtc tagPose = tag.ftcPose;
                if (tagPose != null) {
                    telemetry.addLine("---- AprilTag Detected ----");
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Range (m)", tagPose.range);
                    telemetry.addData("Bearing (deg)", tagPose.bearing);
                    telemetry.addData("Yaw (deg)", tagPose.yaw);
                    telemetry.addData("Pitch (deg)", tagPose.pitch);
                    telemetry.addData("Roll (deg)", tagPose.roll);
                }
            } else {
                telemetry.addData("AprilTag", "Not Detected");
            }

            telemetry.update();
        }
    }
}
