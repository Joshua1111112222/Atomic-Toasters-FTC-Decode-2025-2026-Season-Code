package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "AutonPather", group = "Utility")
public class AutonPather extends LinearOpMode {

    private DcMotor motorFrontLeft, motorFrontRight;
    private IMU imu;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Motors (Encoders) ---
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

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
            // === Encoder readings ===
            int encoderX = -motorFrontLeft.getCurrentPosition();  // reversed as before
            int encoderY = -motorFrontRight.getCurrentPosition(); // reversed as before

            // === IMU Heading ===
            double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // === AprilTag Detection ===
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();

            telemetry.addData("Encoder X (FrontLeft)", encoderX);
            telemetry.addData("Encoder Y (FrontRight)", encoderY);
            telemetry.addData("IMU Heading (deg)", headingDeg);

            if (tagDetected) {
                AprilTagDetection tag = detections.get(0);
                AprilTagPoseFtc pose = tag.ftcPose;
                if (pose != null) {
                    telemetry.addLine("---- AprilTag Detected ----");
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Range (m)", pose.range);
                    telemetry.addData("Bearing (deg)", pose.bearing);
                    telemetry.addData("Yaw (deg)", pose.yaw);
                    telemetry.addData("Pitch (deg)", pose.pitch);
                    telemetry.addData("Roll (deg)", pose.roll);
                }
            } else {
                telemetry.addData("AprilTag", "Not Detected");
            }

            telemetry.update();
        }
    }
}
