package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(name="AutoV1", group="Linear Opmode")
public class AutoV1 extends LinearOpMode {

    private DcMotorEx flywheelLeft, flywheelRight;
    private Servo greenStorageServo, purpleStorageServo;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Hardware
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        greenStorageServo = hardwareMap.get(Servo.class, "greenStorageServo");
        purpleStorageServo = hardwareMap.get(Servo.class, "purpleStorageServo");

        // Vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera((CameraName) hardwareMap.get("Webcam 1")) // <-- this is the problem
                .addProcessor(aprilTag)
                .build();


        telemetry.addLine("Initialized - looking for AprilTag...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 1. Detect AprilTag
            String pattern = "PPP"; // default
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                int tagId = detections.get(0).id;
                if (tagId == 1) pattern = "GPP"; // Green, Purple, Purple
                else if (tagId == 2) pattern = "PGP"; // Purple, Green, Purple
                else if (tagId == 3) pattern = "PPG"; // Purple, Purple, Green
            }

            telemetry.addData("Pattern", pattern);
            telemetry.update();

            // 2. Spin up flywheels
            setFlywheelRPM(2000); // example RPM
            sleep(1500); // wait until at speed

            // 3. Launch artifacts in correct pattern
            for (char c : pattern.toCharArray()) {
                if (c == 'G') {
                    greenStorageServo.setPosition(1.0); // open green
                    sleep(700);
                    greenStorageServo.setPosition(0.0); // close
                } else if (c == 'P') {
                    purpleStorageServo.setPosition(1.0); // open purple
                    sleep(700);
                    purpleStorageServo.setPosition(0.0); // close
                }
                sleep(1000); // time between launches
            }

            // stop flywheels
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
        }
    }

    private void setFlywheelRPM(double targetRPM) {
        // Example: simple velocity control
        double ticksPerRev = 28; // adjust for your motor
        double targetTicksPerSecond = (targetRPM / 60.0) * ticksPerRev;
        flywheelLeft.setVelocity(targetTicksPerSecond);
        flywheelRight.setVelocity(targetTicksPerSecond);
    }
}
