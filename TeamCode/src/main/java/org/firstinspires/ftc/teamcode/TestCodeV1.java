package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp
public class TestCodeV1 extends LinearOpMode {

    // -------------------- Intake + Sorting --------------------
    Servo sorterServo;
    DcMotorEx conveyorGreen, conveyorPurple;
    ColorSensor intakeColorSensor;
    int artifactCount = 0;

    // -------------------- Flywheels --------------------
    DcMotorEx flyLeft, flyRight;
    Servo greenStorageServo, purpleStorageServo;

    boolean flywheelOn = false;
    boolean flywheelReady = false;
    boolean lastBumper = false;

    // Flywheel PIDF
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double TARGET_RPM = 5000;
    private static final double TOLERANCE = 100;
    private double targetVelocity;

    // -------------------- Turret --------------------
    DcMotorEx turretMotor;
    boolean autoAim = false;
    boolean lastA = false;
    private static final double TURRET_KP = 0.02;

    // -------------------- Vision --------------------
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drivetrain ---
        DcMotorEx motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        DcMotorEx motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        DcMotorEx motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        DcMotorEx motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders for positional telemetry
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // --- Intake + Sorting ---
        sorterServo = hardwareMap.servo.get("sorterServo");
        conveyorGreen = hardwareMap.get(DcMotorEx.class, "conveyorGreen");
        conveyorPurple = hardwareMap.get(DcMotorEx.class, "conveyorPurple");
        intakeColorSensor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");

        // --- Flywheels ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");

        flyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        greenStorageServo = hardwareMap.servo.get("greenStorageServo");
        purpleStorageServo = hardwareMap.servo.get("purpleStorageServo");

        targetVelocity = (TARGET_RPM / 60.0) * TICKS_PER_REV * GEAR_RATIO;

        // --- Turret ---
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Vision ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // -------------------- WAIT FOR START --------------------
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // === Field-Centric Drive ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // strafe adjustment

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            motorFrontLeft.setPower((rotY + rotX + rx) / denominator);
            motorBackLeft.setPower((rotY - rotX + rx) / denominator);
            motorFrontRight.setPower((rotY - rotX - rx) / denominator);
            motorBackRight.setPower((rotY + rotX - rx) / denominator);

            // === Intake + Sorting (HSV Color Detection) ===
            int r = intakeColorSensor.red();
            int g = intakeColorSensor.green();
            int b = intakeColorSensor.blue();

            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);
            float hue = hsv[0];

            if (hue >= 260 && hue <= 320) {         // Purple
                sorterServo.setPosition(1.0);
                conveyorPurple.setPower(1.0);
                artifactCount++;
                telemetry.addLine("Artifact: PURPLE");
            } else if (hue >= 80 && hue <= 160) {  // Green
                sorterServo.setPosition(0.0);
                conveyorGreen.setPower(1.0);
                artifactCount++;
                telemetry.addLine("Artifact: GREEN");
            } else {
                conveyorGreen.setPower(0);
                conveyorPurple.setPower(0);
            }

            // Safety eject if too many artifacts
            if (artifactCount > 3) {
                conveyorGreen.setPower(-1.0);
                conveyorPurple.setPower(-1.0);
                telemetry.addLine("Overfilled! Reversing...");
                artifactCount--;
            }

            // === Flywheels Toggle ===
            if (gamepad1.right_bumper && !lastBumper) {
                flywheelOn = !flywheelOn;
            }
            lastBumper = gamepad1.right_bumper;

            if (flywheelOn) {
                setFlywheelVelocity(flyLeft, targetVelocity);
                setFlywheelVelocity(flyRight, targetVelocity);

                flywheelReady =
                        Math.abs(flyLeft.getVelocity() - targetVelocity) < TOLERANCE &&
                                Math.abs(flyRight.getVelocity() - targetVelocity) < TOLERANCE;

                if (flywheelReady) gamepad1.rumble(200);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                flywheelReady = false;
            }

            // === Launch Control ===
            if (flywheelReady) {
                greenStorageServo.setPosition(gamepad1.x ? 1.0 : 0.0);
                purpleStorageServo.setPosition(gamepad1.y ? 1.0 : 0.0);
            }

            // === Turret Auto-Aim ===
            if (gamepad1.a && !lastA) {
                autoAim = !autoAim;
            }
            lastA = gamepad1.a;

            if (autoAim) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                if (detections != null && !detections.isEmpty()) {
                    AprilTagDetection tag = detections.get(0);
                    AprilTagPoseFtc pose = tag.ftcPose;

                    double bearingDeg = pose.bearing;
                    double turretPower = TURRET_KP * bearingDeg;

                    turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
                    turretMotor.setPower(turretPower);

                    telemetry.addData("Turret Auto-Aim", "ON");
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Bearing (deg)", "%.1f", bearingDeg);
                    telemetry.addData("Turret Power", "%.2f", turretPower);
                } else {
                    turretMotor.setPower(0);
                    telemetry.addData("Turret Auto-Aim", "ON - No Tag");
                }
            } else {
                turretMotor.setPower(0);
                telemetry.addData("Turret Auto-Aim", "OFF");
            }

            // === Telemetry ===
            telemetry.addData("XPOS (Front Left Encoder)", motorFrontLeft.getCurrentPosition());
            telemetry.addData("YPOS (Front Right Encoder)", motorFrontRight.getCurrentPosition());
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Artifact Count", artifactCount);
            telemetry.addData("Flywheel Vel L", flyLeft.getVelocity());
            telemetry.addData("Flywheel Vel R", flyRight.getVelocity());
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.update();
        }
    }

    // -------------------- Utility: Flywheel PIDF --------------------
    private void setFlywheelVelocity(DcMotorEx motor, double targetTicksPerSec) {
        double error = targetTicksPerSec - motor.getVelocity();
        double output = (kP * error) + (kF * targetTicksPerSec);
        motor.setPower(Math.min(Math.max(output, 0), 1));
    }
}
//github things