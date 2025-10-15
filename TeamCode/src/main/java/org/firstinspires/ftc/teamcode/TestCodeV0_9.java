package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp
public class TestCodeV0_9 extends LinearOpMode {

    // --- Intake ---
    DcMotor intakeMotor;

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    boolean flywheelOn = false;
    boolean flywheelReady = false;
    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    // Flywheel PIDF constants
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;
    private static final double closeRPM = 2500;
    private static final double farRPM = 2700;
    private static final double tolerance = 100;

    private double targetRPM = 0;
    private double targetVelocity = 0;

    // --- Vision ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean autoAim = false;
    boolean lastA = false;
    private static final double CHASSIS_KP = 0.02;
    private static final double SMOOTH_FACTOR = 0.3;
    Double targetHeading = null;

    // --- Conveyor ---
    CRServo conveyorLeft, conveyorRight, conveyorLeft2;
    boolean conveyorOn = false;
    boolean lastB = false;
    private static final double CONVEYOR_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Drivetrain ---
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- IMU ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        // --- Intake ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Flywheels ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Conveyor ---
        conveyorLeft = hardwareMap.get(CRServo.class, "conveyorLeft");
        conveyorRight = hardwareMap.get(CRServo.class, "conveyorRight");
        conveyorLeft2 = hardwareMap.get(CRServo.class, "conveyorLeft2"); // ✅ FIXED initialization

        conveyorLeft.setDirection(CRServo.Direction.REVERSE);
        conveyorLeft2.setDirection(CRServo.Direction.REVERSE);

        // --- Vision / AprilTag ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        double smoothCorrection = 0;

        while (opModeIsActive()) {

            // === Field-centric drive ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // --- AprilTag Detection ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();

            // === Auto Align Toggle ===
            if (gamepad1.a && !lastA) {
                autoAim = !autoAim;
                if (!autoAim) targetHeading = null;
            }
            lastA = gamepad1.a;

            if (autoAim) {
                if (tagDetected) {
                    AprilTagDetection tag = detections.get(0);
                    AprilTagPoseFtc pose = tag.ftcPose;
                    double desiredHeading = botHeading + Math.toRadians(pose.bearing);
                    targetHeading = desiredHeading;
                    telemetry.addData("AutoAlign", "Tracking Tag ID: %d | Bearing: %.2f°", tag.id, pose.bearing);
                }

                if (targetHeading != null) {
                    double headingError = targetHeading - botHeading;
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    double correction = CHASSIS_KP * headingError;
                    smoothCorrection = smoothCorrection * (1 - SMOOTH_FACTOR) + correction * SMOOTH_FACTOR;
                    rx = smoothCorrection;
                }
            } else {
                smoothCorrection = 0;
            }

            // === Conveyor Toggle ===
            if (gamepad1.b && !lastB) {
                conveyorOn = !conveyorOn;
            }
            lastB = gamepad1.b;

            if (conveyorOn) {
                conveyorLeft.setPower(CONVEYOR_POWER);
                conveyorRight.setPower(CONVEYOR_POWER);
                conveyorLeft2.setPower(CONVEYOR_POWER);
            } else {
                conveyorLeft.setPower(0);
                conveyorRight.setPower(0);
                conveyorLeft2.setPower(0);
            }

            // === Apply drivetrain power ===
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

            // === Intake Control ===
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) {
                intakePower = 1.0;
            } else if (gamepad1.left_trigger > 0.2) {
                intakePower = -1.0;
            }
            intakeMotor.setPower(intakePower);

            // === Flywheel Control ===
            if (gamepad1.left_bumper && !lastLeftBumper) {
                if (flywheelOn && targetRPM == closeRPM) {
                    flywheelOn = false;
                } else {
                    targetRPM = closeRPM;
                    flywheelOn = true;
                }
            }
            if (gamepad1.right_bumper && !lastRightBumper) {
                if (flywheelOn && targetRPM == farRPM) {
                    flywheelOn = false;
                } else {
                    targetRPM = farRPM;
                    flywheelOn = true;
                }
            }

            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            if (flywheelOn) {
                targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;
                double velL = flyLeft.getVelocity();
                double velR = flyRight.getVelocity();
                double rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
                double rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);
                double errorL = targetVelocity - velL;
                double errorR = targetVelocity - velR;
                double powerL = (kP * errorL) + (kF * targetVelocity);
                double powerR = (kP * errorR) + (kF * targetVelocity);
                flyLeft.setPower(powerL);
                flyRight.setPower(powerR);
                flywheelReady = (Math.abs(rpmL - targetRPM) < tolerance &&
                        Math.abs(rpmR - targetRPM) < tolerance);
                telemetry.addData("Flywheel Mode", targetRPM == closeRPM ? "CLOSE (2500)" : "FAR (2700)");
                telemetry.addData("Flywheel L RPM", rpmL);
                telemetry.addData("Flywheel R RPM", rpmR);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                flywheelReady = false;
            }

            // === Telemetry ===
            telemetry.addData("AutoAlign Active", autoAim);
            telemetry.addData("AprilTag Detected", tagDetected);
            telemetry.addData("Conveyor On", conveyorOn);
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
        }
    }
}
