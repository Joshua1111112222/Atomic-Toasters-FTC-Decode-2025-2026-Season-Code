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

    // Tuning: angular control
    private static final double CHASSIS_KP = 0.02;
    private static final double SMOOTH_FACTOR = 0.3;

    // Tuning: range control
    private static final double TARGET_RANGE_M = 1.0;
    private static final double RANGE_KP = 0.6;
    private static final double FORWARD_BACK_MAX = 0.35;

    // Search / reacquire
    private static final double TAG_LOST_TIMEOUT = 0.25;
    private static final double SEARCH_ROTATION_SPEED = 0.18;
    private static final double SEARCH_BACKUP_SPEED = 0.18;

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

        // --- IMU (Control Hub IMU) ---
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
        conveyorLeft2 = hardwareMap.get(CRServo.class, "conveyorLeft2");

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
        double lastSeenBearingDeg = 0.0;
        double lastTagSeenTime = -1000.0;
        boolean searching = false;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // --- AprilTag Detection ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagDetected = !detections.isEmpty();
            AprilTagPoseFtc lastPose = null;

            // === Auto Align Toggle ===
            if (gamepad1.a && !lastA) {
                autoAim = !autoAim;
                if (!autoAim) {
                    targetHeading = null;
                    searching = false;
                } else {
                    smoothCorrection = 0.0;
                    lastSeenBearingDeg = 0.0;
                    lastTagSeenTime = -1000.0;
                    searching = false;
                }
            }
            lastA = gamepad1.a;

            // === Auto-align logic ===
            if (autoAim) {
                rotatedX = 0.0;

                if (tagDetected) {
                    AprilTagDetection tag = detections.get(0);
                    if (tag.ftcPose != null) {
                        lastPose = tag.ftcPose;
                        lastSeenBearingDeg = lastPose.bearing;
                        lastTagSeenTime = getRuntime();
                        searching = false;

                        double headingErrorRad = Math.toRadians(lastSeenBearingDeg);
                        double correction = CHASSIS_KP * headingErrorRad;
                        smoothCorrection = smoothCorrection * (1 - SMOOTH_FACTOR) + correction * SMOOTH_FACTOR;
                        rx = smoothCorrection;

                        double range = lastPose.range;
                        double rangeError = range - TARGET_RANGE_M;
                        double forwardPower = RANGE_KP * rangeError;
                        if (forwardPower > FORWARD_BACK_MAX) forwardPower = FORWARD_BACK_MAX;
                        if (forwardPower < -FORWARD_BACK_MAX) forwardPower = -FORWARD_BACK_MAX;
                        rotatedY = forwardPower;

                        telemetry.addData("AutoAlign", String.format("Tag ID %d | Bearing: %.2f° | Range: %.2fm", tag.id, lastSeenBearingDeg, range));
                    } else {
                        tagDetected = false;
                    }
                }

                if (!tagDetected) {
                    double timeSinceSeen = getRuntime() - lastTagSeenTime;

                    if (timeSinceSeen <= TAG_LOST_TIMEOUT) {
                        double headingErrorRad = Math.toRadians(lastSeenBearingDeg);
                        double correction = CHASSIS_KP * headingErrorRad;
                        smoothCorrection = smoothCorrection * (1 - SMOOTH_FACTOR) + correction * SMOOTH_FACTOR;
                        rx = smoothCorrection;
                        rotatedY = 0.0;

                        telemetry.addData("AutoAlign", String.format("Lost briefly - using last bearing %.2f°", lastSeenBearingDeg));
                    } else {
                        searching = true;
                        double sign = (lastSeenBearingDeg == 0.0) ? 1.0 : Math.signum(lastSeenBearingDeg);
                        rx = SEARCH_ROTATION_SPEED * sign;
                        rotatedY = -SEARCH_BACKUP_SPEED;
                        smoothCorrection = 0.0;

                        telemetry.addData("AutoAlign", String.format("Searching (last %.2f°, %.2fs ago)", lastSeenBearingDeg, timeSinceSeen));
                    }
                }
            } else {
                smoothCorrection = 0;
            }

            // === Conveyor Control ===
            if (gamepad1.b && !lastB) {
                conveyorOn = !conveyorOn;
            }
            lastB = gamepad1.b;

            double conveyorPower = 0.0;
            if (gamepad1.y) {
                conveyorPower = -CONVEYOR_POWER;
            } else if (conveyorOn) {
                conveyorPower = CONVEYOR_POWER;
            }

            conveyorLeft.setPower(conveyorPower);
            conveyorRight.setPower(conveyorPower);
            conveyorLeft2.setPower(conveyorPower);

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
            telemetry.addData("AutoAlign State", searching ? "SEARCHING" : (autoAim ? "ALIGNING" : "OFF"));
            telemetry.addData("Last Seen Bearing (deg)", String.format("%.2f", lastSeenBearingDeg));
            telemetry.addData("Time Since Last Seen (s)", String.format("%.2f", getRuntime() - lastTagSeenTime));
            telemetry.addData("Conveyor On", conveyorOn);
            telemetry.addData("Holding Reverse (Y)", gamepad1.y);
            telemetry.addData("Control Hub IMU Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
        }
    }
}
//github things