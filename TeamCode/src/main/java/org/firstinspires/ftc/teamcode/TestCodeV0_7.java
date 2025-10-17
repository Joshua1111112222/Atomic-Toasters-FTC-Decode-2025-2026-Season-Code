package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestCodeV0_7 extends LinearOpMode {

    // --- Intake ---
    DcMotor intakeMotor;

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    boolean flywheelOn = false;
    boolean flywheelReady = false;
    boolean lastBumper = false;

    // --- LED ---
    RevBlinkinLedDriver blinkin;
    RevBlinkinLedDriver.BlinkinPattern currentPattern = RevBlinkinLedDriver.BlinkinPattern.RED;

    // --- Flywheel constants ---
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;
    private static final double targetRPM = 3000;
    private static final double tolerance = 200; // ±200 RPM
    private double targetVelocity;

    // --- Timer ---
    private double startTime;
    private final double matchTimeSec = 90.0; // 1 minute 30 seconds

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

        // --- LED setup ---
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        waitForStart();
        startTime = getRuntime();

        targetVelocity = (targetRPM / 60.0) * ticksPerRev * gearRatio;

        while (opModeIsActive()) {
            double elapsed = getRuntime() - startTime;

            // === Field-centric drive ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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

            // === Intake ===
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) {
                intakePower = 1.0;
            } else if (gamepad1.left_trigger > 0.2) {
                intakePower = -1.0;
            }
            intakeMotor.setPower(intakePower);

            // === Flywheel Toggle ===
            if (gamepad1.right_bumper && !lastBumper) {
                flywheelOn = !flywheelOn;
            }
            lastBumper = gamepad1.right_bumper;

            double rpmL = 0, rpmR = 0;
            if (flywheelOn) {
                double velL = flyLeft.getVelocity();
                double velR = flyRight.getVelocity();
                rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
                rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);

                double errorL = targetVelocity - velL;
                double errorR = targetVelocity - velR;
                double powerL = (kP * errorL) + (kF * targetVelocity);
                double powerR = (kP * errorR) + (kF * targetVelocity);

                flyLeft.setPower(powerL);
                flyRight.setPower(powerR);

                flywheelReady = (Math.abs(rpmL - targetRPM) < tolerance &&
                        Math.abs(rpmR - targetRPM) < tolerance);
            } else {
                flyLeft.setPower(0);
                flyRight.setPower(0);
                flywheelReady = false;
            }

            // === LED Logic ===
            boolean timerExpired = elapsed >= matchTimeSec;
            RevBlinkinLedDriver.BlinkinPattern newPattern = currentPattern;

            if (!timerExpired) {
                // Before 1:30 → Red unless flywheel ready
                newPattern = flywheelReady ?
                        RevBlinkinLedDriver.BlinkinPattern.GREEN :
                        RevBlinkinLedDriver.BlinkinPattern.RED;
            } else {
                // After 1:30 → Flowing rainbow by default, green override when ready
                newPattern = flywheelReady ?
                        RevBlinkinLedDriver.BlinkinPattern.GREEN :
                        RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE;
            }

            if (newPattern != currentPattern) {
                blinkin.setPattern(newPattern);
                currentPattern = newPattern;
            }

            // === Telemetry ===
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Flywheel On", flywheelOn);
            telemetry.addData("Flywheel Ready", flywheelReady);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Flywheel L RPM", rpmL);
            telemetry.addData("Flywheel R RPM", rpmR);
            telemetry.addData("Timer", String.format("%.1f / %.1f sec", elapsed, matchTimeSec));
            telemetry.addData("LED Pattern", currentPattern);
            telemetry.update();
        }
    }
}
//github things