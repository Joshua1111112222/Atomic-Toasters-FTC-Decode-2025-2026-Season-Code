package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestCodeV0_4 extends LinearOpMode {

    // Intake motor (GoBILDA)
    DcMotor intakeMotor;

    // Belt + Shooter
    DcMotorEx beltMotor;
    DcMotorEx shooterLeft, shooterRight;

    // Gate servo
    Servo gateServo;

    // States
    boolean beltOn = false;
    boolean shooterOn = false;

    // Shooter target RPM
    final double SHOOTER_RPM = 3000; // adjust for your robot

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drivetrain ---
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and enable encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU for field-centric ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // --- Intake (GoBILDA motor) ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Belt ---
        beltMotor = hardwareMap.get(DcMotorEx.class, "beltMotor");
        beltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Shooter ---
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Gate Servo ---
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        waitForStart();

        while (opModeIsActive()) {
            // --- Field-centric drive ---
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

            // --- Intake control (triggers) ---
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.2) intakePower = 1.0;
            else if (gamepad1.left_trigger > 0.2) intakePower = -1.0;

            intakeMotor.setPower(intakePower);

            // --- Belt (toggle with left bumper) ---
            if (gamepad1.left_bumper) {
                beltOn = !beltOn;
                sleep(200); // debounce
            }
            beltMotor.setPower(beltOn ? 0.4 : 0.0);

            // --- Shooter (toggle with right bumper) ---
            if (gamepad1.right_bumper) {
                shooterOn = !shooterOn;
                sleep(200); // debounce
            }

            if (shooterOn) {
                double ticksPerRev = 28.0; // adjust if not correct for your motor
                double ticksPerSecond = (SHOOTER_RPM / 60.0) * ticksPerRev;

                shooterLeft.setVelocity(ticksPerSecond);
                shooterRight.setVelocity(ticksPerSecond);
            } else {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Belt On", beltOn);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Shooter Left Velocity (ticks/s)", shooterLeft.getVelocity());
            telemetry.addData("Shooter Right Velocity (ticks/s)", shooterRight.getVelocity());

            // Drivetrain encoder telemetry (reversed values for FL & FR)
            telemetry.addData("Front Left Encoder (reversed)", -motorFrontLeft.getCurrentPosition());
            telemetry.addData("Front Right Encoder (reversed)", -motorFrontRight.getCurrentPosition());

            telemetry.update();
        }
    }
}
