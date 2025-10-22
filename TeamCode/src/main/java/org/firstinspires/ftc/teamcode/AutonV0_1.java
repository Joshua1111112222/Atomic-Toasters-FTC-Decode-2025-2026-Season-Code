package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonV0_1", group = "Main")
public class AutonV0_1 extends LinearOpMode {

    // --- Flywheels ---
    DcMotorEx flyLeft, flyRight;
    private static final double kP = 0.002;
    private static final double kF = 0.0006;
    private static final double ticksPerRev = 28.0;
    private static final double gearRatio = 1.0;

    // --- Conveyor + Intake ---
    CRServo conveyorLeft, conveyorRight, conveyorLeft2;
    DcMotor intakeMotor;

    // --- Constants ---
    private static final double TARGET_RPM = 2500;
    private static final double RPM_TOLERANCE = 200;        // ±200 RPM window
    private static final double RPM_DROP_THRESHOLD = 100;   // stop conveyors if drop >100
    private static final double CONVEYOR_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;

    // --- Shot control ---
    private static final int MAX_SHOTS = 3;
    private int shotsFired = 0;
    private boolean feeding = false;
    private boolean shotDetected = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware map ---
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft.setDirection(DcMotorEx.Direction.REVERSE);

        conveyorLeft = hardwareMap.get(CRServo.class, "conveyorLeft");
        conveyorRight = hardwareMap.get(CRServo.class, "conveyorRight");
        conveyorLeft2 = hardwareMap.get(CRServo.class, "conveyorLeft2");
        conveyorLeft.setDirection(CRServo.Direction.REVERSE);
        conveyorLeft2.setDirection(CRServo.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Ready to run Auto Shooting.");
        telemetry.update();

        waitForStart();

        double targetVelocity = (TARGET_RPM / 60.0) * ticksPerRev * gearRatio;

        while (opModeIsActive() && shotsFired < MAX_SHOTS) {

            // --- Flywheel control loop ---
            double velL = flyLeft.getVelocity();
            double velR = flyRight.getVelocity();

            double rpmL = (velL * 60.0) / (ticksPerRev * gearRatio);
            double rpmR = (velR * 60.0) / (ticksPerRev * gearRatio);
            double avgRPM = (rpmL + rpmR) / 2.0;

            double errorL = targetVelocity - velL;
            double errorR = targetVelocity - velR;

            double powerL = (kP * errorL) + (kF * targetVelocity);
            double powerR = (kP * errorR) + (kF * targetVelocity);

            flyLeft.setPower(powerL);
            flyRight.setPower(powerR);

            // --- Shooting logic ---
            if (!feeding && Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                // Up to speed → start feeding
                feeding = true;
                conveyorLeft.setPower(CONVEYOR_POWER);
                conveyorRight.setPower(CONVEYOR_POWER);
                conveyorLeft2.setPower(CONVEYOR_POWER);
                intakeMotor.setPower(INTAKE_POWER);
                shotDetected = false;
            }

            else if (feeding && avgRPM < (TARGET_RPM - RPM_DROP_THRESHOLD)) {
                // RPM dropped → a shot likely fired
                feeding = false;
                shotDetected = true;
                conveyorLeft.setPower(0);
                conveyorRight.setPower(0);
                conveyorLeft2.setPower(0);
                intakeMotor.setPower(0);
            }

            // Count shot after RPM recovery
            if (shotDetected && Math.abs(avgRPM - TARGET_RPM) <= RPM_TOLERANCE) {
                shotsFired++;
                shotDetected = false;
                telemetry.addData("Shot Fired!", shotsFired);
            }

            telemetry.addData("Flywheel Avg RPM", avgRPM);
            telemetry.addData("Shots Fired", shotsFired);
            telemetry.addData("Feeding State", feeding ? "RUNNING" : "WAITING");
            telemetry.update();
        }

        // --- Stop everything after 3 shots ---
        flyLeft.setPower(0);
        flyRight.setPower(0);
        conveyorLeft.setPower(0);
        conveyorRight.setPower(0);
        conveyorLeft2.setPower(0);
        intakeMotor.setPower(0);

        telemetry.addLine("✅ Finished 3 shots!");
        telemetry.update();
        sleep(2000);
    }
}
