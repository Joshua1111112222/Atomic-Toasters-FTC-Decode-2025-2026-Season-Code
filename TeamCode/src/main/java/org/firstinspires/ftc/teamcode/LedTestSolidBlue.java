package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "LED Test - Solid Blue")
public class LedTestSolidBlue extends LinearOpMode {

    RevBlinkinLedDriver blinkin;

    @Override
    public void runOpMode() throws InterruptedException {

        // Map the LED driver
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        // Wait for start button
        waitForStart();

        // Set the LEDs solid blue
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        // Keep OpMode running so LEDs stay on
        while (opModeIsActive()) {
            idle(); // let the system breathe
        }
    }
}
