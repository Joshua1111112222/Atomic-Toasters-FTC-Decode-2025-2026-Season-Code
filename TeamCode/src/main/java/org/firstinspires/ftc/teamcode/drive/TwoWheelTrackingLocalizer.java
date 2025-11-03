/*
 * Copyright (c) 2025 Team 21518 - The Atomic Toasters
 * Licensed under the No Copy, No Modify License (see LICENSE-TeamAtomicToasters)
 * You MAY NOT copy, modify, or redistribute this file without permission.
 */

package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/**
 * Custom two-wheel tracking localizer for Team 21518.
 * Uses front left and front right motors as encoders for X and Y movement.
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 538.0; // goBILDA 312 RPM or similar
    public static double WHEEL_RADIUS = 2.0; // in
    public static double GEAR_RATIO = 1.0;   // output (wheel) speed / input (encoder) speed

    // Position of tracking wheels relative to robot center (in inches)
    public static double PARALLEL_X = 0; // forward offset
    public static double PARALLEL_Y = 0; // lateral offset

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 0;

    // Encoders
    private final Encoder parallelEncoder;       // X-axis movement (forward/back)
    private final Encoder perpendicularEncoder;  // Y-axis movement (strafe)
    private final SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                // parallel encoder (forward)
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                // perpendicular encoder (strafe)
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        // Use your drive motors as encoders
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFrontLeft"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motorFrontRight"));

        // Reverse directions to match encoder readings from AutonPather
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }
}
