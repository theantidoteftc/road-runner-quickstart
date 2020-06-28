package org.firstinspires.ftc.teamcode.drive;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1440;
    public static double WHEEL_RADIUS = 1.14173; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.75174; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4.3125; // in; offset of the lateral wheel

    public static double MULTIPLIER_FORWARD = 1.002123993327; //fudging woooooo
    public static double MULTIPLIER_SIDEWAYS = 1.000791754896;

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("fL");
        rightEncoder = hardwareMap.dcMotor.get("fR");
        frontEncoder = hardwareMap.dcMotor.get("rR"); //reversed
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * MULTIPLIER_FORWARD,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * MULTIPLIER_FORWARD,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * MULTIPLIER_SIDEWAYS
        );
    }
}
