package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import android.util.Log;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

@Config
@Autonomous(group = "drive")
public class FeedForwardTuner2 extends LinearOpMode {
    public static double DISTANCE = 72; // in

    private static final String PID_VAR_NAME = "FF_TUNER";

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private String catName;
    private CustomVariable catVar;

    private SampleMecanumDrive drive;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                DriveConstants.BASE_CONSTRAINTS.maxVel,
                DriveConstants.BASE_CONSTRAINTS.maxAccel,
                DriveConstants.BASE_CONSTRAINTS.maxJerk);
    }

    /*private void addPidVariable() {
        return;

        catName = getClass().getSimpleName();
        catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null) {
            // this should never happen...
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);

            RobotLog.w("Unable to find top-level category %s", catName);
        }

        CustomVariable pidVar = new CustomVariable();
        pidVar.putVariable("kP", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kP;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(value, coeffs.kI, coeffs.kD));
            }
        }));
        pidVar.putVariable("kI", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kI;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(coeffs.kP, value, coeffs.kD));
            }
        }));
        pidVar.putVariable("kD", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).kD;
            }

            @Override
            public void set(Double value) {
                PIDCoefficients coeffs = drive.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDCoefficients(coeffs.kP, coeffs.kI, value));
            }
        }));

        catVar.putVariable(PID_VAR_NAME, pidVar);
        dashboard.updateConfig();
    }*/

    /*private void removePidVariable() {
        return;

        if (catVar.size() > 1) {
            catVar.removeVariable(PID_VAR_NAME);
        } else {
            dashboard.getConfigRoot().removeVariable(catName);
        }
        dashboard.updateConfig();
    }*/

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            // calculate and set the motor power
            double profileTime = clock.seconds() - profileStart;

            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = kV * motionState.getV();
            drive.setDrivePower(new Pose2d(targetPower, 0, 0));

            TelemetryPacket packet = new TelemetryPacket();

            // update telemetry
            packet.put("targetVelocity", motionState.getV());

            Log.e("NOAHISAWESOME", Boolean.toString(drive.myLocalizer == null));
            Log.e("NOAHISAWESOME", Boolean.toString(drive.myLocalizer.getPoseVelocity() == null));

            Pose2d poseVelo = drive.myLocalizer.getPoseVelocity();
            double velo = Math.hypot(poseVelo.getX(), poseVelo.getY());
            packet.put("poseVelocity", velo);
            packet.put("error", motionState.getV() - velo);

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}