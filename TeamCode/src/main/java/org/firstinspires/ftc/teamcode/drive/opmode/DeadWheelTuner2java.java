package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import kotlin.jvm.JvmField;

@Autonomous
public class DeadWheelTuner2java extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        this.telemetry.addLine("Press play to begin the track width tuner routine");
        this.waitForStart();
        if (!this.isStopRequested()) {
            this.telemetry.clearAll();
            this.telemetry.addLine("Running...");
            this.telemetry.update();
            drive.turn(Math.toRadians((double)360 * DeadWheelTuner2Settings.NUM_TURNS + (double)270));
            double imuHeadingAccumulator = 0.0D;
            double imuLastHeading = 0.0D;
            double deadWheelHeadingAccumulator = 0.0D;
            double lastDeadWheelHeading = 0.0D;

            while(!this.isStopRequested() && drive.isBusy()) {
                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();
                DashboardUtil.drawRobot(fieldOverlay, new Pose2d(0.0D, 0.0D, drive.getPoseEstimate().getHeading()));
                double imuHeading = drive.getRawExternalHeading();
                imuHeadingAccumulator += Angle.norm(imuHeading - imuLastHeading);
                imuLastHeading = imuHeading;
                double deadWheelHeading = drive.getPoseEstimate().getHeading();
                deadWheelHeadingAccumulator += Angle.norm(deadWheelHeading - lastDeadWheelHeading);
                lastDeadWheelHeading = deadWheelHeading;
                packet.put("imu", Math.toDegrees(imuHeadingAccumulator));
                packet.put("heading", Math.toDegrees(deadWheelHeadingAccumulator));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                drive.update();
            }

        }
    }

    @Config
    public static final class DeadWheelTuner2Settings {
        @JvmField
        public static double NUM_TURNS;
        public static final DeadWheelTuner2java.DeadWheelTuner2Settings INSTANCE;

        private DeadWheelTuner2Settings() {
        }

        static {
            DeadWheelTuner2java.DeadWheelTuner2Settings var0 = new DeadWheelTuner2java.DeadWheelTuner2Settings();
            INSTANCE = var0;
            NUM_TURNS = 10.0D;
        }
    }
}
