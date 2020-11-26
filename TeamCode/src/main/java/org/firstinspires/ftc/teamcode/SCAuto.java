package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import static java.lang.Math.PI;

@Autonomous(name="SCAuto", group="Autonomous")
public class SCAuto extends LinearOpMode implements status{

    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor rl = null;
    private DcMotor rr = null;

    private static T265Camera slamra = null;

    SlamraCentric drive = new SlamraCentric();
    private BNO055IMU imu = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        // adds start telemetry
        telemetry.addData("status", "initialized");
        telemetry.update();

        // configures hardware
        fl = hardwareMap.get(DcMotor.class, "flMotor");
        fr = hardwareMap.get(DcMotor.class, "frMotor");
        rl = hardwareMap.get(DcMotor.class, "rlMotor");
        rr = hardwareMap.get(DcMotor.class, "rrMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(param);
        slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);

        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        DcMotor[] motors = {fr, rr, rl, fl};
        double[] motorAngles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        try {
            packet.addLine("setup starting");
            dashboard.sendTelemetryPacket(packet);
            drive.setUp(motors, motorAngles, imu, slamra);
            packet.addLine("setup completed");
            dashboard.sendTelemetryPacket(packet);
        } catch (Exception e) {
            e.printStackTrace();
        }

        waitForStart();

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);

        if (opModeIsActive()) {
            drive.startDetection();

            drive.Drive(30, 40, 0.5, 0, this);
        }
    }
}
