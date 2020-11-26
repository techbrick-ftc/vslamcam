package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class SlamraCentric {
    // Variable setup, all will be explained within code
    private DcMotor[] motors;
    private double[] wheelAngles;
    private double r;
    private double theta;
    private double currentAngle;
    private BNO055IMU imu;

    private double[] wheelPowers;

    private T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void setUp(DcMotor[] motors, double[] wheelAngles, BNO055IMU imu, T265Camera slamra) throws Exception {
        // Check if we have angles for every motor, and vice versa
        if (motors.length != wheelAngles.length) {
            throw new Exception("Motor and wheelAngle arrays do not have same length.\nCheck your code!!!");
        }

        this.motors = motors;
        this.wheelAngles = wheelAngles;
        this.wheelPowers = new double[motors.length];
        this.imu = imu;
        this.slamra = slamra;

    }

    public void startDetection() {
        slamra.start();
    }

    private void getAngle() {
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    /**
     * Run every loop to drive robot using field centricity
     * @param turn The input to control robot's turn
     */
    public void Drive(double targetX, double targetY, double speed, double turn, status status) {

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) System.out.println("up is equal to null");

        Translation2d pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);

        double currentX = pose.getX();
        double currentY = pose.getY();

        double diffX = targetX - currentX;
        double diffY = targetY - currentY;

        while (Math.abs(diffX) >= 0.5 || Math.abs(diffY) >= 0.5 && status.opModeIsActive()) {
            up = slamra.getLastReceivedCameraUpdate();
            if (up == null) System.out.println("up is equal to null");

            final int robotRadius = 9; // inches

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            field.strokeCircle(pose.getX(), pose.getY(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = pose.getX() + arrowX  / 2, y1 = pose.getY() + arrowY / 2;
            double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            packet.put("Current X", currentX);
            packet.put("Current Y", currentY);
            packet.put("Target X", targetX);
            packet.put("Target Y", targetY);

            dashboard.sendTelemetryPacket(packet);

            currentX = pose.getX();
            currentY = pose.getY();

            diffX = targetX - currentX;
            diffY = targetY - currentY;

             /*
            Get the current angle
            */
            getAngle();

            /*
            Set theta (for polar coords) to the theta of the point (x,y) to the x-axis at the origin
            (The direction to go)
            */
            theta = Math.atan2(diffX, diffY);

            /*
            Add current angle to account for rotation (since we are getting the theta from a controller
            axis (-1.0 to 1.0) we don't know the angle we are currently at
            */
            double newTheta = theta + currentAngle;

            /*
            Get the angle of the wheel and subtract the newTheta (because newTheta is clockwise and
            math is counter-clockwise)
            */
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(Math.sin(wheelAngles[i] - newTheta) * speed + turn);
                wheelPowers[i] = motors[i].getPower();
            }
        }
    }
}

interface status {
    boolean opModeIsActive();
}
