package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

public class ShootingCalc {

    public static double getVelocityNeeded(double distance, double shootingAngle, double height){
        double GRAVITY = 9.80665;

        double vx = Math.sqrt(
                (GRAVITY * distance * distance) /
                        (2 * Math.pow(Math.cos(shootingAngle), 2) * (distance * Math.tan(shootingAngle) - height))
        );

        double vy = vx * Math.tan(shootingAngle);

        // RETURN TOTAL VELOCITY (FIXED)
        return Math.sqrt(vx * vx + vy * vy);
    }

    public static Vector getShotVector(Vector robotVelocity, Vector robotPosition, Vector targetVector, double shootingAngle, double height){

        Vector toTarget = targetVector.minus(robotPosition);

        double distanceMeters = toTarget.getMagnitude() / 39.3701;

        double velocity = getVelocityNeeded(distanceMeters, shootingAngle, height);

        // direction stays same
        Vector idealShot = new Vector(velocity, toTarget.getTheta());

        // convert robot velocity to m/s before subtracting
        return idealShot.minus(robotVelocity.times(1 / 39.3701));
    }

    public static double flatMagVx(double distance, double height){
        double GRAVITY = 9.80665;
        return Math.sqrt(Math.abs(GRAVITY * distance * distance / 2) / height);
    }

    public static double flatMagVy(double distance, double height){
        double vX = flatMagVx(distance, height);
        double time = distance / vX;
        double GRAVITY = 9.80665;
        return (GRAVITY / 2 * time * time + height) / time;
    }

    public static double flatShootingAngleCalc(double distance, double height){
        return Math.atan2(flatMagVy(distance, height), flatMagVx(distance, height));
    }
}