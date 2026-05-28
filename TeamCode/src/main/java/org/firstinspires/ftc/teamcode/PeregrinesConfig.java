package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class PeregrinesConfig {

    static DcMotorEx intake, rhinoL, rhinoR;
    static Servo eat;
    TelemetryManager telemetryManager;
    HeadingPIDFController headingPIDFController = new HeadingPIDFController();
    GoBildaPinpointDriver pinpoint;

    static ElapsedTime ballTimer = new ElapsedTime();
    public static boolean isIntaking = false;
    public static boolean isLaunching = false;

    public static PIDFCoefficients flywheelPIDF = new PIDFCoefficients(70,0,0,0.35);
    public static double heading_p = 0, heading_d = 0, heading_f = 0;
    public static double flywheelVelocity = 2900;

    public static void launchBalls() {
        if (!isLaunching) {
            ballTimer.reset();
            rhinoL.setVelocity(flywheelVelocity);
            rhinoR.setVelocity(flywheelVelocity);
            eat.setPosition(0.4);
            intake.setPower(1);
            isLaunching = true;
        }
        if (isLaunching && ballTimer.seconds() >= 0.5) {
            rhinoL.setPower(0);
            rhinoR.setPower(0);
            eat.setPosition(0);
            intake.setPower(0);
            isLaunching = false;
        }
    }

    public static void toggleIntake() {
        if (!isIntaking) {
            intake.setPower(1);
        }
        if (isIntaking) {
            intake.setPower(0);
        }
    }
}