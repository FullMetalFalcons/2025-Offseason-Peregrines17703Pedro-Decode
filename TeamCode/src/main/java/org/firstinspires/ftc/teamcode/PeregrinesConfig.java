package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    static ElapsedTime spinUp = new ElapsedTime();
    public static boolean isIntaking = false;
    public static boolean isLaunching = false;

    public static PIDFCoefficients flywheelPIDF = new PIDFCoefficients(70, 0, 0, 0.35);
    public static double heading_p = 0, heading_d = 0, heading_f = 0;

    public static double flywheelVelocity = 2135;

    public static void launchBalls() {
        if (!isLaunching) {
            ballTimer.reset();
            intake.setPower(1);
            isLaunching = true;
        }
    }

    public static void spinUpFlyWheels() {
        rhinoL.setVelocity(flywheelVelocity);
        rhinoR.setVelocity(flywheelVelocity-62);
        eat.setPosition(0.4);
    }

    public static void update() {
        if (isLaunching && ballTimer.seconds() >= 1.25) {
            rhinoL.setPower(0);
            rhinoR.setPower(0);
            eat.setPosition(0);
            intake.setPower(0);
            isLaunching = false;
        }
        if (!isIntaking && !isLaunching) {
            intake.setPower(0);
        }
        if (isIntaking) {
            intake.setPower(1);
        }
    }

    public static void init(HardwareMap hardwaremap) {
        intake = (DcMotorEx) hardwaremap.dcMotor.get("intake");
        rhinoL = (DcMotorEx) hardwaremap.dcMotor.get("rhinoL");
        rhinoR = (DcMotorEx) hardwaremap.dcMotor.get("rhinoR");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rhinoL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhinoR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rhinoL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rhinoR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        rhinoL.setDirection(DcMotorSimple.Direction.REVERSE);
        rhinoR.setDirection(DcMotorSimple.Direction.FORWARD);

        eat = (Servo) hardwaremap.servo.get("eat");
        eat.setPosition(0);

    }

    public static void toggleIntake() {
        isIntaking = !isIntaking;
    }
}