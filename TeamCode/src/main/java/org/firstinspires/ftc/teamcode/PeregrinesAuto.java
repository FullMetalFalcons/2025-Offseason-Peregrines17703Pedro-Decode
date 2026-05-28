package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "CloseAuto", group = "Auto")
public class PeregrinesAuto extends OpMode {

    public Follower follower;
    private int pathState;
    HeadingPIDFController headingPIDFController = new HeadingPIDFController();
    GoBildaPinpointDriver pinpoint;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    ElapsedTime ballTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    public static Pose endPose = new Pose();
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    public boolean isBlue = true;
    private String location = "blue";

    DcMotorEx intake, rhinoL, rhinoR;
    Servo eat;

    public static PIDFCoefficients flywheelPIDF = new PIDFCoefficients(70,0,0,0.35);
    public static double heading_p = 0, heading_d = 0, heading_f = 0;
    public static double flywheelVelocity = 2900;


    // Define important coordinate locations for the Blue side of the field

    // === Positions ===
    private Pose startPose = new Pose(26, 129, 143);
    private Pose shootPos = new Pose(45, 114, 135);
    private Pose intakeSpikeMarkTwo = new Pose(20, 60, 180);
    private Pose gateIntake = new Pose(10, 63, 140);
    private Pose intakeSpikeMarkOne = new Pose(20, 84, 180);

    // === CP ===
    private Pose CP_SpikeMarkTwo = new Pose(67, 55);
    private Pose CP_GateIntake = new Pose(67, 73);
    private Pose CP_spikeMarkOne = new Pose(67, 80);



    private PathChain launchPreload, getSpikeMarkTwo, shootSpikeMarkTwo, collectGatePath, CollectSpikeMarkOne, shootSpikeMarkOne, shootGateIntake;


    @Override
    public void init() {
        headingPIDFController.PIDFController(heading_p, 0 ,heading_d, heading_f);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rhinoL = (DcMotorEx)  hardwareMap.dcMotor.get("rhinoL");
        rhinoR = (DcMotorEx) hardwareMap.dcMotor.get("rhinoR");

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rhinoL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rhinoR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rhinoL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rhinoR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        rhinoL.setDirection(DcMotorSimple.Direction.REVERSE);
        rhinoR.setDirection(DcMotorSimple.Direction.FORWARD);

        eat = (Servo) hardwareMap.servo.get("eat");
        eat.setPosition(0);
    }

    @Override
    public void init_loop() {

        // Modify the delay before the autonomous begins
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            isBlue = !isBlue;
        }

        if (isBlue) {
            location = "Blue";
        }
        else {
            location = "Red";
        }

        telemetry.addData("Delay in seconds", delaySeconds);
        telemetry.addData("Location", location);
        telemetry.update();

    }

    @Override
    public void start() {
        if (!isBlue) {
            startPose = startPose.mirror();
            shootPos = shootPos.mirror();
            intakeSpikeMarkTwo = intakeSpikeMarkTwo.mirror();
            gateIntake = gateIntake.mirror();
            intakeSpikeMarkOne = intakeSpikeMarkOne.mirror();

            CP_SpikeMarkTwo = CP_SpikeMarkTwo.mirror();
            CP_spikeMarkOne = CP_spikeMarkOne.mirror();
            CP_GateIntake = CP_GateIntake.mirror();
        }

        delayTimer.reset();
        autoTimer.reset();
        ballTimer.reset();

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        autonomousPathUpdate(); // Update autonomous state machine
        endPose = follower.getPose();
    }

    public void buildPaths() {
        launchPreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading()).build();

        getSpikeMarkTwo = follower.pathBuilder()
                .addPath(new BezierCurve(shootPos, CP_SpikeMarkTwo, intakeSpikeMarkTwo))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeSpikeMarkTwo.getHeading())
                .build();

        shootSpikeMarkTwo = follower.pathBuilder()
                .addPath(new BezierCurve(intakeSpikeMarkTwo, CP_GateIntake, shootPos))
                .setLinearHeadingInterpolation(intakeSpikeMarkOne.getHeading(), shootPos.getHeading())
                .build();

        collectGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(shootPos, CP_GateIntake, gateIntake))
                .setLinearHeadingInterpolation(shootPos.getHeading(), gateIntake.getHeading())
                .build();

        shootGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntake, CP_GateIntake, shootPos))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), shootPos.getHeading())
                .build();

        CollectSpikeMarkOne = follower.pathBuilder()
                .addPath(new BezierCurve(shootPos, CP_spikeMarkOne, intakeSpikeMarkOne))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeSpikeMarkOne.getHeading())
                .build();

        shootSpikeMarkOne = follower.pathBuilder()
                .addPath(new BezierCurve(intakeSpikeMarkOne, CP_spikeMarkOne, shootPos))
                .setLinearHeadingInterpolation(intakeSpikeMarkOne.getHeading(), shootPos.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    follower.followPath(launchPreload, true);
                    pathState = 1;
                }
                break;
            case 1: // 3
                if (!follower.isBusy()) {
                    PeregrinesConfig.launchBalls();
                    PeregrinesConfig.launchBalls();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() && !PeregrinesConfig.isLaunching) {
                    PeregrinesConfig.toggleIntake(); // on
                    follower.followPath(getSpikeMarkTwo);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    PeregrinesConfig.toggleIntake(); // off
                    follower.followPath(shootSpikeMarkTwo);
                    pathState = 4;
                }
                break;
            case 4: // 6
                if (!follower.isBusy()) {
                    PeregrinesConfig.launchBalls();
                    pathState = 5;
                }
                break;
            case 5:
                if (!PeregrinesConfig.isLaunching) {
                    PeregrinesConfig.toggleIntake(); // on
                    follower.followPath(collectGatePath);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    PeregrinesConfig.toggleIntake(); // off
                    follower.followPath(shootGateIntake);
                    pathState = 7;
                }
                break;
            case 7: // 9
                if (!follower.isBusy()) {
                    PeregrinesConfig.launchBalls();
                    pathState = 8;
                }
                break;
            case 8:
                if (!PeregrinesConfig.isLaunching) {
                    PeregrinesConfig.toggleIntake(); // on
                    follower.followPath(collectGatePath);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                   PeregrinesConfig.toggleIntake(); // off
                    follower.followPath(shootGateIntake);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    PeregrinesConfig.launchBalls(); // 12
                    pathState = 11;
                }
            break;
            case 11:
                if (!PeregrinesConfig.isLaunching) {
                    PeregrinesConfig.toggleIntake(); // on
                    follower.followPath(CollectSpikeMarkOne);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    PeregrinesConfig.toggleIntake(); // off
                    follower.followPath(shootSpikeMarkOne);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    PeregrinesConfig.launchBalls(); // 15
                    pathState = -1; // END AUTO ROUTE!!! WOOO 15 BALL AUTO?!?!?! (hopefully)
                }
                break;
        }
    }
}