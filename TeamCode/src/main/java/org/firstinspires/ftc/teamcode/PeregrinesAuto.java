package org.firstinspires.ftc.teamcode; //67

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "CloseAuto", group = "Auto")
public class PeregrinesAuto extends OpMode {

    public Follower follower;
    private int pathState;
    HeadingPIDFController headingPIDFController = new HeadingPIDFController();
    GoBildaPinpointDriver pinpoint;

    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();
    ElapsedTime ballTimer = new ElapsedTime();
    ElapsedTime GateTimer = new ElapsedTime();
    double delaySeconds = 0.0;
    public static Pose endPose = new Pose (72,72,0); //PeregrinesPos.startPos;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;

    public boolean isBlue = true;
    private String location = "blue";

    DcMotorEx intake, rhinoL, rhinoR;
    Servo eat;

    public static PIDFCoefficients flywheelPIDF = new PIDFCoefficients(70,0,0,0.35);
    public static double heading_p = 0, heading_d = 0, heading_f = 0;
    public static double flywheelVelocity = 2150;


    // Define important coordinate locations for the Blue side of the field

    // === Positions ===
    private Pose startPose = PeregrinesPos.startPos;
    private Pose shootPos = new Pose(45, 114, Math.toRadians(145));
    private Pose intakeSpikeMarkTwo = new Pose(15, 60, Math.toRadians(180));
    private Pose gateIntake = new Pose(12.55, 59.25, Math.toRadians(143.07));
    private Pose intakeSpikeMarkOne = new Pose(25, 84, Math.toRadians(180));

    // === CP ===
    private Pose CP_SpikeMarkTwo = new Pose(67, 55);
    private Pose CP_GateIntake = new Pose(67, 73);
    private Pose CP_spikeMarkOne = new Pose(67, 80);

    // === PARKING ===
    private Pose farPark = new Pose(16, 8, Math.toRadians(90));
    private Pose parkClose = new Pose(27, 91, 145);



    private PathChain launchPreload, getSpikeMarkTwo, shootSpikeMarkTwo, collectGatePath, CollectSpikeMarkOne, shootSpikeMarkOne, shootGateIntake, farParkPath, pathofpark;


    @Override
    public void init() {

        String pinpointName = Constants.localizerConstants.hardwareMapName;
        GoBildaPinpointDriver.EncoderDirection forwardDirection = Constants.localizerConstants.forwardEncoderDirection;
        GoBildaPinpointDriver.EncoderDirection strafeDirection = Constants.localizerConstants.strafeEncoderDirection;
        GoBildaPinpointDriver.GoBildaOdometryPods resolution = Constants.localizerConstants.encoderResolution;

        headingPIDFController.PIDFController(heading_p, 0 ,heading_d, heading_f);

        PeregrinesConfig.init(hardwareMap);

        if (PeregrinesPos.pos == 1 || PeregrinesPos.pos == 3) {
            endPose = endPose.mirror();

            shootPos = shootPos.mirror();
            intakeSpikeMarkTwo = intakeSpikeMarkTwo.mirror();
            gateIntake = new Pose(132.44, 61.76, Math.toRadians(34.9));
            intakeSpikeMarkOne = intakeSpikeMarkOne.mirror();

            CP_SpikeMarkTwo = CP_SpikeMarkTwo.mirror();
            CP_spikeMarkOne = CP_spikeMarkOne.mirror();
            CP_GateIntake = CP_GateIntake.mirror();
            parkClose = parkClose.mirror();
        }


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {

        telemetry.addData("Delay in seconds", delaySeconds);
        telemetry.addData("Location", PeregrinesPos.positions[PeregrinesPos.pos]);

        telemetry.update();

    }

    @Override
    public void start() {
        delayTimer.reset();
        autoTimer.reset();
        ballTimer.reset();

       /* if (PeregrinesPos.pos == 1) {
            flywheelVelocity = (14.70469 * 48.5677) + 1641;
        }
        if (PeregrinesPos.pos == 0) {
            flywheelVelocity = 2200;
        }
        */

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing - will also cause the robot to follow the current path
        PeregrinesConfig.update();
        endPose = follower.getPose();

        if (PeregrinesPos.pos == 0 || PeregrinesPos.pos == 1) {
            autonomousPathUpdateClose(); // Update autonomous state machine
        }
        if (PeregrinesPos.pos == 2 || PeregrinesPos.pos == 3) {
            autonomousPathUpdateFar(); // Update auto state machine
        }

        telemetry.addData("PathState", pathState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Is Intaking?", PeregrinesConfig.isIntaking);
        telemetry.update();
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

        farParkPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, farPark))
                .setLinearHeadingInterpolation(startPose.getHeading(), farPark.getHeading())
                .build();
        pathofpark = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, parkClose))
                .setConstantHeadingInterpolation(145)
                .build();
    }


    public void autonomousPathUpdateClose() {

        // Autonomous state machine
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    PeregrinesConfig.spinUpFlyWheels();
                    follower.followPath(launchPreload, true);
                    pathState = 1;
                }
                break;
            case 1: // 3
                if (!follower.isBusy()) {
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
                    follower.followPath(shootSpikeMarkTwo);
                    PeregrinesConfig.toggleIntake(); // off
                    PeregrinesConfig.spinUpFlyWheels();
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
                    GateTimer.reset();
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (GateTimer.seconds() >= 3.8) {
                        PeregrinesConfig.toggleIntake(); // off
                        PeregrinesConfig.spinUpFlyWheels();
                        follower.followPath(shootGateIntake);
                        pathState = 7;
                    }
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
                    GateTimer.reset();
                        pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    if (GateTimer.seconds() >= 3.8) {
                        PeregrinesConfig.toggleIntake(); // off
                        PeregrinesConfig.spinUpFlyWheels();
                        follower.followPath(shootGateIntake);
                        pathState = 10;
                    }
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
                        PeregrinesConfig.spinUpFlyWheels();
                        follower.followPath(shootSpikeMarkOne);
                        pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    PeregrinesConfig.launchBalls(); // 15
                    pathState = 14; // END AUTO ROUTE!!! WOOO 15 BALL AUTO?!?!?! (hopefully)
                }
                break;
            case 14:
                if (!PeregrinesConfig.isLaunching) {
                    follower.followPath(pathofpark);
                    pathState = -1;
                }
        }
    }

    public void autonomousPathUpdateFar() {
        switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    follower.followPath(farParkPath, true);
                    pathState = 1;
                }
                break;
        }
    }
}