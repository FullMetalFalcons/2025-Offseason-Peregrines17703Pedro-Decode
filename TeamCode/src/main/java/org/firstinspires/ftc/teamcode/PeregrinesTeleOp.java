package org.firstinspires.ftc.teamcode; // 67

import static com.pedropathing.math.MathFunctions.normalizeAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class PeregrinesTeleOp extends OpMode {

    DcMotorEx motorLF, motorRF, motorLB, motorRB, intake, rhinoL, rhinoR;
    Servo eat, stephen;
    TelemetryManager telemetryManager;
    HeadingPIDFController headingPIDFController = new HeadingPIDFController();
    GoBildaPinpointDriver pinpoint;

    boolean noahBeingStupid = false;
    ElapsedTime timer = new ElapsedTime();
    double yIntercept = 2121.36088;

    double headingFieldCentric;
    double headingRadians;
    double robotX;
    double robotY;
    boolean fieldCentricInUse = true;

    public final double CHRISTIANITY = 0.279;
    public final double LED_ORANGE = 0.333;
    public final double LED_YELLOW = 0.38;
    public final double DECLAN = 0.5;
    public final double LED_BLUE = 0.611;
    public final double LED_PURPLE = 0.7;

    public static PIDFCoefficients flywheelPIDF = new PIDFCoefficients(
        150,
        15,
        0,
        0.5
    ); // TYSM TREVOR ILYYY!!!
    public static double heading_p = 1,
        heading_d = 0.097,
        heading_f = 0.03;
    public static double flywheelVelocity = 2900,
        leftMult = 1.1;

    public static boolean blue = false;
    double targetBlueX = 10,
        targetBlueY = 140;
    double targetRedX = 140,
        targetRedY = 140;
    double targetCurrentX = 140,
        targetCurrentY = 140;

    double headingError;

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        initPinpoint();
        initDriveMotors(DcMotor.ZeroPowerBehavior.BRAKE);

        headingPIDFController.PIDFController(
            heading_p,
            0,
            heading_d,
            heading_f
        );

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rhinoL = (DcMotorEx) hardwareMap.dcMotor.get("rhinoL");
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
        stephen = (Servo) hardwareMap.servo.get("goon");

        if (PeregrinesPos.pos == 0 || PeregrinesPos.pos == 2) {
            blue = true;
        }
        if (PeregrinesPos.pos == 1 || PeregrinesPos.pos == 3) {
            blue = false;
        }
    }

    @Override
    public void loop() {
        // *************    ODOMETRY    *************
        pinpoint.update();
        if (blue) {
            headingFieldCentric = -pinpoint.getHeading(AngleUnit.RADIANS) + Math.PI;
        }
        if (!blue) {
            headingFieldCentric = pinpoint.getHeading(AngleUnit.RADIANS);
        }

        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);
        double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);

        // *************    TARGET LOGIC    *************

        /*if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            correctedTargetToggle = !correctedTargetToggle;
        }*/

        /*if (PeregrinesPos.pos == 0 || PeregrinesPos.pos == 2) {
            targetCurrentX = targetBlueX;
            targetCurrentY = targetBlueY;
        } else {
            targetCurrentX = targetRedX;
            targetCurrentY = targetRedY;
        }*/

        // ....... MECANUM DRIVE CONTROLS .......
        // Set the desired powers based on joystick inputs (or dpad, for slow mode)
        double desiredForward = -gamepad1.left_stick_y;
        double desiredStrafe = gamepad1.left_stick_x;

        if (blue) {
            headingFieldCentric = pinpoint.getHeading(AngleUnit.RADIANS) + Math.PI;
        } else {
            headingFieldCentric = pinpoint.getHeading(AngleUnit.RADIANS);
        }

        double powerAngular = -gamepad1.right_stick_x;
        double powerForward = desiredForward;  // Assume field-centric is not being used
        double powerStrafe = desiredStrafe;    // Assume field-centric is not being used

        // FIELD CENTRIC MODIFICATIONS AND TOGGLE CONTROL
        // Modify powers based on robot heading for field-centric drive
        if (fieldCentricInUse) {
            powerForward = (desiredForward * Math.cos(headingFieldCentric)) - (desiredStrafe * Math.sin(headingFieldCentric));
            powerStrafe = (desiredStrafe * Math.cos(headingFieldCentric)) + (desiredForward * Math.sin(headingFieldCentric));
        }
        if (!fieldCentricInUse) {
            powerStrafe = gamepad1.left_stick_x;
            powerForward = -gamepad1.left_stick_y;
        }
        // Be able to toggle field centric drive in case the Pinpoint fails somehow
        if (gamepad1.backWasPressed()) {
            fieldCentricInUse = !fieldCentricInUse;
        }


        if (gamepad2.dpadDownWasPressed()) {
            blue = !blue;
        }
        if (blue) {
            targetCurrentX = targetBlueX;
            targetCurrentY = targetBlueY;
        } else {
            targetCurrentX = targetRedX;
            targetCurrentY = targetRedY;
        }

        double distance = calculateDistance(
            currentX,
            currentY,
            targetCurrentX,
            targetCurrentY
        );

        //    *************    SHOOT AND MOVE    *************
        /*double[] targetCurrentAdjusted = getAdjustedTarget(
                targetCurrentX, targetCurrentY,
                velX, velY,
                timeOfFlight = calculateTimeOfFlight(1.7142857143 * targetVel, distance)
        );
        if (correctedTargetToggle && currentY > 42) {
            targetCurrentX = targetCurrentAdjusted[0];
            targetCurrentY = targetCurrentAdjusted[1];
            distance = calculateDistance(currentX, currentY, targetCurrentX, targetCurrentY);
        }*/

        double targetHeading = Math.atan2(
            targetCurrentY - currentY,
            targetCurrentX - currentX
        );

        // *************    MECANUM    *************
        double powerX = 0.0; // Desired power for strafing           (-1 to 1)
        double powerY = 0.0; // Desired power for forward/backward   (-1 to 1)
        double powerAng = 0.0; // Desired power for turning          (-1 to 1)

        boolean targetTrack;

        headingPIDFController.PIDFController(
            heading_p,
            0,
            heading_d,
            heading_f
        );

        headingError = determineRotationDirection(
            pinpoint.getHeading(AngleUnit.RADIANS),
            targetHeading
        );

        if (gamepad2.right_trigger > 0.2) {
            powerAngular = headingPIDFController.run(headingError);
            powerAngular = Math.max(-1.0, Math.min(1.0, powerAngular));
            targetTrack = true;
        } else {
            powerAngular = -gamepad1.right_stick_x;
            targetTrack = false;
        }


        if (!noahBeingStupid) {
            mecanumDriveCode(powerForward, powerStrafe, powerAngular, 1.0);
        }

        // *************    LAUNCHER    *************
        double velDifference = Math.abs(
            rhinoL.getVelocity() - rhinoR.getVelocity()
        );

            if ((gamepad1.right_bumper && !noahBeingStupid) || gamepad2.right_bumper) {
                intake.setPower(-1);
            } else if (((gamepad1.left_bumper && !noahBeingStupid) || gamepad2.left_bumper)) {
                //&& (((gamepad1.right_trigger >= 0.2 || gamepad2.right_trigger >= 0.2) && velDifference <= 20) || (gamepad1.right_trigger <= 0.2 || gamepad2.right_trigger <= 0.2))) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

        // FLYWHEEL CODE
        rhinoL.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            flywheelPIDF
        );
        rhinoR.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            flywheelPIDF
        );

        if (PeregrinesPos.isSolo) {
            if (gamepad1.right_trigger >= 0.2) {
                flywheelVelocity = (14.70469 * distance) + yIntercept;
                rhinoL.setVelocity(flywheelVelocity);
                rhinoR.setVelocity(flywheelVelocity - 63);
                eat.setPosition(0.4);
            } else if (gamepad1.left_trigger >= 0.2) {
                rhinoL.setVelocity(-flywheelVelocity);
                rhinoR.setVelocity(-flywheelVelocity);
            } else {
                rhinoL.setPower(0);
                rhinoR.setPower(0);
                eat.setPosition(0);
            }
        }

        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadLeftWasPressed()) {
            yIntercept += 100;
        }
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadRightWasPressed()) {
            yIntercept -= 100;
        }

        if (noahBeingStupid) {
            powerForward = 0;
            powerStrafe = 0;
            powerAngular = 0;
        }

        if (gamepad2.right_trigger >= 0.2) {
            flywheelVelocity = (14.70469 * distance) + yIntercept;
            rhinoL.setVelocity(flywheelVelocity);
            rhinoR.setVelocity(flywheelVelocity - 75);
            eat.setPosition(0.4);
        } else if (gamepad2.left_trigger >= 0.2) {
            rhinoL.setVelocity(-flywheelVelocity);
            rhinoR.setVelocity(-flywheelVelocity);
        } else {
            rhinoL.setPower(0);
            rhinoR.setPower(0);
            eat.setPosition(0);
        }
        /*
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            eat.setPosition(0.5);
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            eat.setPosition(0);
        }*/

        if (gamepad2.backWasPressed()) {
            noahBeingStupid = !noahBeingStupid;
        }
        if (timer.seconds() <= 25) {
            if (!blue) {
                if (
                    (robotX > 30 && robotX < 40) && (robotY > 30 && robotY < 40)
                ) {
                    stephen.setPosition(LED_PURPLE);
                }
                if ((robotX > 77) && (robotY < 80)) {
                    stephen.setPosition(LED_ORANGE);
                }
            }
            if (blue) {
                if (
                    (robotX > 100 && robotX < 110) &&
                    (robotY > 30 && robotY < 40)
                ) {
                    stephen.setPosition(LED_PURPLE);
                }
                if ((robotX < 77) && (robotY < 80)) {
                    stephen.setPosition(LED_ORANGE);
                }
            }
        }
        if (distance <= 79 && distance >= 40) {
            stephen.setPosition(DECLAN);
        } else if (blue) {
            stephen.setPosition(LED_BLUE);
        } else if (!blue) {
            stephen.setPosition(CHRISTIANITY);
        } else {
            stephen.setPosition(LED_PURPLE);
        }

        telemetryManager.addData("launchVel L", rhinoL.getVelocity());
        telemetryManager.addData("launchVel R", rhinoR.getVelocity());

        telemetryManager.addData(
            "launchError L",
            flywheelVelocity - rhinoL.getVelocity()
        );
        telemetryManager.addData(
            "launchError R",
            flywheelVelocity - rhinoR.getVelocity()
        );

        telemetryManager.addData("targetVel", flywheelVelocity);

        telemetryManager.update();

        telemetry.addData("velocity R", rhinoR.getVelocity());
        telemetry.addData("velocity L", rhinoL.getVelocity());
        telemetry.addData("velocity difference", velDifference);
        telemetry.addData("distance", distance);
        telemetry.addLine();
        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("H", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("targetHeading", Math.toDegrees(targetHeading));
        telemetry.addData("targetX", targetCurrentX);
        telemetry.addData("targetY", targetCurrentY);
        telemetry.addData("Y INTERCEPT", yIntercept);
        telemetry.addData("Target Velocity", (14.70469 * distance) + yIntercept);
        //telemetry.addData("in close launch", launchDetection());
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    // Any additional methods go here
    public void initDriveMotors(DcMotor.ZeroPowerBehavior behavior) {
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get(
            Constants.driveConstants.leftFrontMotorName
        );
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get(
            Constants.driveConstants.rightFrontMotorName
        );
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get(
            Constants.driveConstants.leftRearMotorName
        );
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get(
            Constants.driveConstants.rightRearMotorName
        );

        motorLF.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        motorLB.setDirection(Constants.driveConstants.leftRearMotorDirection);
        motorRF.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        motorRB.setDirection(Constants.driveConstants.rightRearMotorDirection);

        motorLF.setZeroPowerBehavior(behavior);
        motorLB.setZeroPowerBehavior(behavior);
        motorRF.setZeroPowerBehavior(behavior);
        motorRB.setZeroPowerBehavior(behavior);
    }

    public void initPinpoint() {
        String pinpointName = Constants.localizerConstants.hardwareMapName;
        GoBildaPinpointDriver.EncoderDirection forwardDirection =
            Constants.localizerConstants.forwardEncoderDirection;
        GoBildaPinpointDriver.EncoderDirection strafeDirection =
            Constants.localizerConstants.strafeEncoderDirection;
        GoBildaPinpointDriver.GoBildaOdometryPods resolution =
            Constants.localizerConstants.encoderResolution;

        double xOffset = Constants.localizerConstants.forwardPodY;
        double yOffset = Constants.localizerConstants.strafePodX;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.setEncoderDirections(forwardDirection, strafeDirection);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(resolution);

        pinpoint.setPosition(
            PoseConverter.poseToPose2D(
                PeregrinesAuto.endPose,
                PedroCoordinates.INSTANCE
            )
        );
    }

    public void mecanumDriveCode(double forward, double strafe, double angular, double speedPercent) {
        // Perform vector math to determine the desired powers for each wheel
        double powerLF = strafe + forward - angular;
        double powerLB = -strafe + forward - angular;
        double powerRF = -strafe + forward + angular;
        double powerRB = strafe + forward + angular;

        // Determine the greatest wheel power and set it to max
        double max = Math.max(1.0, Math.abs(powerLF));
        max = Math.max(max, Math.abs(powerRF));
        max = Math.max(max, Math.abs(powerLB));
        max = Math.max(max, Math.abs(powerRB));

        // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
        motorLF.setPower(powerLF /max * speedPercent);
        motorLB.setPower(powerLB /max * speedPercent);
        motorRF.setPower(powerRF /max * speedPercent);
        motorRB.setPower(powerRB /max * speedPercent);
    }

    public double determineRotationDirection(double current, double target) {
        double currentCircularHeading = normalizeAngle(current);
        double targetHeading = normalizeAngle(target);
        double clockwiseRadians;
        double counterclockwiseRadians;

        // Determine the larger of the two headings
        if (targetHeading > currentCircularHeading) {
            // Subtract the smaller (current) heading from the larger (target) heading
            //   to find the radians needed to turn to get to the target going counterclockwise
            counterclockwiseRadians = targetHeading - currentCircularHeading;
            // Find the alternative
            clockwiseRadians = 2 * Math.PI - counterclockwiseRadians;
        } else {
            // Subtract the smaller (target) heading from the larger (current) heading
            //   to find the radians needed to turn to get to the target doing clockwise
            clockwiseRadians = currentCircularHeading - targetHeading;
            // Find the alternative
            counterclockwiseRadians = 2 * Math.PI - clockwiseRadians;
        }
        // Determine the most efficient direction and return the proper multiplier
        if (clockwiseRadians < counterclockwiseRadians) {
            return -clockwiseRadians;
        } else {
            return counterclockwiseRadians;
        }
    }

    public static double calculateDistance(
        double x1,
        double y1,
        double x2,
        double y2
    ) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
}

/*

    public boolean launchDetection()
    {
        double pinX = pinpoint.getPosX(DistanceUnit.INCH);
        double pinY = pinpoint.getPosY(DistanceUnit.INCH);

        if (pinX >= 70) {
            double minY = pinX;
            if (pinY >= minY) {
                return true;
            } else {
                return false;
            }
        } else {
            double minY = -pinX + 140;
            if (pinY >= minY) {
                return true;
            } else {
                return false;
            }
        }
    }*/
