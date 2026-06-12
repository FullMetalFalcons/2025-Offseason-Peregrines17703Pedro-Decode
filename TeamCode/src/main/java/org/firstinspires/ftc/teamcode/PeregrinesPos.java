package org.firstinspires.ftc.teamcode; //67

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Configuration")
@Configurable
public class PeregrinesPos extends OpMode {

    TelemetryManager telemetryManager;

    public static Pose closeStart = new Pose(26, 129, Math.toRadians(143));
    public static Pose farStart = new Pose(56, 8, Math.toRadians(90));
    public static Pose startPos = new Pose();
    public static int pos = 0;
    public static String[] positions = {"Blue Close", "Red Close", "Blue Far", "Red Far"};
    public static boolean isSolo = false;

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Position", positions[pos]);
        telemetry.addData("Solo? ", isSolo);
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            pos += 1;
            if (pos > 3) {
                pos = 0;
            }
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            isSolo = !isSolo;
        }

        telemetry.update();


    }

    @Override
    public void start() {
        if (pos == 0) {
            startPos = closeStart;
        }
        if (pos == 1) {
            startPos = closeStart.mirror();
        }
        if (pos == 2) {
            startPos = farStart;
        }
        if (pos == 3) {
            startPos = farStart.mirror();
        }
    }


    @Override
    public void loop() {
        telemetry.addData("Selected start position: ", positions[pos]);
    }

    }
