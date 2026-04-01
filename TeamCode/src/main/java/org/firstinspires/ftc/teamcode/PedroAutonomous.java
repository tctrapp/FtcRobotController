package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    // ── States ────────────────────────────────────────────────────────────────
    private static final int STATE_ELEVATE      = 0; // Raise servos before moving
    private static final int STATE_PATH1        = 1; // Drive forward (start → waypoint)
    private static final int STATE_PATH2        = 2; // Strafe/translate left
    private static final int STATE_PATH3        = 3; // Return to starting triangle
    private static final int STATE_AT_START     = 4; // Action fired on every return
    private static final int STATE_DONE         = 5; // Finished all loops

    // How many full loop cycles to run before stopping
    private static final int MAX_CYCLES = 3;

    // How long (ms) to hold the servo action at the start position
    private static final long ACTION_HOLD_MS = 750;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private Servo leftServo;
    private Servo rightServo;

    // ── Pedro Pathing ─────────────────────────────────────────────────────────
    public Follower follower;
    private Paths paths;

    // ── State tracking ────────────────────────────────────────────────────────
    private int pathState;
    private int cycleCount;
    private ElapsedTime actionTimer;

    // ── Telemetry ─────────────────────────────────────────────────────────────
    private TelemetryManager panelsTelemetry;

    // ── Poses ─────────────────────────────────────────────────────────────────
    private static final Pose START_POSE    = new Pose(56.000,  8.000, Math.toRadians(90));
    private static final Pose WAYPOINT_POSE = new Pose(56.000, 36.000, Math.toRadians(180));
    private static final Pose COLLECT_POSE  = new Pose(18.779, 35.541, Math.toRadians(180));
    private static final Pose RETURN_POSE   = new Pose(55.939,  7.851, Math.toRadians(120));

    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Servos — update hardware map names to match your robot config
        leftServo  = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        paths = new Paths(follower);

        actionTimer = new ElapsedTime();
        pathState   = STATE_ELEVATE;
        cycleCount  = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State",  stateName(pathState));
        panelsTelemetry.debug("Cycle",       cycleCount + " / " + MAX_CYCLES);
        panelsTelemetry.debug("X",           follower.getPose().getX());
        panelsTelemetry.debug("Y",           follower.getPose().getY());
        panelsTelemetry.debug("Heading",     Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    // ── State Machine ─────────────────────────────────────────────────────────

    private void autonomousPathUpdate() {
        switch (pathState) {

            // ── ELEVATE: raise servos to 0.6 before the robot moves ──────────
            case STATE_ELEVATE:
                leftServo.setPosition(0.6);
                rightServo.setPosition(0.6);
                pathState = STATE_PATH1;
                follower.followPath(paths.path1, true);
                break;

            // ── PATH 1: drive forward toward the field waypoint ───────────────
            case STATE_PATH1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path2, true);
                    pathState = STATE_PATH2;
                }
                break;

            // ── PATH 2: translate across field toward the collect zone ─────────
            case STATE_PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.path3, true);
                    pathState = STATE_PATH3;
                }
                break;

            // ── PATH 3: return to the starting triangle ───────────────────────
            case STATE_PATH3:
                if (!follower.isBusy()) {
                    cycleCount++;
                    actionTimer.reset();
                    pathState = STATE_AT_START;
                    fireStartAction();          // always fires on every return
                }
                break;

            // ── AT START: run the action, then decide whether to loop ─────────
            case STATE_AT_START:
                // Hold position and keep servos active during the action window
                if (actionTimer.milliseconds() >= ACTION_HOLD_MS) {
                    finishStartAction();        // retract / reset after action
                    if (cycleCount < MAX_CYCLES) {
                        // Re-elevate servos and repeat the route
                        leftServo.setPosition(0.6);
                        rightServo.setPosition(0.6);
                        follower.followPath(paths.path1, true);
                        pathState = STATE_PATH1;
                    } else {
                        pathState = STATE_DONE;
                    }
                }
                break;

            case STATE_DONE:
                // Autonomous complete — servos can idle at 0.0
                leftServo.setPosition(0.0);
                rightServo.setPosition(0.0);
                break;
        }
    }

    /**
     * Called every time the robot arrives back at the starting triangle.
     * Add any scoring, depositing, or mechanism action here.
     */
    private void fireStartAction() {
        // Example: lower servos to deposit / release
        leftServo.setPosition(0.2);
        rightServo.setPosition(0.2);
    }

    /** Resets hardware after the start-position action window expires. */
    private void finishStartAction() {
        // Return to neutral before re-elevating on the next cycle
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

    // ── Path Definitions ──────────────────────────────────────────────────────

    public static class Paths {
        public final PathChain path1;
        public final PathChain path2;
        public final PathChain path3;

        public Paths(Follower follower) {

            // Straight forward from start toward the field center
            path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(START_POSE.getX(),    START_POSE.getY()),
                            new Pose(WAYPOINT_POSE.getX(), WAYPOINT_POSE.getY())
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            // Lateral move from waypoint to the collection zone
            path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(WAYPOINT_POSE.getX(), WAYPOINT_POSE.getY()),
                            new Pose(COLLECT_POSE.getX(),  COLLECT_POSE.getY())
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // Return from collection zone back to the starting triangle
            path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_POSE.getX(), COLLECT_POSE.getY()),
                            new Pose(RETURN_POSE.getX(),  RETURN_POSE.getY())
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private static String stateName(int state) {
        switch (state) {
            case STATE_ELEVATE:  return "ELEVATE";
            case STATE_PATH1:    return "PATH1";
            case STATE_PATH2:    return "PATH2";
            case STATE_PATH3:    return "PATH3";
            case STATE_AT_START: return "AT_START";
            case STATE_DONE:     return "DONE";
            default:             return "UNKNOWN";
        }
    }
}
