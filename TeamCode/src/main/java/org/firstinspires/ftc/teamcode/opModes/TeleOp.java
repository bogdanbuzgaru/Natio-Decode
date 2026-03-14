package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Sensor;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private List<Double> results = new ArrayList<>();
    private enum State { E_BILE, NU_E_BILE }

    private Turret turret;
    private Shooter shooter;
    private Position pos;
    private Index index;
    private Intake intake;
    private Movement movement;
    private Sensor sensor;
    private RevColorSensorV3 colorSensor, colorSensor2;
    private Lift lift;
    private StateMachine<State> fsm = new StateMachine<>(State.NU_E_BILE);

    private boolean manual = false;
    private boolean park = true;
    private ElapsedTime leftBumperHoldTimer = new ElapsedTime();
    private boolean leftBumperWasPressed = false;
    private int angle = 0;
    private double tick;
    private ElapsedTime timer = new ElapsedTime();

    public static Follower follower;

    public void init() {
        // --- START FILE READING LOGIC ---
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        try {
            String[] vals = ReadWriteFile.readFile(file).split("\n");
            for (String val : vals) {
                results.add(Double.parseDouble(val));
            }
        } catch (Exception e) {
            // Fallback if file is missing
            results.add(0.0); results.add(0.0); results.add(0.0);
        }

        double startX = results.get(results.size() - 3);
        double startY = results.get(results.size() - 2);
        double startHeadingDeg = Math.toRadians(results.get(results.size() - 1));
        Pose startPose = new Pose(startX, startY, startHeadingDeg);
        // --- END FILE READING LOGIC ---

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);

        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);
        sensor = new Sensor(hardwareMap, "colorSensor");
        pos = new Position(startPose);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.enableLed(true);
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        colorSensor2.enableLed(true);

        setUp();
    }

    public void loop() {
        follower.update();
        pos.update(follower.getPose()); // Update math object with Follower's Pose
        fsm.update();

        increaseAngle();
        increaseDecrease();

        movement.movementLoop(gamepad1);
        intake.take(gamepad1);
        index.feed(gamepad1);
        pos.chooseAlliance(gamepad2);
        resetPosition(gamepad2);
        pos.whereToShoot(gamepad1);
        pos.setCoord(true);

        // Lift Toggle
        if (gamepad1.dpadDownWasPressed()) {
            if (park) lift.lift(); else lift.lower();
            park = !park;
        }

        // Manual Mode Toggle
        if (gamepad1.triangleWasPressed()) manual = !manual;

        // Auto Shooting Logic
        if ((isLeftBumperHeld(0.0006) && (pos.shootClose() || pos.shootHigh()) || isLeftBumperHeld(0.5))) {
            shooter.raiseBarrier();
            intake.autoTake();
            index.autoFeed();
        } else {
            shooter.lowerBarrier();
        }

        if (pos.activateOrientation() && !manual) turret.update();

        if (manual) {
            turret.goNeutral();
            if (gamepad1.leftBumperWasPressed()) shooter.raiseBarrier();
            else if (gamepad1.rightBumperWasPressed()) shooter.lowerBarrier();
        }

        // Cord logic using Follower Pose
        pos.setChangeCord(follower.getPose().getY() < 40);

        // Shooter & Turret Updates using Follower Velocity
        double velX = follower.getVelocity().getXComponent();
        double velY = follower.getVelocity().getYComponent();

        if (pos.isRed()) {
            shooter.setTicks(pos.getTicks(6.89911, 1100.04194));
            turret.setTargetAngle(pos.getTargetAngle() + angle);
            turret.setOffsetAngle(pos.offsetAngleRed(velX, velY, pos.getTicks(8.8057, 1098)));
        } else if (pos.isBlue()) {
            shooter.setTicks(pos.getTicksBlue(6.89911, 1100.04194));
            turret.setTargetAngle(pos.getTargetAngle() + angle);
            turret.setOffsetAngle(pos.getOffetAngle(velX, velY));
        }
        shooter.lowerHood(gamepad1);
        shooter.update();

        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Target Angle", pos.getTargetAngle());
        telemetry.addData("Shooter Velocity", shooter.getTicks());
        telemetry.addData("increased angle", angle);
        telemetry.update();
    }
    private void increaseAngle(){
        if(gamepad2.circleWasPressed()){
            angle += 1;
        }else if(gamepad2.squareWasPressed()) {
            angle -= 1;
        }else if (gamepad2.triangleWasPressed()){
            angle = 0;
        }
    }
    private void increaseDecrease(){
        if(gamepad2.right_trigger > 0.01){
            tick = 100;
        }else if (gamepad2.left_trigger > 0.01){
            tick = -120;
        }else{
            tick = 0;
        }
    }
    private void setUp() {
        fsm.onStateEnter(State.NU_E_BILE, () -> { timer.reset(); return null; });
        fsm.onStateUpdate(State.NU_E_BILE, () -> {
            if (timer.milliseconds() >= 450) {
                double distance = colorSensor.getDistance(DistanceUnit.CM);
                if (!Double.isNaN(distance) && distance < 5.2) {
                    gamepad1.rumble(200);
                    return State.E_BILE;
                }
                timer.reset();
            }
            return null;
        });
        fsm.onStateUpdate(State.E_BILE, () -> {
            if (timer.milliseconds() >= 300) {
                double distance = colorSensor.getDistance(DistanceUnit.CM);
                if (Double.isNaN(distance) || distance > 5.8) return State.NU_E_BILE;
                timer.reset();
            }
            return null;
        });
        fsm.init();
    }

    private boolean isLeftBumperHeld(double holdTimeSeconds) {
        if (gamepad1.left_bumper) {
            if (!leftBumperWasPressed) {
                leftBumperHoldTimer.reset();
                leftBumperWasPressed = true;
            }
            return leftBumperHoldTimer.seconds() >= holdTimeSeconds;
        }
        leftBumperWasPressed = false;
        return false;
    }

    private void resetPosition(Gamepad gamepad) {
        if(pos.isRed()) {
            if (gamepad.dpadUpWasPressed()) {
                follower.setPose(new Pose(
                        124.49, 84.91,        //Up
                        Math.toRadians(0)
                ));
                manual = false;
            } else if (gamepad.dpadLeftWasPressed()) {
                follower.setPose(new Pose(
                        116.214, 125.133,    //basket
                        Math.toRadians(38.4)
                ));
                manual = false;
            } else if (gamepad.dpadDownWasPressed()) {
                {
                    follower.setPose(new Pose(
                            123.33, 68.3,    //Up barrier
                            Math.toRadians(0)
                    ));
                    manual = false;
                }
                manual = false;
            } else if (gamepad.dpadRightWasPressed()) {
                follower.setPose(new Pose(
                        8.26, 9.08,    //our human player
                        Math.toRadians(270)
                ));
                manual = false;
            }
        }else if(pos.isBlue()){
            if (gamepad.dpadUpWasPressed()) {
                follower.setPose(new Pose(
                        19.51, 84.91,        //Up
                        Math.toRadians(180)
                ));
                manual = false;
            } else if (gamepad.dpadLeftWasPressed()) {
                follower.setPose(new Pose(
                        27.786, 125.133,    //basket
                        Math.toRadians(141.6)
                ));
                manual = false;
            } else if (gamepad.dpadDownWasPressed()) {
                {
                    follower.setPose(new Pose(
                            20.67, 68.3,    //Up barrier
                            Math.toRadians(180)
                    ));
                    manual = false;
                }
                manual = false;
            } else if (gamepad.dpadRightWasPressed()) {
                follower.setPose(new Pose(
                        135.74, 9.08,    //our human player
                        Math.toRadians(270)
                ));
                manual = false;
            }
        }
    }
}