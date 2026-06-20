package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
    private final List<Double> results = new ArrayList<>();
    private enum State { E_BILE, NU_E_BILE, BOMB }

    private Turret turret;
    private Shooter shooter;
    private Position pos;
    private Index index;
    private Intake intake;
    private Movement movement;
    private Rev2mDistanceSensor distanceSensor, distanceSensor2, distanceSensor3;
    private Lift lift;
    private final StateMachine<State> fsm = new StateMachine<>(State.NU_E_BILE);
    private ServoImplEx rgbLed;
    private boolean fieldCentric = false;
    private boolean manual = false;
    private boolean park = true;
    private int angle = 0;
    private double tick;
    private ElapsedTime timer = new ElapsedTime();
    public static Follower follower;
    private int offset = 0;
    private ElapsedTime bombTimer = new ElapsedTime();
    private Servo indexMove;

    public void init() {
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        try {
            String[] vals = ReadWriteFile.readFile(file).split("\n");
            for (String val : vals) {
                results.add(Double.parseDouble(val));
            }
        } catch (Exception e) {
            results.add(0.0); results.add(0.0); results.add(0.0);
        }
        double startX = results.get(results.size() - 3);
        double startY = results.get(results.size() - 2);
        double startHeadingDeg = Math.toRadians(results.get(results.size() - 1));
        Pose startPose = new Pose(startX, startY, startHeadingDeg);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        rgbLed = hardwareMap.get(ServoImplEx.class, "RGB");
        rgbLed.setPwmRange(new PwmControl.PwmRange(500, 2500));

        indexMove = hardwareMap.get(Servo.class, "indexMove");
        indexMove.setPosition(0.47);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);
        pos = new Position(startPose);
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor2");
        distanceSensor3 = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor3");
        shooter.lowerHood();
        shooter.middleBar();
//        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
//        colorSensor2.enableLed(true);

        setUp();
    }
    public void start(){
        bombTimer.reset();
    }

    public void loop() {
        follower.update();
        pos.update(follower.getPose());
        fsm.update();
//        turret.setRed(pos.isRed());
        increaseDecrease();
        if(!fieldCentric)
            movement.movementLoop(gamepad1);
        else
            movement.movementFieldCentric(gamepad1, follower.getPose().getHeading(), pos.isRed());

        if(gamepad1.triangleWasPressed()){
            fieldCentric = !fieldCentric;
        }
        hasBalls();
        intake.take(gamepad1);
        index.feed(gamepad1);
        pos.chooseAlliance(gamepad2);
        resetPosition(gamepad1);        //TODO Make it Gamepad2
//        pos.whereToShoot(gamepad2);
        shooter.changeCoef(pos.isFar());
        // Lift Toggle
        if (gamepad1.dpadDownWasPressed()) {
            if (park) lift.lift(); else lift.lower();
            park = !park;
        }

        if (gamepad2.triangleWasPressed())
            manual = !manual;

        if (gamepad1.left_bumper) {
            shooter.raiseBarrier();
            intake.autoTake();
            index.autoFeed();

        } else {
            shooter.lowerBarrier();
        }

        if (pos.activateOrientation() && !manual)
            turret.update();
        if (manual) {
            turret.goNeutral();
            if (gamepad1.leftBumperWasPressed()) shooter.raiseBarrier();
            else if (gamepad1.rightBumperWasPressed()) shooter.lowerBarrier();
        }
        double addOn = 130;
        // Shooter & Turret Updates using Follower Velocity
        double velX = follower.getVelocity().getXComponent();
        double velY = follower.getVelocity().getYComponent();
        boolean add = false;
        if(gamepad1.dpadRightWasPressed()){
            add = !add;
        }
        if (pos.isRed() && !add) {
            shooter.setTicks(pos.getTicks(10.0037095, 1145.560813));
            turret.setTargetAngle(pos.target()); //+ angle if needed
            turret.setHeading(Math.toDegrees(follower.getHeading()), true);
        } else if (pos.isBlue() && !add) {
            shooter.setTicks(pos.getTicksBlue(10.0037095, 1145.560813) + 120);
            turret.setTargetAngle(pos.target());//+ angle if needed
            turret.setHeading(Math.toDegrees(follower.getHeading()), false);
        }
        if (pos.isRed() && add) {
            shooter.setTicks(pos.getTicks(10.0037095, 1145.560813) + addOn);
            turret.setTargetAngle(pos.target()); //+ angle if needed
            turret.setHeading(Math.toDegrees(follower.getHeading()), true);
        } else if (pos.isBlue() && add) {
            shooter.setTicks(pos.getTicksBlue(10.0037095, 1145.560813) + addOn + 120);
            turret.setTargetAngle(pos.target());//+ angle if needed
            turret.setHeading(Math.toDegrees(follower.getHeading()), false);
        }
        shooter.update();

        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Target Angle", pos.getTargetAngle());
        telemetry.addData("Shooter Velocity", shooter.getTicks());
        telemetry.addData("increased angle", angle);
        telemetry.addData("First servo position", turret.getPosition1());
        telemetry.addData("Second servo position", turret.getPosition2());
        telemetry.addData("Third servo position", turret.getPosition3());

        telemetry.update();
    }
    private void hasBalls(){
        if(distanceSensor.getDistance(DistanceUnit.CM) < 6 || distanceSensor2.getDistance(DistanceUnit.CM) < 7){
            index.setLower(true);
        }else{
            index.setLower(false);
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
            indexMove.setPosition(0.47);
            if(bombTimer.seconds() < 110)
                rgbLed.setPosition(0.7);
//            else{
//                return State.BOMB;
//            }
            if (timer.milliseconds() >= 450) {
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                double distance2 = distanceSensor2.getDistance(DistanceUnit.CM);
                double distance3 = distanceSensor3.getDistance(DistanceUnit.CM);
                if (!(Double.isNaN(distance) && Double.isNaN(distance2) && Double.isNaN(distance3))
                        && (distance < 6 && distance2 < 6 && distance3 < 15))
                {
                    gamepad1.rumble(200);
                    return State.E_BILE;
                }
                timer.reset();
            }
            return null;
        });
        fsm.onStateUpdate(State.E_BILE, () -> {
            indexMove.setPosition(0.2);
            if(bombTimer.seconds() < 110)
                rgbLed.setPosition(0.5);
//            else{
//                return State.BOMB;
//            }
            if (timer.milliseconds() >= 300) {
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                double distance2 = distanceSensor2.getDistance(DistanceUnit.CM);
                double distance3 = distanceSensor3.getDistance(DistanceUnit.CM);
                if (gamepad1.leftBumperWasPressed())
                    return State.NU_E_BILE;
                timer.reset();
            }
            return null;
        });
        fsm.onStateUpdate(State.BOMB, () ->{
            if(bombTimer.seconds() < 115){
                rgbLed.setPosition(0.7);
            }else if (blink(bombTimer.milliseconds())){
                rgbLed.setPosition(0.7);
            }
            return null;
        });
        fsm.init();
    }
    private boolean blink(double currTime){
        return (currTime % 400) < 200;
    }



    private void resetPosition(Gamepad gamepad) {
        if(pos.isRed()) {
            if (gamepad.dpadUpWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        53.3397, 12.2725,        //Far shooting
                        Math.toRadians(-90)
                ));
                manual = false;
            } else if (gamepad.dpadLeftWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        116.214, 125.133,    //basket
                        Math.toRadians(38.4)
                ));
                manual = false;}
//            } else if (gamepad.dpadDownWasPressed()) {
//                movement.resetHeading();
//                movement.setOff(0);
//                {
//                    follower.setPose(new Pose(
//                            122.332, 79.3,    //Near gate
//                            Math.toRadians(0)
//                    ));
//                    manual = false;
//                }
//                manual = false;
//            }// else if (gamepad.dpadRightWasPressed()) {
//                movement.resetHeading();
//                movement.setOff(0);
//                follower.setPose(new Pose(
//                        10.8364, 10.7869,    //our human player w/ ext
//                        Math.toRadians(180)
//                ));
//                manual = false;
//            }
        }else if(pos.isBlue()){
            if (gamepad.dpadUpWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        90.6603, 12.2725,        //Far shooting
                        Math.toRadians(-90)
                ));
                manual = false;
            } else if (gamepad.dpadLeftWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        27.786, 125.133,    //basket
                        Math.toRadians(141.6)
                ));
                manual = false;}
//            } else if (gamepad.dpadDownWasPressed()) {
//                movement.resetHeading();
//                movement.setOff(0);
//                {
//                    follower.setPose(new Pose(
//                            21.668, 79.3,    //Near gate
//                            Math.toRadians(180)
//                    ));
//                    manual = false;
//                }
//                manual = false;}
//            } else if (gamepad.dpadRightWasPressed()) {
//                movement.resetHeading();
//                movement.setOff(0);
//                follower.setPose(new Pose(
//                        133.1636, 10.7869,    //our human player w/ ext
//                        Math.toRadians(0)
//                ));
//                manual = false;
//            }
        }
    }
}