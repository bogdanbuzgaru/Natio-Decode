package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
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
    private boolean fieldCentric = true;
    private boolean manual = false;
    private boolean park = true;
    private int angle = 0;
    private double tick;
    private double redBase = 790.04194;
    private ElapsedTime timer = new ElapsedTime();
    public static Follower follower;
    private int offset = 0;
    private ElapsedTime bombTimer = new ElapsedTime();
    private double isRed;
    private boolean lowerSpeed = false;
    private boolean parking = false;
    private ElapsedTime checkArt = new ElapsedTime();
    private PathChain autoPark;
    public double distance, distance2, distance3;
    public boolean useDistSens = true;
    private ElapsedTime toPark = new ElapsedTime();
    private boolean stopRumbling = true;
    private boolean stopTurret = false;
    public void init() {
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        try {
            String[] vals = ReadWriteFile.readFile(file).split("\n");
            for (String val : vals) {
                results.add(Double.parseDouble(val));
            }
        } catch (Exception e) {
            results.add(0.0); results.add(0.0); results.add(0.0); results.add(0.0);
        }
        isRed = results.get(results.size() - 4);
        double startX = results.get(results.size() - 3);
        double startY = results.get(results.size() - 2);
        double startHeadingDeg = Math.toRadians(results.get(results.size() - 1));
        Pose startPose = new Pose(startX, startY, startHeadingDeg);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        rgbLed = hardwareMap.get(ServoImplEx.class, "RGB");
        pos = new Position(startPose);
        turret = new Turret(hardwareMap);
        if(isRed == 1){
            pos.setRed();
            turret.setBlue(false);
        }else if (isRed == 0){
            pos.setBlue();
            turret.setBlue(true);
        }

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        distanceSensor2 = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor2");
        distanceSensor3 = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor3");
//        shooter.lowerHood();
//        shooter.middleBar();
//        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
//        colorSensor2.enableLed(true);

        setUp();
    }
    public void start(){
        bombTimer.reset();
        rgbLed.setPwmRange(new PwmControl.PwmRange(500, 2500));
        shooter.lowerHood();
        shooter.middleBar();
        checkArt.reset();
        toPark.reset();
    }

    public void loop() {
        follower.update();
        pos.update(follower.getPose());
        fsm.update();
        turret.addForOffset(gamepad2);
//        turret.setRed(pos.isRed());
        increaseDecrease();
        if(isRed == 1){
            pos.setRed();
            turret.setBlue(false);
        }else if (isRed == 0){
            pos.setBlue();
            turret.setBlue(true);
        }
        if(gamepad2.leftBumperWasPressed()) {
            if (isRed == 1) {
                isRed = 0;
            } else if (isRed == 0) {
                isRed = 1;
            }
        }
        if(!fieldCentric)
            movement.movementLoop(gamepad1);
        else
            movement.movementFieldCentric(gamepad1, follower.getPose().getHeading(), pos.isRed());

        if(gamepad1.triangleWasPressed()){
            fieldCentric = !fieldCentric;
        }
        hasBalls();
        if(toPark.seconds() >= 115 && stopRumbling){
            gamepad1.rumble(500);
            stopRumbling = false;
        }
        intake.take(gamepad1);
        index.feed(gamepad1);
        resetPosition(gamepad2, gamepad1);        //TODO Make it Gamepad2
//        pos.whereToShoot(gamepad2);
        // Lift Toggle
        if(gamepad2.rightBumperWasPressed()){
            useDistSens = !useDistSens;
        }
        if (gamepad1.dpadDownWasPressed()) {
            if (park == true)
                lift.lift();
            else
                lift.lower();
            park = !park;
        }
        if (gamepad2.triangleWasPressed()){
            stopTurret = !stopTurret;
            turret.goNeutral();
        }
        redBase = tick + 790.04194;
        if (lowerSpeed){
            index.setValue(0.59);
        }else{
            index.setValue(0.95);
        }
        if(gamepad2.circleWasPressed()){
            lowerSpeed = !lowerSpeed;
        }
        if(gamepad1.left_bumper && follower.getPose().getY() < 43){
            shooter.raiseBarrier();
            intake.autoTake();
            index.slowFeed();
        }else if (gamepad1.left_bumper && follower.getPose().getY() > 48){
            shooter.raiseBarrier();
            intake.autoTake();
            index.autoFeed();
        }else{
            shooter.lowerBarrier();
        }
        if(gamepad1.dpadRightWasPressed()){
            parking = !parking;
        }
        if(parking){
            turret.angleToPos(90);
        }
        if (pos.activateOrientation() && !stopTurret && !parking || follower.getPose().getY() < 0)
            turret.update();
        if (manual) {
            turret.goNeutral();
            if (gamepad1.left_bumper)
                shooter.raiseBarrier();
            else
                shooter.lowerBarrier();
        }
        double addOn = 260;
        // Shooter & Turret Updates using Follower Velocity
        double velX = follower.getVelocity().getXComponent();
        double velY = follower.getVelocity().getYComponent();
        boolean add = false;
        if(gamepad1.dpadRightWasPressed()){
            add = !add;
        }
        if(!parking) {
            if (pos.isRed() && !add) {
                shooter.setTicks(pos.getTicks(6.89911, redBase), follower.getPose().getY() < 50, false);
                turret.setTargetAngle(pos.getTargetAngle(gamepad1)); //+ angle if needed        //TODO changed from target to getTargetAngle or smth
                turret.setHeading(Math.toDegrees(follower.getHeading()), true);
//                turret.setOffsetAngle(pos.getOffetAngle(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), true));
            } else if (pos.isBlue() && !add) {
                shooter.setTicks(pos.getTicksBlue(6.89911, redBase) + 120, follower.getPose().getY() < 50, true);
                turret.setTargetAngle(pos.getTargetAngle(gamepad1)); //+ angle if needed        //TODO changed from target to getTargetAngle or smth
                turret.setHeading(Math.toDegrees(follower.getHeading()), false);
//                turret.setOffsetAngle(pos.getOffetAngle(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), false));
            }
            if (pos.isRed() && add) {
                shooter.setTicks(pos.getTicks(6.89911, redBase) + addOn, follower.getPose().getY() < 50, false);
                turret.setTargetAngle(pos.getTargetAngle(gamepad1)); //+ angle if needed        //TODO changed from target to getTargetAngle or smth
                turret.setHeading(Math.toDegrees(follower.getHeading()), true);
//                turret.setOffsetAngle(pos.getOffetAngle(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), true));
            } else if (pos.isBlue() && add) {
                shooter.setTicks(pos.getTicksBlue(6.89911, redBase) + addOn + 120, follower.getPose().getY() < 50, true);
                turret.setTargetAngle(pos.getTargetAngle(gamepad1)); //+ angle if needed        //TODO changed from target to getTargetAngle or smth
                turret.setHeading(Math.toDegrees(follower.getHeading()), false);
//                turret.setOffsetAngle(pos.getOffetAngle(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), false));
            }
            shooter.update();
//        }else if (parking && isRed == 1){
//            autoPark = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                            new Pose (33.000, 35.000)
//                    ))
//                    .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(135))
//                    .build();
//            follower.followPath(autoPark);
//            if(!follower.isBusy()){
//                lift.lift();
//            }
//        }else if (parking && isRed == 0){
//            autoPark = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(follower.getPose().getX(), follower.getPose().getY()),
//                            new Pose (111.000, 35.000)
//                    ))
//                    .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(45))
//                    .build();
//            follower.followPath(autoPark);
//            if(!follower.isBusy()){
//                lift.lift();
//            }
        }


        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter Velocity", shooter.getTicks());
        telemetry.addData("increased angle", angle);
        telemetry.addData("First servo position", turret.getPosition1());
        telemetry.addData("Second servo position", turret.getPosition2());
        telemetry.addData("Third servo position", turret.getPosition3());
        telemetry.addData("hood target", shooter.getTargetHood());
        telemetry.addData("adauga sau nu", turret.isAddAngle());
        telemetry.update();
    }
    private void hasBalls(){
        index.setLower(distanceSensor.getDistance(DistanceUnit.CM) < 6 && distanceSensor2.getDistance(DistanceUnit.CM) < 7);
    }
    private void increaseDecrease(){
        if(gamepad2.right_trigger > 0.2){
            tick = 100;
        }else if (gamepad2.left_trigger > 0.2){
            tick = 180;
        }else{
            tick = 0;
        }
    }
    private void setUp() {
        fsm.onStateEnter(State.NU_E_BILE, () -> { timer.reset(); return null; });
        fsm.onStateUpdate(State.NU_E_BILE, () -> {
//            index.normalIndex();
//            if(bombTimer.seconds() < 110)
                rgbLed.setPosition(0.7);
//            else{
//                return State.BOMB;
//            }
//            if(checkArt.milliseconds() >= 75 && useDistSens) {
//                distance = distanceSensor.getDistance(DistanceUnit.CM);
//                distance2 = distanceSensor2.getDistance(DistanceUnit.CM);
//                distance3 = distanceSensor3.getDistance(DistanceUnit.CM);
//                checkArt.reset();
//            }else if (!useDistSens){
//                if(gamepad2.squareWasPressed()){
//                    return State.E_BILE;
//                }
//            }
            if(gamepad2.squareWasPressed()){
                return State.E_BILE;
            }
//            if (timer.milliseconds() >= 450) {
//                if (!(Double.isNaN(distance) && Double.isNaN(distance2) && Double.isNaN(distance3))
//                        && (distance < 6 && distance2 < 6 && distance3 < 15))
//                {
//                    return State.E_BILE;
//                }
//                if(!(Double.isNaN(distance) && Double.isNaN(distance2) && Double.isNaN(distance3))
//                        && (distance < 6 && distance2 < 6 && distance3 > 8)){
//                    rgbLed.setPosition(0.9);
//                }else{
//                    rgbLed.setPosition(0.7);
//                }
//                timer.reset();
//            }
            return null;
        });
        fsm.onStateEnter(State.E_BILE, () -> {
            gamepad1.rumble(200);
            return null;
        });
        fsm.onStateUpdate(State.E_BILE, () -> {
            index.lowerIndex();
//            if(bombTimer.seconds() < 110)
                rgbLed.setPosition(0.5);
//            else{
//                return State.BOMB;
//            }
//            if (timer.milliseconds() >= 300 && useDistSens) {
//                double distance = distanceSensor.getDistance(DistanceUnit.CM);
//                double distance2 = distanceSensor2.getDistance(DistanceUnit.CM);
//                double distance3 = distanceSensor3.getDistance(DistanceUnit.CM);
                if (gamepad1.leftBumperWasPressed() || gamepad2.crossWasPressed())
                    return State.NU_E_BILE;
//                timer.reset();
//            }else if (gamepad1.leftBumperWasPressed() || gamepad2.crossWasPressed())
//                return State.NU_E_BILE;
            return null;
        });
        fsm.init();
    }
    private boolean blink(double currTime){
        return (currTime % 400) < 200;
    }
    private void resetPosition(Gamepad gamepad, Gamepad driver) {
        if(pos.isRed()) {
            if(driver.dpadLeftWasPressed()){
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        121.863, 3.48,        //other human
                        Math.toRadians(0)
                ));
                manual = false;
            }
            if (gamepad.dpadUpWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        120.86, 75.2791,        //Gate
                        Math.toRadians(0)
                ));
                manual = false;
            } else if (gamepad.dpadLeftWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        116.214, 125.133,    //basket
                        Math.toRadians(38.4)
                ));
                manual = false;
            } else if (gamepad.dpadDownWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                {
                    follower.setPose(new Pose(
                            47.4658, 12.92,    //Shooting zone far
                            Math.toRadians(-90)
                    ));
                    manual = false;
                }
                manual = false;
            }else if (gamepad.dpadRightWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        1.38, 20.934,    //our human player w/ ext
                        Math.toRadians(180)
                ));
                manual = false;
            }
        }else if(pos.isBlue()){
            if(driver.dpadLeftWasPressed()){
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        144 - 121.863, 3.48,        //other human
                        Math.toRadians(0)
                ));
                manual = false;
            }
            if (gamepad.dpadUpWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        23.14, 75.2791,        //Far shooting
                        Math.toRadians(180)
                ));
                manual = false;
            } else if (gamepad.dpadLeftWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        27.786, 125.133,    //basket
                        Math.toRadians(141.6)
                ));
                manual = false;
            } else if (gamepad.dpadDownWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                {
                    follower.setPose(new Pose(
                            144 - 47.4658, 12.92,    //Shooting zone far
                            Math.toRadians(-90)
                    ));
                    manual = false;
                }
                manual = false;
            }else if (gamepad.dpadRightWasPressed()) {
                movement.resetHeading();
                movement.setOff(0);
                follower.setPose(new Pose(
                        142.1786, 19.9337,    //our human player w/ ext
                        Math.toRadians(180)
                ));
                manual = false;
            }
        }
    }
}