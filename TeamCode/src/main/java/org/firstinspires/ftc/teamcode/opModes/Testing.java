package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Sensor;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@Configurable
@TeleOp
public class Testing extends OpMode {
    private enum State{
        E_BILE,
        NU_E_BILE
    }
    private Turret turret;
    private Shooter shooter;
    private Position pos;
    private Index index;
    private Intake intake;
    private Movement movement;
//    private GoBildaPinpointDriver pinpoint;
    private Sensor sensor;
    private double ticks = 100;
    private boolean manual = false;
    private boolean detects = false;
    private StateMachine<State> fsm = new StateMachine<>(State.NU_E_BILE);
    private Lift lift;
    private boolean park = true;
    private ElapsedTime leftBumperHoldTimer = new ElapsedTime();
    private boolean leftBumperWasPressed = false;
    private ElapsedTime timer = new ElapsedTime();
    public static Follower follower;
    private int angle = 0;
    private double tick = 0;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7.2440944808, 7.08661417, Math.toRadians(90)));
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);
        sensor = new Sensor(hardwareMap, "colorSensor");
        pos = new Position(new Pose(
                7.2440944808, 7.08661417,
                Math.toRadians(90)
        ));

//        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
//        colorSensor2.enableLed(true);

//        shooterCalculations = new ShooterCalculations(35, 45, 0.14,
//                1, 8.85, 332);
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setOffsets(-3.5433070866, 4.2519685039, DistanceUnit.INCH);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        pinpoint.setPosition(new Pose2D(
//                        DistanceUnit.INCH,
//                7.2440944808, 6.43700786745,
//                        AngleUnit.DEGREES,
//                        90
//        ));
//        pinpoint.recalibrateIMU();
        setUp();
    }
    public void loop() {

        follower.update();
        pos.update(follower.getPose());
        fsm.update();
        movement.movementLoop(gamepad1);
        intake.take(gamepad1);
        index.feed(gamepad1);
        pos.chooseAlliance(gamepad2);
        resetPosition(gamepad2);
        shooter.setVoltagee(hardwareMap.voltageSensor.iterator().next().getVoltage());
        pos.whereToShoot(gamepad1);

        increaseAngle();
        increaseDecrease();
        if(gamepad1.dpadDownWasPressed()){
            if(park){
                lift.lift();
            }else{
                lift.lower();
            }
            park = !park;
        }

        if(gamepad1.triangleWasPressed()){
            manual = !manual;
        }
        if((isLeftBumperHeld(0.0006) && (pos.shootClose() || pos.shootHigh()) || isLeftBumperHeld(0.5))){
            shooter.raiseBarrier();
            intake.autoTake();
            index.autoFeed();
        }else{
            shooter.lowerBarrier();
        }
        if (pos.activateOrientation() && !manual)
            turret.update();

        if(manual){
            turret.goNeutral();

            if(gamepad1.leftBumperWasPressed()){
                shooter.raiseBarrier();
            }else if (gamepad1.rightBumperWasPressed()){
                shooter.lowerBarrier();
            }
        }
//        shooter.lowerHood(gamepad1);
//        if(pos.isRed()) {
//            shooter.setTicks(pos.getTicks(6.89911, 1100.04194) + ticks);        //5.4
////            turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
//            turret.setTargetAngle(pos.getTargetAngle() + angle);
//            turret.setOffsetAngle(pos.offsetAngleRed(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), pos.getTicks(6.89911, 1100.04194)));
//        }else if(pos.isBlue()){
//            shooter.setTicks(pos.getTicksBlue(6.89911,1100.04194) + ticks);
////            turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
//            turret.setTargetAngle(pos.getTargetAngle() + angle);
//            turret.setOffsetAngle(pos.getOffetAngle(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
//        }

//        shooter.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("target angle", pos.getTargetAngle());
//        telemetry.addData("velocity", shooter.getTicks());
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
//        telemetry.addData("First servo position", turret.getPosition1());
//        telemetry.addData("Second servo position", turret.getPosition2());
//        telemetry.addData("Third servo position", turret.getPosition3());
//        telemetry.addData("Target angle", turret.getTargetAngle());
//        telemetry.addData("Hood", shooter.getHoodPosition());
//        telemetry.addData("TARGET HOOD", shooter.getTarget());
//        telemetry.addData("Angle added", angle);
//        telemetry.addData("Angle ratio", turret.getAngleRatio());
    }
    private void increaseAngle(){
        if(gamepad2.circleWasPressed()){
            angle += 5;
        }else if(gamepad2.squareWasPressed()) {
            angle -= 4;
        }else if (gamepad2.triangleWasPressed()){
            angle = 0;
        }
    }
    private void increaseDecrease(){
        if(gamepad2.right_trigger > 0.01){
            tick = 100;
        }else if (gamepad2.left_trigger > 0.01){
            ticks = -120;
        }else{
            ticks = 0;
        }
    }
    private void setUp(){
        fsm.onStateEnter(State.NU_E_BILE, () -> {
            timer.reset();
            return null;});
        fsm.onStateUpdate(State.NU_E_BILE, () -> {
            if (timer.milliseconds() >= 450) {


                timer.reset();
                return null;
            }
            return null;
        });
        fsm.onStateEnter(State.E_BILE, () -> {return null;});
        fsm.onStateUpdate(State.E_BILE, () -> {
            if (timer.milliseconds() >= 300) {

                timer.reset();
                return null;
            }
            return null;
        });
        fsm.init();
    }
    private double actualPositionX(double value){
        return value + 7.2440944808;       //2.48818898
    }
    private double actualPositionXBlue(double value){
        return value - 7.2440944808;
    }
    private double actualPositionY(double value){
        return value - (6.43700786745 + 0.236220472);
    }

    private boolean isLeftBumperHeld(double holdTimeSeconds) {
        if (gamepad1.left_bumper) {
            // Button is currently pressed
            if (!leftBumperWasPressed) {
                // Button just pressed, start timer
                leftBumperHoldTimer.reset();
                leftBumperWasPressed = true;
            }
            // Check if held long enough
            if (leftBumperHoldTimer.seconds() >= holdTimeSeconds) {
                return true;
            }
        } else {
            // Button released, reset state
            leftBumperWasPressed = false;
        }

        return false;
    }
    private void resetPosition(Gamepad gamepad){
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
