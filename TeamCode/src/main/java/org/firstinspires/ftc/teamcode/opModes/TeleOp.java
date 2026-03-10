package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Sensor;
import org.firstinspires.ftc.teamcode.movement.Movement;
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
    private GoBildaPinpointDriver pinpoint;
    private Sensor sensor;
    private double ticks = 100;
    private RevColorSensorV3 colorSensor, colorSensor2;
    private boolean manual = false;
    private boolean detects = false;
    private StateMachine<State> fsm = new StateMachine<>(State.NU_E_BILE);
    private Lift lift;
    private boolean park = true;
    private ElapsedTime leftBumperHoldTimer = new ElapsedTime();
    private boolean leftBumperWasPressed = false;
    private ElapsedTime timer = new ElapsedTime();

    public void init(){
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        String[] vals = ReadWriteFile.readFile(file).split("\n");
        for (String val : vals) {
            results.add(Double.parseDouble(val));
        }
        System.out.println(results.toString());

        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);
        sensor = new Sensor(hardwareMap, "colorSensor");
        pos = new Position(new Pose2D(
                DistanceUnit.INCH,
                results.get(results.toArray().length - 3), results.get(results.toArray().length - 2),
                AngleUnit.DEGREES,
                results.get(results.toArray().length - 1)
            ));
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.enableLed(true);
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        colorSensor2.enableLed(true);
//        shooterCalculations = new ShooterCalculations(35, 45, 0.14,
//                1, 8.85, 332);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(4.133859, 4.133858, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                results.get(results.toArray().length - 3), results.get(results.toArray().length - 2),
                AngleUnit.DEGREES,
                results.get(results.toArray().length - 1)
        ));
        pinpoint.recalibrateIMU();
        setUp();
    }
    public void loop(){

        pinpoint.update();
        pos.update(pinpoint.getPosition());
        fsm.update();
        movement.movementLoop(gamepad1);
        intake.take(gamepad1);
        index.feed(gamepad1);
        pos.chooseAlliance(gamepad2);
        resetPosition(gamepad2);

        pos.whereToShoot(gamepad1);     //TODO new added

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
        if(pinpoint.getPosY(DistanceUnit.INCH) >= 40){
            pos.setChangeCord(false);
        }else{
            pos.setChangeCord(true);
        }
        if(pos.isRed()) {
            shooter.setTicks(pos.getTicks(11.45522, 764.68357));
//            turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
            turret.setTargetAngle(pos.getTargetAngle());
            turret.setOffsetAngle(pos.offsetAngleRed(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pos.getTicks(8.8057, 1098)));
        }else if(pos.isBlue()){
            shooter.setTicks(pos.getTicksBlue(11.45522,764.68357));
//            turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
            turret.setTargetAngle(pos.getTargetAngle());
            turret.setOffsetAngle(pos.getOffetAngle(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)));
        }

        shooter.update();

        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("target angle", pos.getTargetAngle());
        telemetry.addData("velocity", shooter.getTicks());
        telemetry.addData("heading", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("First servo position", turret.getPosition1());
        telemetry.addData("Second servo position", turret.getPosition2());
        telemetry.addData("Third servo position", turret.getPosition3());
        telemetry.addData("Target angle", turret.getTargetAngle());
//        telemetry.addData("Angle ratio", turret.getAngleRatio());
    }
    private void setUp(){
        fsm.onStateEnter(State.NU_E_BILE, () -> {
            timer.reset();
            return null;});
        fsm.onStateUpdate(State.NU_E_BILE, () -> {
            if (timer.milliseconds() >= 450) {
                double distance = colorSensor.getDistance(DistanceUnit.CM);
                boolean ballPresent = !Double.isNaN(distance) && distance < 5.2;
                if(ballPresent){
                    gamepad1.rumble(200);
                    return State.E_BILE;
                }
                timer.reset();
                return null;
            }
            return null;
        });
        fsm.onStateEnter(State.E_BILE, () -> {return null;});
        fsm.onStateUpdate(State.E_BILE, () -> {
            if (timer.milliseconds() >= 300) {
                double distance = colorSensor.getDistance(DistanceUnit.CM);
                boolean ballAbsent = Double.isNaN(distance) || distance > 5.8;
                if(ballAbsent){
                    return State.NU_E_BILE;
                }
                timer.reset();
                return null;
            }
            return null;
        });
        fsm.init();
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
        if(pos.isRed()){
            if (gamepad.dpadUpWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        113.6, 102,        //Up
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadLeftWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        20.67, 18.2,    //opposite human player
                        AngleUnit.DEGREES,
                        180
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadDownWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        113.4, 91,    //Up barrier
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadRightWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        120.6, 24.9,    //our human player
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }
        }else if(pos.isBlue()){
            if (gamepad.dpadUpWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        30.4, 102,        //Up
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadLeftWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        120.6, 24.9,//opposite human player
                        AngleUnit.DEGREES,
                        180
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadDownWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        30.6, 91,    //Up barrier
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadRightWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        20.67, 18.2,       //our human player
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }
        }
    }
}
