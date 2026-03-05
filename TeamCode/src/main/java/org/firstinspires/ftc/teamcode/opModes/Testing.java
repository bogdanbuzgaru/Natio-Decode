package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Sensor;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
    private GoBildaPinpointDriver pinpoint;
    private Sensor sensor;
    private double ticks = 100;
    private Sensor colorSensor, colorSensor2;
    private ShooterCalculations shooterCalculations;
    private boolean manual = false;
    private boolean detects = false;
    private StateMachine<State> fsm = new StateMachine<>(State.NU_E_BILE);

    public void init(){
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        sensor = new Sensor(hardwareMap, "colorSensor");
        pos = new Position(new Pose2D(
                DistanceUnit.INCH,
                7.2440944808, 6.43700786745,
                AngleUnit.DEGREES,
                90
        ));
        colorSensor = new Sensor(hardwareMap, "colorSensor");
        colorSensor2 = new Sensor(hardwareMap, "colorSensor2");

        shooterCalculations = new ShooterCalculations(35, 45, 0.14,
                1, 8.85, 332);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(3.64173228, 5.5511811, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,
                7.2440944808, 6.43700786745,
                        AngleUnit.DEGREES,
                        90
        ));
        pinpoint.recalibrateIMU();
        setUp();
    }
    public void loop() {
        pinpoint.update();
        pos.update(pinpoint.getPosition());
        fsm.update();
        movement.movementLoop(gamepad1);
        intake.take(gamepad1);
        index.feed(gamepad1);
        pos.chooseAlliance(gamepad2);
        resetPosition(gamepad2);

        pos.whereToShoot(gamepad1);     //TODO new added

        ShooterCalculations.ShootingParameters parameters = shooterCalculations.calculateShootingParameters(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                130, 130, 45,
                Math.toRadians(-45),
                Math.toRadians(35),
                Math.toRadians(45));


        if(gamepad1.triangleWasPressed()){
            manual = !manual;
        }

        if (pos.activateOrientation() && !manual)
            turret.update();
        if ((pos.shootClose() || pos.shootHigh()) && !manual) {
            shooter.raiseBarrier();
        }
        if(!pos.shootClose() && !pos.shootHigh()){
            shooter.lowerBarrier();
        }

        if(manual){
            turret.goNeutral();

            if(gamepad1.leftBumperWasPressed()){
                shooter.raiseBarrier();
            }else if (gamepad1.rightBumperWasPressed()){
                shooter.lowerBarrier();
            }
        }
        if (Math.abs(parameters.getFlywheelSpeed() - shooter.getTicks()) > 49) {
            shooter.setHoodPosition(parameters.getHoodServoPosition());
            if(pos.isRed()) {
                shooter.setTicks(pos.getTicks(8.8057, 1098));
                turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
                turret.setTargetAngle(pos.getTargetAngle());
                turret.setOffsetAngle(pos.offsetAngleRed(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pos.getTicks(8.8057, 1098)));
            }
            else {
                shooter.setTicks(pos.getTicksBlue(8.8057, 1098));
                turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
                turret.setTargetAngle(pos.getTargetAngle());
                turret.setOffsetAngle(pos.offsetAngleBlue(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pos.getTicks(8.8057, 1098)));
            }
        }
        shooter.update();
    }
    private void setUp(){
        fsm.onStateEnter(State.NU_E_BILE, () -> {return null;});
        fsm.onStateUpdate(State.NU_E_BILE, () -> {
            boolean det = colorSensor.isGreen() || colorSensor2.isGreen() || colorSensor.isPurple() || colorSensor2.isPurple();
            if(det){
                gamepad1.rumble(700);
                return State.E_BILE;
            }
            return null;
        });
        fsm.onStateEnter(State.E_BILE, () -> {return null;});
        fsm.onStateUpdate(State.E_BILE, () -> {
            boolean det = colorSensor.isGreen() || colorSensor2.isGreen() || colorSensor.isPurple() || colorSensor2.isPurple();
            if(!det){
                return State.NU_E_BILE;
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
    private void rumble(){
        gamepad1.rumble(650);
        gamepad2.rumble(1000);
    }
    private void resetPosition(Gamepad gamepad){
        if(pos.isRed()){
            if (gamepad.dpadUpWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        actualPositionX(120.000), actualPositionY(96.000),        //Up
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadLeftWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        actualPositionX(0) + 2.48818898, Math.abs(actualPositionY(0)),    //opposite human player
                        AngleUnit.DEGREES,
                        180
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadDownWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        actualPositionX(120.000), actualPositionY(81.000),    //Up barrier
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadRightWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        144.000 - 9.7322834608, Math.abs(actualPositionY(0)),    //our human player
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
                        actualPositionXBlue(24.000), actualPositionY(96.000),        //Up
                        AngleUnit.DEGREES,
                        180
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadLeftWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        actualPositionXBlue(144) - 2.48818898, Math.abs(actualPositionY(0)),    //opposite human player
                        AngleUnit.DEGREES,
                        0
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadDownWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        actualPositionX(24.000), actualPositionY(81.000),    //Up barrier
                        AngleUnit.DEGREES,
                        180
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }else if(gamepad.dpadRightWasPressed()){
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,              //TODO tune
                        9.7322834608, Math.abs(actualPositionY(0)),    //our human player
                        AngleUnit.DEGREES,
                        180
                ));
                manual = false;
                pinpoint.recalibrateIMU();
            }
        }
    }
}
