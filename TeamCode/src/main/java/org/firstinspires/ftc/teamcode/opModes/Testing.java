package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Sensor;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@Configurable
@TeleOp
public class Testing extends OpMode {
    private Turret turret;
    private Shooter shooter;
    private Position pos;
    private Index index;
    private Intake intake;
    private Movement movement;
    private ShooterCalculations shooterCalculations;
    private GoBildaPinpointDriver pinpoint;
    private Sensor sensor;
    private double ticks = 100;

    public void init(){
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
        sensor = new Sensor(hardwareMap, "colorSensor");
        pos = new Position(new Pose2D(
                DistanceUnit.INCH,
                7.8, 7,
                AngleUnit.DEGREES,
                90));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(92.5, 141, DistanceUnit.MM);
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

        shooterCalculations = new ShooterCalculations(30, 45,
                                                    0.29, 1,       //TODO MUST TUNE
                                                    1.2, 100);
    }
    public void loop(){
        pinpoint.update();
        pos.update(pinpoint.getPosition());
        movement.movementLoop(gamepad1);
        intake.take(gamepad1);
        index.feed(gamepad1);
        ShooterCalculations.ShootingParameters parameters
                = shooterCalculations.calculateShootingParameters(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                138, 138,
                45, Math.toRadians(-30),
                Math.toRadians(30), Math.toRadians(45)
                );
        turret.setHeading(pinpoint.getHeading(AngleUnit.DEGREES));
        turret.setDifPos(parameters.getTurretOffsetDegrees());
        turret.setTargetAngle(pos.getTargetAngle());
        turret.setOffsetAngle(pos.getOffetAngle(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH)));
        turret.setAngle(pos.getAngle());
        if(pos.activateOrientation())
            turret.update();
        if(pos.shootClose() || pos.shootHigh()){
            shooter.raiseBarrier();
        }else{
            shooter.lowerBarrier();
        }

        shooter.setTicks(ticks);
//        shooter.setHoodPosition(parameters.getHoodServoPosition());
        shooter.update();
        telemetry.addData("Get turret target angle", turret.getTargetAngle());
        telemetry.addData("Get angle atan", pos.getAngle());
        telemetry.addData("Turns left", turret.turnLeft());
        telemetry.addData("Turns right", turret.turnRight());
        telemetry.addData("Target Turret Degrees", pos.getTargetAngle());
        telemetry.addData("Target from parameters", parameters.getTargetTurretDegrees());
        telemetry.addData("Offset movement prameters", parameters.getTurretOffsetDegrees());
        telemetry.addData("Ticks", parameters.getFlywheelSpeed());
        telemetry.addData("Is green", sensor.isGreen());
        telemetry.addData("Is purple", sensor.isPurple());
        telemetry.addData("heading", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Can shoot close", pos.shootClose());
        telemetry.addData("Can shoot high", pos.shootHigh());
        telemetry.addData("Max X", pos.getMaxX());
        telemetry.addData("Max Y", pos.getMaxY());
        telemetry.addData("Min X", pos.getMinX());
        telemetry.addData("Min Y", pos.getMinY());
        telemetry.update();
    }
}
