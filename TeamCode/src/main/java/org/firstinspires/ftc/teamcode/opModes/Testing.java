package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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
    private double ticks = 100;

    public void init(){
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        movement = new Movement(hardwareMap);
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
                        7.8, 7,
                        AngleUnit.DEGREES,
                        90
        ));
        pinpoint.recalibrateIMU();

        shooterCalculations = new ShooterCalculations(30, 45,
                                                    0, 1,       //TODO MUST TUNE
                                                    0.12, 15);
    }
    public void loop(){
        pinpoint.update();
        pos.setPose(pinpoint.getPosition());
        movement.movementLoop(gamepad1);
        if(gamepad1.crossWasPressed()){
            shooter.raiseBarrier();
        }
        if(gamepad1.leftBumperWasPressed()){
            shooter.lowerHood();
            shooter.lowerBarrier();
        }else if(gamepad1.rightBumperWasPressed()){
            shooter.raiseHood();
            shooter.raiseBarrier();
        }
        if(gamepad1.right_trigger >= 0.1){
            intake.take(gamepad1.right_trigger);
            index.feed(gamepad1.right_trigger);
        }else if(gamepad1.dpadDownWasPressed()){
            intake.spit();
            index.eject();
        }
        if(gamepad1.dpadLeftWasPressed()){
            ticks -= 100;
        }
        if(gamepad1.dpadRightWasPressed()){
            ticks += 100;
        }
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
        turret.setAngle(parameters.getTargetTurretDegrees() - pinpoint.getHeading(AngleUnit.DEGREES));
        turret.update();

        shooter.setTicks(parameters.getFlywheelSpeed());
        shooter.update();

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
