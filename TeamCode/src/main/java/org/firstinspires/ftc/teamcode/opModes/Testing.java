package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
public class Testing extends OpMode {
    private Pose2D pose = new Pose2D(DistanceUnit.INCH, 7.8, 7, AngleUnit.DEGREES, 90);
    private Turret turret;
    private Shooter shooter;
    private Position pos;
    private ShooterCalculations shooterCalculations;
    private GoBildaPinpointDriver pinpoint;

    public void init(){
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pos = new Position(pose);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(pose);

        shooterCalculations = new ShooterCalculations(30, 45,
                                                    0, 1,       //TODO MUST TUNE
                                                    0.12, 15);
    }
    public void loop(){
        pinpoint.update();
        pos.setPose(pinpoint.getPosition());
        if(gamepad1.crossWasPressed()){
            shooter.raiseBarrier();
        }else if(gamepad1.leftBumperWasPressed()){
            turret.goLeft();
        }else if(gamepad1.rightBumperWasPressed()){
            turret.goRight();
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
    }
}
