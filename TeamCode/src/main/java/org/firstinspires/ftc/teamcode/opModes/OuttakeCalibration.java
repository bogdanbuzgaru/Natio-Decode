package org.firstinspires.ftc.teamcode.opModes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pid.PIDController;
import org.firstinspires.ftc.teamcode.pid.SimpleMotorFeedforward;

@TeleOp
@Configurable
public class OuttakeCalibration extends LinearOpMode {
    public static  double ks, kv, ka, kp, target, vecocity, nominalVoltage = 10.7, voltage, pow;
    public static boolean manual = false;
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);
    org.firstinspires.ftc.teamcode.pid.PIDController p = new PIDController(kp, 0, 0);
    DcMotorEx motorLeft, motorRight;
    private double voltagee;
    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.get(DcMotorEx.class, "flywheel1");
        motorRight = hardwareMap.get(DcMotorEx.class, "flywheel2");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        voltagee = hardwareMap.voltageSensor.iterator().next().getVoltage();
        while (opModeIsActive()) {

            if(manual) {
                motorLeft.setPower(pow);
                motorRight.setPower(pow);
            }


            SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);
            p.setPID(kp, 0, 0);

            vecocity = motorLeft.getVelocity();

            double p_output = p.calculate(vecocity, target);
            double ff_ouput = ff.calculate(target);

            motorRight.setPower((p_output + ff_ouput) * (nominalVoltage / voltagee));
            motorLeft.setPower((p_output + ff_ouput) * (nominalVoltage / voltagee));

            telemetry.addData("Velocity", vecocity);
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }
}
