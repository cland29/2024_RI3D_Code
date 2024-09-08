package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.NavX;
import org.firstinspires.ftc.teamcode.util.motor.DcMotorExBuilder;

public class Arm {

    public DcMotorEx armMotor;
    public final PIDFCoefficients pidCoef = new PIDFCoefficients(-0.5, 0.0, 0.0, 0.0);

    public Arm(HardwareMap hardwareMap, Telemetry telemtry) {
        armMotor = DcMotorExBuilder.create(hardwareMap, "armMotor")
                .withPositionalPIDF(10, 2, 0.0, 0.0)
                .withDirection(DcMotorSimple.Direction.FORWARD)
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .build();

        armMotor.setVelocity(2700);
    }


    public void setMotorPower(double power){
        armMotor.setPower(power);
    }

    public void setMotorPos(int degreesFromHorizontal){
        armMotor.setTargetPosition(degreesFromHorizontal);
    }

    public int getEncoderPosition(){
        return armMotor.getCurrentPosition();
    }

    public int getEncoderTarget(){
        return armMotor.getTargetPosition();
    }
}