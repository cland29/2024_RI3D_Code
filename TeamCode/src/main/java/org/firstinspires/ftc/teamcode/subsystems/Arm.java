package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareIDMap;
import org.firstinspires.ftc.teamcode.util.motor.DcMotorExBuilder;

public class Arm {

    public DcMotorEx leftArmMotor, rightArmMotor;
    public final PIDFCoefficients pidCoef = new PIDFCoefficients(-0.5, 0.0, 0.0, 0.0);

    public Arm(HardwareMap hardwareMap, Telemetry telemtry) {
        leftArmMotor = DcMotorExBuilder.create(hardwareMap, HardwareIDMap.LEFT_ARM_MOTOR_ID)
                .withPositionalPIDF(10, 2, 0.0, 0.0)
                .withDirection(DcMotorSimple.Direction.FORWARD)
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .build();

        rightArmMotor = DcMotorExBuilder.create(hardwareMap, HardwareIDMap.RIGHT_ARM_MOTOR_ID)
                .withPositionalPIDF(10, 2, 0.0, 0.0)
                .withDirection(DcMotorSimple.Direction.REVERSE)
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .build();

        leftArmMotor.setVelocity(2700);
        rightArmMotor.setVelocity(2700);
    }


    public void setMotorPower(double power){
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void setMotorPos(int degreesFromHorizontal){
        leftArmMotor.setTargetPosition(degreesFromHorizontal);
        rightArmMotor.setTargetPosition(degreesFromHorizontal);
    }

    public int getLeftEncoderPosition(){
        return leftArmMotor.getCurrentPosition();
    }

    public int getLeftEncoderTarget(){
        return leftArmMotor.getTargetPosition();
    }

    public int getRightEncoderPosition(){
        return leftArmMotor.getCurrentPosition();
    }

    public int getRightEncoderTarget(){
        return leftArmMotor.getTargetPosition();
    }
}