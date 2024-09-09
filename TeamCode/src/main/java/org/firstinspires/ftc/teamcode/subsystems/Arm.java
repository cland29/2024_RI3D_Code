package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareIDMap;
import org.firstinspires.ftc.teamcode.util.motor.DcMotorExBuilder;

public class Arm {

    public DcMotorEx leftArmMotor, rightArmMotor;
    public CRServo leftIntakeServo, rightIntakeServo;
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

        leftIntakeServo = hardwareMap.get(CRServo.class, HardwareIDMap.LEFT_INTAKE_SERV0_ID);
        rightIntakeServo = hardwareMap.get(CRServo.class, HardwareIDMap.RIGHT_INTAKE_SERVO_ID);

        leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
    }


    public void setMotorPower(double power){
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void setMotorPos(int degreesFromHorizontal){
        leftArmMotor.setTargetPosition(degreesFromHorizontal);
        rightArmMotor.setTargetPosition(degreesFromHorizontal);
    }

    public void setIntakePower(double power){
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
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