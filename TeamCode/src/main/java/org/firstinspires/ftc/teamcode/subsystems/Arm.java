package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareIDMap;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.motor.DcMotorExBuilder;

public class Arm {

    public DcMotorEx armMotor;
    public CRServo leftIntakeServo, rightIntakeServo;
    private double[] pidfValues = SubsystemConstants.ArmConstants.PIDF_VALUES;

    public Arm(HardwareMap hardwareMap, Telemetry telemtry) {
        armMotor = DcMotorExBuilder.create(hardwareMap, HardwareIDMap.LEFT_ARM_MOTOR_ID)
                .withPositionalPIDF(pidfValues[0], pidfValues[1], pidfValues[2], pidfValues[3])
                .withDirection(DcMotorSimple.Direction.FORWARD)
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .build();



        armMotor.setVelocity(SubsystemConstants.ArmConstants.MAX_VELOCITY);


        leftIntakeServo = hardwareMap.get(CRServo.class, HardwareIDMap.LEFT_INTAKE_SERV0_ID);
        rightIntakeServo = hardwareMap.get(CRServo.class, HardwareIDMap.RIGHT_INTAKE_SERVO_ID);

        leftIntakeServo.setDirection(CRServo.Direction.FORWARD);
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
    }


    public void setMotorPower(double power){
        armMotor.setPower(power);

    }

    public void setMotorPos(int degreesFromHorizontal){
        armMotor.setTargetPosition(degreesFromHorizontal);

    }

    public void setIntakePower(double power){
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    public int getLeftEncoderPosition(){
        return armMotor.getCurrentPosition();
    }

    public int getLeftEncoderTarget(){
        return armMotor.getTargetPosition();
    }

    public int getRightEncoderPosition(){
        return armMotor.getCurrentPosition();
    }

    public int getRightEncoderTarget(){
        return armMotor.getTargetPosition();
    }

    public boolean getBusy(){
        return armMotor.isBusy();
    }
}