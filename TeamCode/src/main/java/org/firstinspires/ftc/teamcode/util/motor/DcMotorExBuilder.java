package org.firstinspires.ftc.teamcode.util.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotorExBuilder {

    private DcMotorEx motor;

    public DcMotorExBuilder(HardwareMap hardwareMap, String hardwareID){
        motor = hardwareMap.get(DcMotorImplEx.class, hardwareID);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotorExBuilder withPositionalPIDF(double p, double i, double d, double f){
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition(0);
        this.motor.setVelocityPIDFCoefficients(p, i, d, f);
        this.motor.setPositionPIDFCoefficients(p);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return this;
    }

    public DcMotorExBuilder withDirection(DcMotorSimple.Direction direction){
        this.motor.setDirection(direction);
        return this;
    }

    public DcMotorExBuilder withZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }

    public static DcMotorExBuilder create(HardwareMap hardwareMap, String hardwareID){
        return new DcMotorExBuilder(hardwareMap, hardwareID);
    }

    public DcMotorEx build(){
        return motor;
    }

}
