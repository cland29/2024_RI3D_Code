package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.motor.DcMotorExBuilder;

public class Elevator {

    private DcMotorEx liftMotor;
    private Servo leftBucketServo;
    private Servo rightBucketServo;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        liftMotor = DcMotorExBuilder.create(hardwareMap, "liftMotor")
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .withPositionalPIDF(10, 2, 0, 0)
                .build();

        leftBucketServo = hardwareMap.get(Servo.class, "leftBucketServo");
        rightBucketServo = hardwareMap.get(Servo.class, "rightBucketServo");

        leftBucketServo.setDirection(Servo.Direction.FORWARD);
        rightBucketServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setMotorPos(int pos){
        liftMotor.setTargetPosition(pos);
    }

    public void setBucketPos(int pos){
        leftBucketServo.setPosition(pos);
        rightBucketServo.setPosition(pos);
    }


}
