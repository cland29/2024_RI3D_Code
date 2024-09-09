package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareIDMap;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.motor.DcMotorExBuilder;

public class Elevator {

    private DcMotorEx rightLiftMotor, leftLiftMotor;
    private Servo leftBucketServo;
    private Servo rightBucketServo;
    private double[] pidfValues = SubsystemConstants.ElevatorConstants.PIDF_VALUES;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry){
        rightLiftMotor = DcMotorExBuilder.create(hardwareMap, HardwareIDMap.RIGHT_LIFT_MOTOR_ID)
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .withPositionalPIDF(pidfValues[0], pidfValues[1], pidfValues[2], pidfValues[3])
                .withDirection(DcMotorSimple.Direction.FORWARD)
                .build();

        leftLiftMotor = DcMotorExBuilder.create(hardwareMap, HardwareIDMap.LEFT_LIFT_MOTOR_ID)
                .withZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                .withPositionalPIDF(pidfValues[0], pidfValues[1], pidfValues[2], pidfValues[3])
                .withDirection(DcMotorSimple.Direction.REVERSE)
                .build();

        leftBucketServo = hardwareMap.get(Servo.class, HardwareIDMap.LEFT_BUCKET_SERVO_ID);
        rightBucketServo = hardwareMap.get(Servo.class, HardwareIDMap.RIGHT_BUCKET_SERVO_ID);

        leftBucketServo.setDirection(Servo.Direction.FORWARD);
        rightBucketServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setMotorPos(int pos){
        rightLiftMotor.setTargetPosition(pos);
        leftLiftMotor.setTargetPosition(pos);
    }

    public void setBucketPos(int pos){
        leftBucketServo.setPosition(pos);
        rightBucketServo.setPosition(pos);
    }


}
