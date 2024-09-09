package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
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
    private CRServo leftBucketServo;
    private CRServo rightBucketServo;
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

        leftBucketServo = hardwareMap.get(CRServo.class, HardwareIDMap.LEFT_BUCKET_SERVO_ID);
        rightBucketServo = hardwareMap.get(CRServo.class, HardwareIDMap.RIGHT_BUCKET_SERVO_ID);

        leftBucketServo.setDirection(CRServo.Direction.FORWARD);
        rightBucketServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void setMotorPos(int pos){
        rightLiftMotor.setTargetPosition(pos);
        leftLiftMotor.setTargetPosition(pos);
    }

    public void setBucketPower(double power){
        leftBucketServo.setPower(power);
        rightBucketServo.setPower(power);
    }

    public boolean isBusy(){
        return leftLiftMotor.isBusy() || rightLiftMotor.isBusy();
    }


}
