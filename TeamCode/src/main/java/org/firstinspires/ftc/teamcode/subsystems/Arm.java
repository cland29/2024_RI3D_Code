package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.NavX;

public class Arm {
    public DcMotorEx armMotor;
    public final PIDFCoefficients pidCoef = new PIDFCoefficients(-0.5, 0.0, 0.0, 0.0);



    public Arm(HardwareMap hardwareMap, Telemetry telemtry) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoef);
        armMotor.setTargetPosition(0);
        armMotor.setVelocityPIDFCoefficients(10, 2, 0.0, 0.0);
        armMotor.setPositionPIDFCoefficients(10);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(2700);

        //armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCoef);


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