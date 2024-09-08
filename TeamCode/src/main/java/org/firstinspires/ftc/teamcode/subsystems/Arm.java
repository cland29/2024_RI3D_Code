package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.NavX;

public class Arm {
    private DcMotor armMotor;



    public Arm(HardwareMap hardwareMap, Telemetry telemtry) {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        armMotor.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void setMotorPower(double power){
        armMotor.setPower(power);
    }


}