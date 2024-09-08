package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class Teleop extends OpMode
{
    // Declare OpMode members.
    private MecanumDrivetrain drivetrain;
    private Arm arm;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        makeSubsystems();

        telemetry.addData("Status", "Initialized    \nTeam Taco is ready to play.\n\n:)");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        drivetrain.mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (gamepad1.dpad_up){
            arm.setMotorPower(0.5);
        } else if (gamepad1.dpad_down) {
            arm.setMotorPower(-0.5);

        }else{
            arm.setMotorPower(0.0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    public void makeSubsystems(){
        drivetrain = new MecanumDrivetrain(this.hardwareMap, this.telemetry);
        arm = new Arm(this.hardwareMap, this.telemetry);
    }

}
