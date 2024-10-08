package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class Teleop extends OpMode
{
    // Declare OpMode members.
    private MecanumDrivetrain drivetrain;
    private Arm arm;
    private Elevator elevator;

    public enum State{
        Intaking,
        Stowed,
        Handoff,
        scoring
    }
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
        checkDriverController();
        checkOpController();
        telemetry.addData("Encoder Value", arm.getLeftEncoderPosition());
        telemetry.addData("Encoder Target", arm.getLeftEncoderTarget());
        telemetry.addData("isBusy", arm.armMotor.isBusy());
        telemetry.addData("tolerance", arm.armMotor.getTargetPositionTolerance());
        telemetry.addData("enabled?",  arm.armMotor.isMotorEnabled());
        telemetry.addData("Motor type?", arm.armMotor.getMotorType());
        telemetry.addData("Power", arm.armMotor.getPower());
        telemetry.update();
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
        elevator = new Elevator(this.hardwareMap, this.telemetry);
    }

    public void checkDriverController(){
        drivetrain.mecanumDrive_Cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);


        if (gamepad1.dpad_up){
            arm.setMotorPos(300);
            //arm.setMotorPower(0.5);
        } else if (gamepad1.dpad_down) {
            arm.setMotorPos(-900);
            //arm.setMotorPower(-0.5);
        }
    }

    public void checkOpController(){
        if(gamepad2.circle){
            intakePosArm();
            intake();

        }else if (gamepad2.cross){
            handOffGamepiece();
        }else if (gamepad2.square) {
            ejectFromBucket();
        }else if (gamepad2.dpad_down) {
            lowerElevator();
        }else if (gamepad2.dpad_up) {
            raiseElevator();
        }else{
            stowArm();
            idleIntake();
            idleBucket();
        }
    }



    public void intake(){
        arm.setIntakePower(0.5);
    }

    public void idleIntake(){
        arm.setIntakePower(0.0);
    }


    public void handOffGamepiece(){
        handoffArm();
        if (!arm.getBusy()){
            intake();
        }
    }

    public void lowerElevator(){
        stowArm();
        if (!arm.getBusy()){
            elevator.setMotorPos(0);
        }

    }

    public void raiseElevator(){
        stowArm();
        if (!arm.getBusy()){
            elevator.setMotorPos(1000);
        }

    }

    public void stowArm(){
        arm.setMotorPos(436);
    }

    public void handoffArm(){
        arm.setMotorPos(0);
    }

    public void intakePosArm(){
        arm.setMotorPos(436+875);
    }

    public void ejectFromBucket(){
        elevator.setBucketPower(0.25);
    }

    public void stowBucket(){
        elevator.setBucketPower(-0.25);
    }

    public void idleBucket(){
        elevator.setBucketPower(0.0);
    }


}
