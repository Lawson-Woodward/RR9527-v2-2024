package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.commands.State.*;

//import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.State;

public class RobotEx {

    public Drivetrain drivetrain;
    private static ElapsedTime robotTimer = new ElapsedTime();
    public boolean doClose = true, doClamp = true;
    public Deposit deposit;
    public LinearSlides slides;
    public IntakeEx intake;
    private State state = State.REST;
    private GamepadEx driver;
    private GamepadEx operator;
    public Launcher plane;
    public State lastState = REST;
    Telemetry telemetry;

    public RobotEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx drive, GamepadEx op){
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hardwareMap);
        slides = new LinearSlides(hardwareMap);
        intake = new IntakeEx(hardwareMap, telemetry, drive, op);
        deposit = new Deposit(hardwareMap);
        driver = drive;
        operator = op;
        plane = new Launcher(hardwareMap);

        //arm = new Arm(hardwareMap);
        //claw = new Claw(hardwareMap);
        //slides = new LinearSlides(hardwareMap);
        //plane = new Launcher(hardwareMap);
    }


    public void executeTele() {
        driver.readButtons();
        operator.readButtons();
        telemetry.addData("Time: ", robotTimer.seconds());
        telemetry.addData("B is down: ", operator.isDown(GamepadKeys.Button.B));
        if(!(lastState==state)) {
            robotTimer.reset();
            lastState = state;
        }
        intake.executeTele(state);
        telemetry.update();
    }

    public void setPosition(State state){
        //actuator.setPosition(state);
        //slide.setPosition(state);
        //claw.setPosition(state);
        this.state = state;
    }

    public void setState(State state){this.state = state;}
    public State getState(){
        return state;
    }
}
