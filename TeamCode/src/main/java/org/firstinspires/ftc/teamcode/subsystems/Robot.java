package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.commands.State.*;

//import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.State;

public class Robot {

    public Drivetrain drivetrain;
    private static ElapsedTime robotTimer = new ElapsedTime();
    public boolean doClose = true, doClamp = true;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public Deposit deposit;
    public LinearSlides slides;
    public Intake intake;
    private State state = State.START;
    private GamepadEx driver = null;
    private GamepadEx operator = null;
    public Launcher plane;
    public State lastState = START;
    Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        plane = new Launcher(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx drive, GamepadEx op){
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new LinearSlides(hardwareMap);
        intake = new Intake(hardwareMap);
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
        switch(state) {
            case START:
                break;
            case REST:
                deposit.transitioning();
                if (doClose) {
                        claw.closeRight();
                        claw.closeLeft();
                    } else {
                        claw.openRight();
                        claw.openLeft();
                    }
                if(robotTimer.seconds()>0.5) {
                    arm.transition();
                    wrist.transition();
                    if (!doClose) {
                        claw.closeRight();
                        claw.closeLeft();
                    }
                    claw.closeRight();
                    claw.closeLeft();
                    deposit.openLeftClamp();
                    deposit.openRightClamp();
                    if(robotTimer.seconds()>1.2 && robotTimer.seconds()<1.9) {
                        intake.intakeManualIn();
                    }
                    if(robotTimer.seconds()>=1.9) {
                        intake.holdIntake();
                    }
                }
                break;

            case INTAKE:
                if(robotTimer.seconds()<0.7) {
                    intake.intakeManualOut();
                }
                if(robotTimer.seconds()>=0.7 && robotTimer.seconds()<1.2) {
                    intake.holdIntake();
                    arm.intake();
                    wrist.intake();
                }
                if(robotTimer.seconds()>1.7) {
                    claw.openLeft();
                    claw.openRight();
                    doClose = true;
                    doClamp = true;
                }
                if(robotTimer.seconds()>1.7 && operator!=null) {
                    if(!operator.isDown(GamepadKeys.Button.B)) {
                        state = REST;
                    }
                }
                break;

            case SHORT_DEPOSIT:
                if(robotTimer.seconds()<.7) {
                    intake.intakeManualOut();
                }
                if(robotTimer.seconds()>=.7) {
                    intake.holdIntake();
                    arm.shortDeposit();
                    wrist.shortDeposit();
                    doClose = false;
                }
                if(robotTimer.milliseconds()>700 && operator!=null) {
                    if(!operator.isDown(GamepadKeys.Button.X)) {
                        state = REST;
                    }
                }
                break;

            case EXTEND:
                claw.looseLeft();
                claw.looseRight();
                if(robotTimer.seconds()>0.5 && robotTimer.seconds()<1.0) {
                    deposit.readyToClamp();
                }
                if(robotTimer.seconds()>1.0 && robotTimer.seconds()<1.5) {
                    deposit.closeLeftClamp();
                    deposit.closeRightClamp();
                }
                if(robotTimer.seconds()>1.5 && operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slides.slidesManualUp();
                }
                if(robotTimer.seconds()>1.5 && !operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    slides.holdSlides();
                }
                break;

            case DEPOSIT:
                deposit.depositing();
                if(robotTimer.milliseconds()>700 && operator!=null) {
                    if(!operator.isDown(GamepadKeys.Button.A)) {
                        state = RETRACT;
                        robotTimer.reset();
                    }
                }
                break;

            case RETRACT:
                doClamp = false;
                deposit.openRightClamp();
                deposit.openLeftClamp();
                deposit.depositing();

            case RETURNING:
                claw.looseLeft();

                claw.looseRight();
                if(robotTimer.seconds()>0.5 && robotTimer.seconds()<1.0 && doClamp == true) {
                    deposit.transitioning();
                }
                if(robotTimer.seconds()>1 && robotTimer.seconds()<1.5 && doClamp == true) {
                    deposit.closeLeftClamp();
                    deposit.closeRightClamp();
                }
                if(robotTimer.seconds()>1.5 && operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slides.slidesManualDown();
                }
                if(robotTimer.seconds()>1.5 && !operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slides.holdSlides();
                }

                break;
            case CLIMB:
                slides.slidesManualDown();
                break;
            case LAUNCH_DRONE:
                plane.launch();
                break;

        }
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
