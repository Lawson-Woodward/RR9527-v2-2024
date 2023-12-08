package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import org.firstinspires.ftc.teamcode.commands.VoltageReader;

import java.util.*;

@Config
public abstract class ConstraintTest extends OpMode {

    private Robot bot;

    private ElapsedTime runtime;
    private GamepadEx driver, operator;
    private VoltageReader voltageReader;
    int alliance;
    boolean canCheckI2C;
    double slideOverride;
    int loop;
    double multiplier, loopTime;
    private boolean curRT, oldRT, curLT, oldLT, tilt, recess, locked;
    int section = 0;
    List<LynxModule> allHubs;

    public abstract void setAlliance(); //-1 BLUE, 0 NEITHER, 1 RED

    public String allianceToString(){
        return alliance == -1 ? "BLUE" : (alliance == 0 ? "NEITHER" : "RED");
    }

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Status: Initializing");
        telemetry.update();

        setAlliance();

        bot = new Robot(hardwareMap, telemetry);
        loop = 0;

        voltageReader = new VoltageReader(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        tilt = true;
        recess = true;

        driver = new GamepadEx(gamepad1);   // drives the drivetrain
        operator = new GamepadEx(gamepad2); // controls the scoring systems
        runtime = new ElapsedTime();

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Alliance: " + allianceToString());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        multiplier = 1;
        if(loop++ == 10000) loop = 0;

        double startTime = System.currentTimeMillis();

        // ---------------------------- LOOPING ---------------------------- //
        driver.readButtons();
        operator.readButtons();

        // ---------------------------- DRIVER CODE ---------------------------- //
        double desiredSpeed = 0;

        bot.drivetrain.drive(driver);

        if(driver.wasJustPressed(Button.Y)) {                                        // DPad Up = Reset Gyro
            bot.drivetrain.recenter();
        }

        if(driver.getTrigger(Trigger.LEFT_TRIGGER) > 0.1) {                          // Relative speed by dead zone
            desiredSpeed = (0.7 - driver.getTrigger(Trigger.LEFT_TRIGGER) * 0.4) * multiplier;
        }
        else if(driver.getTrigger(Trigger.RIGHT_TRIGGER) > 0.1) {
            desiredSpeed = 1 * multiplier;
        }
        else {
            desiredSpeed = 0.7 * multiplier;
        }

        if(driver.isDown(Button.RIGHT_BUMPER)) {
            desiredSpeed *= 0.25;
        }

        bot.drivetrain.setSpeed(desiredSpeed);


        if(driver.wasJustPressed(Button.DPAD_RIGHT)){   //RESET BOT
            bot = new Robot(hardwareMap, telemetry);
        }

        if(driver.wasJustPressed(Button.A)){            //uhhh no idea what this does
            tilt = !tilt;
            recess = !recess;
        }

        if(driver.wasJustPressed(Button.LEFT_BUMPER)){        //switch between FC and RC
            bot.drivetrain.switchModes();
        }

        // ---------------------------- OPERATOR CODE ---------------------------- //

        if(operator.isDown(Button.DPAD_UP)) {
            //bot.actuator.getActuator().setPower(-0.5);
            bot.actuator.moveUp();
        }
        else if(operator.isDown(Button.DPAD_DOWN)) {
            //bot.actuator.getActuator().setPower(0.5);
            bot.actuator.moveDown();
        }
        else {
            bot.actuator.getActuator().setPower(0);
            //bot.actuator.setPower(0);
        }

        telemetry.addLine("Actuator Position: " + bot.actuator.getActuator().getCurrentPosition());
        //telemetry.update();

        //  ------------------------------ PID CODE ------------------------------ //

        if(operator.isDown(Button.RIGHT_BUMPER)) {
            bot.slide.slidesManualUp();
        }
        else if(operator.isDown(Button.LEFT_BUMPER)) {
            bot.slide.slidesManualDown();
        }
        else {
            bot.slide.getSlides().setPower(-0.07);
        }
        /*
        if(operator.wasJustPressed(Button.DPAD_LEFT)) {
            //bot.arm.getArm().setPosition(bot.arm.getArm().getPosition()+0.01);
            //bot.claw.getClaw().setPosition(0);
            bot.claw.getClaw().setPosition(bot.claw.getClaw().getPosition()+0.02);
        }
        if(operator.wasJustPressed(Button.DPAD_RIGHT)) {
            //bot.arm.getArm().setPosition(bot.arm.getArm().getPosition()-0.01);
            //bot.claw.getClaw().setPosition(.0784313725);
            bot.claw.getClaw().setPosition(bot.claw.getClaw().getPosition()-0.02);
        }
         */
        if (operator.wasJustPressed(Button.DPAD_LEFT)) {
            bot.intakeServo.moveDown();

        }
        if(operator.wasJustPressed(Button.DPAD_RIGHT)){
            bot.intakeServo.moveUp();
        }

        if(operator.wasJustPressed(Button.A)) {
            bot.arm.getArm().setPosition(.67);
        }
        if(operator.wasJustPressed(Button.X)) {
            bot.arm.getArm().setPosition(.5);
        }
        if(operator.wasJustPressed(Button.Y)) {
            bot.arm.getArm().setPosition(.0789);
        }



        telemetry.addLine("Claw Position: " + bot.claw.getClaw().getPosition());
        telemetry.addLine("Arm Position: " + bot.arm.getArm().getPosition());

        telemetry.addLine("Slides Position: " + bot.slide.getSlides().getCurrentPosition());
        telemetry.addLine("Intake Servo Position" + bot.intakeServo.getIntakeServoPosition());
        telemetry.update();

        /*telemetry.addData("currentPos: ",currentPosition);
        telemetry.addData("power: ", pidOutput);
        telemetry.addData("slidePrefere: ",slidePreference);*/
    }

    @Override
    public void stop() {
        super.stop();
    }

    public void teleTelemetry(){
        telemetry.addLine("Runtime: " + runtime.toString());
        telemetry.addLine("Looptime: " + loopTime);
        telemetry.addLine("Multiplier: " + multiplier);
        telemetry.addData("Mode: ", bot.drivetrain.getMode());
    }

    public void hubPowerTelemetry(){
        telemetry.addData("Voltage", voltageReader.getVoltage());
        for(int i = 0; i < allHubs.size(); i++){
            telemetry.addData("Current - Hub" + i, allHubs.get(i).getCurrent(CurrentUnit.AMPS));
        }
    }
}
