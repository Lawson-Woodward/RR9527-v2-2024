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
public abstract class BaseOp extends OpMode {



    public static double travelingPos = 0.51, intakingPos=0.49, depositingPos=0.66;

    public static double clawClosePos = 0.12, clawReleaseOnePos = 0.35, clawReleaseTwoPos = 0.5, clawIntakePos = 0.35;

    public boolean driverCanControl = false, operatorCanControl = true;

    public static double planeHoldPos = 0.0, planeReleasePos = 1.0;



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
        if (loop++ == 10000) loop = 0;

        double startTime = System.currentTimeMillis();

        // ---------------------------- LOOPING ---------------------------- //
        driver.readButtons();
        operator.readButtons();

        // ---------------------------- DRIVER CODE ---------------------------- //
        double desiredSpeed = 0;

        bot.drivetrain.drive(driver);

        if (driver.wasJustPressed(Button.Y)) {                                        // DPad Up = Reset Gyro
            bot.drivetrain.recenter();
        }

        if (driver.getTrigger(Trigger.LEFT_TRIGGER) > 0.1) {                          // Relative speed by dead zone
            desiredSpeed = (0.7 - driver.getTrigger(Trigger.LEFT_TRIGGER) * 0.4) * multiplier;
        } else if (driver.getTrigger(Trigger.RIGHT_TRIGGER) > 0.1) {
            desiredSpeed = 1 * multiplier;
        } else {
            desiredSpeed = 0.7 * multiplier;
        }

        if (driver.isDown(Button.RIGHT_BUMPER)) {
            desiredSpeed *= 0.25;
        }

        bot.drivetrain.setSpeed(desiredSpeed);


        if (driver.wasJustPressed(Button.DPAD_RIGHT)) {   //RESET BOT
            bot = new Robot(hardwareMap, telemetry);
        }

        if (driver.wasJustPressed(Button.A)) {            //uhhh no idea what this does
            tilt = !tilt;
            recess = !recess;
        }

        if (driver.wasJustPressed(Button.LEFT_BUMPER)) {        //switch between FC and RC
            bot.drivetrain.switchModes();
        }

        if (driver.isDown(Button.X)) {
            driverCanControl=true;
            operatorCanControl=false;
            bot.intake.getLeftBrush().setPosition(0.5);
            bot.intake.getRightBrush().setPosition(0.5);
            bot.intake.getLeftLift().setPosition(0.6);
            bot.intake.getRightLift().setPosition(0.4);
            bot.claw.getClaw().setPosition(clawIntakePos);
            bot.slide.getSlides().setPower(-1);
            bot.intake.getCompliant().setPower(1);
            bot.intake.getCarWash().setPower(1);
        } else if (driver.isDown(Button.Y)) {
            bot.intake.getLeftBrush().setPosition(0.5);
            bot.intake.getRightBrush().setPosition(0.5);
            bot.intake.getLeftLift().setPosition(0.6);
            bot.intake.getRightLift().setPosition(0.4);
            bot.claw.getClaw().setPosition(clawIntakePos);
            bot.intake.getCompliant().setPower(-1);
            bot.intake.getCarWash().setPower(-1);
        } else if(driver.isDown(Button.A)) {
            bot.intake.getLeftBrush().setPosition(1.0);
            bot.intake.getRightBrush().setPosition(0.0);
        } else {
            bot.intake.getLeftBrush().setPosition(0.6);
            bot.intake.getRightBrush().setPosition(0.4);
            bot.intake.getLeftLift().setPosition(0.4);
            bot.intake.getRightLift().setPosition(0.6);
            if(driverCanControl) {bot.claw.getClaw().setPosition(clawClosePos);}
            if(driverCanControl) {bot.slide.getSlides().setPower(0); }
            bot.intake.getCompliant().setPower(0);
            bot.intake.getCarWash().setPower(0);
            driverCanControl = false;
            operatorCanControl=true;
        }

        // ---------------------------- OPERATOR CODE ---------------------------- //




        if (operator.wasJustPressed(Button.DPAD_UP)) {
            bot.plane.getPlane().setPosition(planeHoldPos);
        } else if (operator.wasJustPressed(Button.DPAD_DOWN)) {
            bot.plane.getPlane().setPosition(planeReleasePos);
        }




        if(operator.wasJustPressed(Button.B)) {
            bot.claw.getClaw().setPosition(clawClosePos);
        } else if (operator.wasJustPressed(Button.A)) {
            bot.claw.getClaw().setPosition(clawReleaseOnePos);
        } else if (operator.wasJustPressed(Button.X)) {
            bot.claw.getClaw().setPosition(clawReleaseTwoPos);
        }

        if (operator.isDown(Button.RIGHT_BUMPER)) {
            if(operatorCanControl) {
                bot.slide.slidesManualUp(bot.arm);
                bot.claw.getClaw().setPosition(0.12);
            }
            //if(operatorCanControl && bot.slide.getPosition()<500) {bot.slide.getSlides().setPower(1);}
        } else if (operator.isDown(Button.LEFT_BUMPER)) {
            bot.slide.slidesManualDown(bot.arm);
            bot.claw.getClaw().setPosition(0.12);
        } else { bot.slide.getSlides().setPower(0); }

        telemetry.addLine("Slide Position" + bot.slide.getPosition());
        telemetry.update();


        /*if (operator.isDown(Button.X)) {
            bot.intake.getCompliant().setPower(1);
            bot.intake.getCarWash().setPower(1);
        } else if (operator.isDown(Button.Y)) {
            bot.intake.getCompliant().setPower(-1);
            bot.intake.getCarWash().setPower(-1);
        } else {
            bot.intake.getCompliant().setPower(0);
            bot.intake.getCarWash().setPower(0);
        }*/

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
