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



    public static double armTransition = 0.52, armShortDeposit = 0.68, armIntaking = 0.99;
    public static double _rightClampPos = 0.5, _leftClampPos = 0.5, _depositPos = 0.5;
    public static double wristShortDeposit = 0.53, wristIntaking = 0.64, wristTransition = 0.73;
    public static double rightClawClose = 0.51, rightClawOpen = 0.58;
    public static double leftClawClose = 0.62, leftClawOpen = 0.54;
    public static double leftClawLoose = 0.62, rightClawLoose = 0.51;
    public static double planeHoldPos = 0.0, planeReleasePos = 1.0;
    public static boolean doClose = false;

    private ElapsedTime timer = new ElapsedTime();



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

        if (driver.isDown(Button.LEFT_BUMPER)) {
            bot.drivetrain.switchModes();
        }

        bot.drivetrain.setSpeed(desiredSpeed);


        if (driver.wasJustPressed(Button.DPAD_RIGHT)) {   //RESET BOT
            bot = new Robot(hardwareMap, telemetry);
        }

        /*if (driver.wasJustPressed(Button.A)) {            //uhhh no idea what this does
            tilt = !tilt;
            recess = !recess;
        }*/

        //if right bumper, put the arm thingy down and then the wrist up, and open the claws

        // ---------------------------- OPERATOR CODE ---------------------------- //

        //if dpad up slides go up
        //if dpad down slides go down

        if(operator.wasJustPressed(Button.A)) {
            bot.deposit.getRightClamp().setPosition(_rightClampPos);
        } else if(operator.wasJustPressed(Button.B)) {
            bot.deposit.getLeftClamp().setPosition(_leftClampPos);
        } else if(operator.wasJustPressed(Button.X)) {
            bot.deposit.getDeposit().setPosition(_depositPos);
        }

        if(operator.wasJustPressed(Button.DPAD_LEFT)) {
            bot.claw.getLeftClaw().setPosition(leftClawClose);
        } else if(operator.wasJustPressed(Button.DPAD_RIGHT)) {
            bot.claw.getRightClaw().setPosition(rightClawClose);
        }









        /*if (operator.wasJustPressed(Button.DPAD_UP)) {
            bot.plane.getPlane().setPosition(planeHoldPos);
        } else if (operator.wasJustPressed(Button.DPAD_DOWN)) {
            bot.plane.getPlane().setPosition(planeReleasePos);
        }*/

        telemetry.addLine("Left Slide Position: " + bot.slides.getLeftPosition());
        telemetry.addLine("Right Slide Position: " + bot.slides.getRightPosition());
        telemetry.update();

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