package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public interface State {
    void initiate(SystemManager robot);
    void update(SystemManager robot, TeleOpFSM tFSM);
    void end(SystemManager robot);
}
