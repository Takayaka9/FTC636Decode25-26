package org.firstinspires.ftc.teamcode.states.Management.Handlers.FSM;

import org.firstinspires.ftc.teamcode.states.Management.SystemManager;

public interface State {
    void initiate(SystemManager robot);
    void update(SystemManager robot, TeleOpFSM tFSM);
    void end(SystemManager robot);
}
