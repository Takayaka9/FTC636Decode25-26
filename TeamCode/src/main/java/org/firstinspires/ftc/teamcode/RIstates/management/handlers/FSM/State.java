package org.firstinspires.ftc.teamcode.RIstates.management.handlers.FSM;

import org.firstinspires.ftc.teamcode.RIstates.management.SystemManager;

public interface State {
    void initiate(SystemManager robot);
    void update(SystemManager robot, FSM tFSM);
    void end(SystemManager robot);
}
