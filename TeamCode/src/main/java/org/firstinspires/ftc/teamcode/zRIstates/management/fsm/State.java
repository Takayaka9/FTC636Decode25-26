package org.firstinspires.ftc.teamcode.zRIstates.management.fsm;

import org.firstinspires.ftc.teamcode.zRIstates.management.SystemManager;

public interface State {
    void initiate(SystemManager robot);
    void update(SystemManager robot, FSM tFSM);
    void end(SystemManager robot);
}
