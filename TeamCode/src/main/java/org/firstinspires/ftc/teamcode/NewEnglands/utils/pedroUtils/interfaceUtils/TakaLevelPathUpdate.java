package org.firstinspires.ftc.teamcode.NewEnglands.utils.pedroUtils.interfaceUtils;

import org.firstinspires.ftc.teamcode.NewEnglands.pedro.paths.C9Paths;

public interface TakaLevelPathUpdate extends PathUpdate {
    void init();
    void start();
    BuildPaths buildPaths();
    void pathUpdate();
}
