package org.firstinspires.ftc.teamcode.nePremier.utils.fusionLL;

import com.pedropathing.geometry.Pose;

public interface Localize {
    Pose localize();
    boolean compare(Pose nPose);

}
