package org.firstinspires.ftc.teamcode.afterPremier.util;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.util.InterpLUT;
@Configurable
public class FlightTime {
    InterpLUT flightTime = new InterpLUT();
    public void create(){
        flightTime.add(0, 0.22);
        flightTime.add(36, 0.22);
        flightTime.add(53.6, 0.14);
        flightTime.add(73.5, 0.34);
        flightTime.add(100, 0.44);
        flightTime.add(135, 1.02);
        flightTime.add(150, 1.02);
        flightTime.add(10000000, 1.02);
        flightTime.createLUT();
    }
    public double getLUT(double input){
        return flightTime.get(input);
    }
}
