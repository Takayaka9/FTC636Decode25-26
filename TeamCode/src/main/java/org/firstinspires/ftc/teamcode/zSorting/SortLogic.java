package org.firstinspires.ftc.teamcode.zSorting;
//we should print sortData[] to telemetry and find a way to input preload or always preload the same
public class SortLogic {
    //input array required for function: String[] sortData = new String[3];

    public int ballCounter(boolean in, boolean out, int ballCount) {
        if (in) {
            ballCount++;
        }
        if (out) {
            ballCount--;
        }

        return ballCount;
    }

    public Object[] sortCycleLogic(Object[] sortData) {
        //strings to hold data to prevent overwriting
        Object inProgress2;
        Object inProgress0;

        //rotate ball positions
        inProgress2 = sortData[2];
        sortData[2] = sortData[1];
        inProgress0 = sortData[0];
        sortData[0] = inProgress2;
        sortData[1] = inProgress0;


        return sortData;
    }
    public Object[] updateIntakeArtifact(String newArtifact, Object[] sortData) {

        if (newArtifact.equals("g")) {
            sortData[0] = "g";
        } else if (newArtifact.equals("p")) {
            sortData[0] = "p";
        }

        return sortData;
    }

    public Object[] updateRequiredArtifact(String requiredArtifact, Object[] sortData) {
        //before calling with requiredArtifact, check required artifact is not already in sortData[1]
        int cycles = 0;

        if (requiredArtifact.equals("g")) {
            while (!sortData[1].equals("g")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
            sortData[3] = Integer.toString(cycles);
            sortData[1] = "";
        } else if (requiredArtifact.equals("p")) {
            while (!sortData[1].equals("p")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
            sortData[3] = cycles;
        }
        return sortData;
        //use sortData[3] to rotate the balls the correct amount of times
    }


    public Object[] updateRequiredMotif(String requiredMotif, Object[] sortData){
        int cycles = 0;

        if (requiredMotif.equals("GPP")) {
            while(!sortData[1].equals("g") && !sortData[0].equals("p") && !sortData[2].equals("p")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
        }
        if (requiredMotif.equals("PGP")) {
            while(!sortData[1].equals("p") && !sortData[0].equals("g") && !sortData[2].equals("p")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
        }
        if (requiredMotif.equals("PPP")) {
            while(!sortData[1].equals("p") && !sortData[0].equals("g") && !sortData[2].equals("p")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
        }

        sortData[3] = cycles;
        return sortData;
    }

    public Object[] updateShotArtifact(Object[] sortData) {
        sortData[1] = "";

        return  sortData;
    }
}
