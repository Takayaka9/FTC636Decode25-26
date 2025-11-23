package org.firstinspires.ftc.teamcode.Sorting;
//we should print sortData[] to telemetry and find a way to input preload or always preload the same
public class SortLogic {
    //input array required for function: String[] sortData = new String[3];
    public String[] sortCycleLogic(String[] sortData) {
        //strings to hold data to prevent overwriting
        String inProgress2;
        String inProgress0;

        //rotate ball positions
        inProgress2 = sortData[2];
        sortData[2] = sortData[1];
        //belts would move to make space for 2 to drop in here (this comment only makes sense to emad)
        inProgress0 = sortData[0];
        sortData[0] = inProgress2;
        sortData[1] = inProgress0;


        return sortData;
    }
    public String[] updateSortData(String newArtifact, String requiredArtifact, String[] sortData) {
        int cycles = 0;

        if (newArtifact.equals("g")) {
            sortData[0] = "g";
        } else if (newArtifact.equals("p")) {
            sortData[0] = "p";
        }

        //before calling with requiredArtifact, check required artifact is not already in sortData[1]
        if (requiredArtifact.equals("g")) {
            while (!sortData[1].equals("g")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
            sortData[3] = Integer.toString(cycles);
        } else if (requiredArtifact.equals("p")) {
            while (!sortData[1].equals("p")) {
                sortData = sortCycleLogic(sortData);
                cycles++;
            }
            sortData[3] = Integer.toString(cycles);
        }


        return sortData;
        //use sortData[3] to rotate the balls the correct amount of times
    }
}
