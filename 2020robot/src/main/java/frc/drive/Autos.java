package frc.drive;

public class Autos{
    //[row(stage)][column(data, 0 for x, 1 for y, 2 for speed, 3 for special move #)]
    //special move - -1 for none(just drive), 0-whatever for whichever action to do, -2 for terminate
    public static final double[][] testAuto = {
        //{row#c0, row#c1, row#c2, row#c3},
        {1,2, 1, -1}, //x, y, speed, special move //r0
        {0,4, 1, -1}, //r1
        {0,5, 1, -1}, //r2
        {2,7, 1, -1}, //r3
        {0,0, 0, 0},  //actual data in special command doesn't matter at all
        {0,0, 0, -2}  //actual data in terminator doesn't matter at all
    };

    public static final double[][] auto1 = {
        //{row#c0, row#c1, row#c2, row#c3},
        {-3.5, 1.3, 1, -1}, //x, y, speed, special move //r0
        {-7, 5.25, 1, -1}, //r1
        {-11.5, 8, 1, -1}, //r2
        {-10, 11, 1, -1},
        {0, 0, 1, -1},
        {0,0, 0, -2}  //actual data in terminator doesn't matter at all
    };
}