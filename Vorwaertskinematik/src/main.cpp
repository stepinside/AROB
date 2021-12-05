#include <stdio.h>
#include <math.h>

#define FILE_PATH "../../Vorwaertskinematik/data/data.csv"
#define NUM_VALUES 10000

int main()
{
    FILE *pFile;
    double globalPose[NUM_VALUES][3];
    double invRotMatrix[3][3];
    double vRobot[3];
    double vGlobal[3];
    double xR0, yR0;

    int numValues, n;
    double t, dt, endTime;
    double theta;
    double deltaTheta;

    int revolutionLeft;
    int revolutionRight;
    double angularSpeedLeft;
    double angularSpeedRight;
    int wheelRadius;   // => r
    int wheelDistance; // => 2d
    double wRadius;
    double wDistance;

    printf("Roboterdaten:\n");
    printf("=============\n");
    printf("Drehzahl linkes Rad [U/min]: ");
    scanf("%d", &revolutionLeft);
    printf("Drehzahl rechtes Rad [U/min]: ");
    scanf("%d", &revolutionRight);
    printf("Radius des Rad [mm]: ");
    scanf("%d", &wheelRadius);
    printf("Radabstand [mm]: ");
    scanf("%d", &wheelDistance);

    printf("\nStartzustand:\n");
    printf("===============\n");
    printf("X-Koordinate beim Start [in m]: ");
    scanf("%lf", &xR0);
    printf("Y-Koordinate beim Start [in m]: ");
    scanf("%lf", &yR0);
    printf("Winkel Theta beim Start [in Grad]: ");
    scanf("%lf", &theta);

    printf("\nAufzeichnung:\n");
    printf("=================\n");
    printf("Zeitabstand [ms]: ");
    scanf("%lf", &dt);
    printf("Endzeit [ms] ");
    scanf("%lf", &endTime);

    pFile = fopen(FILE_PATH, "wb");

    if (pFile == NULL)
    {
        printf("Datei kann nicht geoffnet werden!\n\n");
        fflush(stdin);
        getchar();
        return 1;
    }

    // calculate number of necessary values
    numValues = (int)(endTime / dt);
    if (numValues > NUM_VALUES)
    {
        printf("Zu vile Wert!\n");
        printf(" Zeitabstand erhoehen oder Endzeit verringern!\n");
        fclose(pFile);
        fflush(stdin);
        getchar();
        return 1;
    }

    // convert from mm in m and from ms in s
    wRadius = (double)wheelRadius / 1000;
    wDistance = (double)wheelDistance / 1000;
    dt = dt / 1000;
    // convert degree to rad
    theta = theta / 180 * M_PI;

    // calculate angular speed from wheel revolution
    angularSpeedLeft = 2 * M_PI * (double)revolutionLeft / 60;
    angularSpeedRight = 2 * M_PI * (double)revolutionRight / 60;

    // calculate velocity of robot in x direction
    vRobot[0] = wRadius / 2 * (angularSpeedLeft + angularSpeedRight);
    // velocity in y direction is 0 because wheel cannot turn
    vRobot[1] = 0;
    // calculate angular speed of robot
    vRobot[2] = wRadius / wDistance * (angularSpeedLeft - angularSpeedRight);

    // set the robot at the start position in the global plane
    globalPose[0][0] = xR0;
    globalPose[0][1] = yR0;
    globalPose[0][2] = theta;

    printf("t = %.6lf: x= %.9lf\ty = %.9lf\ttheta = %.6lf\n", 0.0, globalPose[0][0], globalPose[0][1], globalPose[0][2]);
    fprintf(pFile, "%.6lf;%.9lf;%.9lf;%.9lf\n", 0.0, globalPose[0][0], globalPose[0][1], globalPose[0][2]);

    n = 1;
    for (t = dt; n < numValues; t = t + dt, n++)
    {
        // calculate elements of the inverse rotation matrix
        invRotMatrix[0][0] = cos(theta);
        invRotMatrix[0][1] = -sin(theta);
        invRotMatrix[0][2] = 0;
        invRotMatrix[1][0] = sin(theta);
        invRotMatrix[1][1] = cos(theta);
        invRotMatrix[1][2] = 0;
        invRotMatrix[2][0] = 0;
        invRotMatrix[2][1] = 0;
        invRotMatrix[2][2] = 1;

        // calculate global velocity for current time
        vGlobal[0] = invRotMatrix[0][0] * vRobot[0] +
                     invRotMatrix[0][1] * vRobot[1] +
                     invRotMatrix[0][2] * vRobot[2];

        vGlobal[1] = invRotMatrix[1][0] * vRobot[0] +
                     invRotMatrix[1][1] * vRobot[1] +
                     invRotMatrix[1][2] * vRobot[2];

        vGlobal[2] = invRotMatrix[2][0] * vRobot[0] +
                     invRotMatrix[2][1] * vRobot[1] +
                     invRotMatrix[2][2] * vRobot[2];

        theta = theta - vGlobal[2] * dt;

        // calculate global pose for new time stamp
        globalPose[n][0] = globalPose[n - 1][0] + vGlobal[0] * dt;
        globalPose[n][1] = globalPose[n - 1][1] + vGlobal[1] * dt;
        globalPose[n][2] = globalPose[n - 1][2] + vGlobal[2] * dt;

        printf("t = %.6lf: x= %.9lf\ty = %.9lf\ttheta = %.6lf\n", t, globalPose[n][0], globalPose[n][1], globalPose[n][2]);
        fprintf(pFile, "%.6lf;%.9lf;%.9lf;%.9lf\n", t, globalPose[n][0], globalPose[n][1], globalPose[n][2]);
    }
    fclose(pFile);

    fflush(stdin);
    getchar();

    return 0;
}