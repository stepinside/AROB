#include <stdio.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define FILE_PATH "../../Drehzahlprofil/data/data.csv"
#define NUM_VALUES 30000
#define CHAR_PER_LINE 21

int main()
{
    FILE *pFile;

    double revProfileLeft[NUM_VALUES] = {0};
    double revProfileRight[NUM_VALUES] = {0};

    double invRotMatrix[3][3];
    double vRobot[3];
    double vGlobal[3];

    double xR0, yR0;
    double xGlobal, yGlobal, xGlobal_old, yGlobal_old;
    double thetaGlobal, thetaGlobal_old;

    int n;

    double t, dt;
    double theta;

    double angularSpeedLeft;
    double angularSpeedRight;
    double wRadius;
    double wDistance;

    char fileName[30];
    int numberLines;
    double revLeft, revRight;

    printf("Differentialantreib: Voerwaertskinematik\n\n");
    printf("Dateiname (Drehzahl-Profil): ");
    fflush(stdin);
    gets(fileName);

    pFile = fopen(fileName, "r");
    if (pFile == NULL)
    {
        printf("Datei kann nicht geoffnet werden!\n\n");
        fflush(stdin);
        getchar();
        return 1;
    }

    fscanf(pFile, "Anzahl Werte = %d\n", &numberLines);
    fscanf(pFile, "r = %lf;2d = %lf\n", &wRadius, &wDistance);
    fscanf(pFile, "x0 = %lf;y0 = %lf;theta0 = %lf\n", &xR0, &yR0, &theta);

    theta = theta / 180 * M_PI;

    if (numberLines > NUM_VALUES)
    {
        printf("Zu viele Werte!\n\n");
        fclose(pFile);

        fflush(stdin);
        getchar();
        return 1;
    }

    // get revolution values from the data file and store them in array
    for (int i = 0; i < numberLines; i++)
    {
        fscanf(pFile, "%lf;%lf;%lf", &t, &revLeft, &revRight);

        revProfileLeft[i] = revLeft;
        revProfileRight[i] = revRight;
    }
    fclose(pFile);

    printf("\nRoboterdaten:\n\n");
    printf("\tRadius des Rades: %d mm\n", (int)(wRadius * 1000));
    printf("\tRadabstand: %d mm\n\n", (int)(wRadius * 1000));
    printf("\tStartposition:\n");
    printf("\txR0 = %.3lf m\n", xR0);
    printf("\tyR0 = %.3lf m\n", yR0);
    printf("\ttheta0 = %.2lf Grad\n\n", theta);

    // set time base to 1 ms
    dt = 0.001;

    // open file to write the pose data
    pFile = fopen(FILE_PATH, "wb");

    if (pFile == NULL)
    {
        printf("Datei zum Schreiben kann nicht geoeffnet werden!\n\n");
        fflush(stdin);
        getchar();
    }

    t = 0;
    fprintf(pFile, "%.6lf;%.9lf;%.9lf;%.9lf\n", t, xR0, yR0, theta);
    xGlobal_old = xR0;
    yGlobal_old = yR0;
    thetaGlobal_old = theta;

    n = 1;
    for (t = dt; n < numberLines; t += dt, n++)
    {
        // calculate the elements of the inverse rotation matrix
        invRotMatrix[0][0] = cos(theta);
        invRotMatrix[0][1] = -sin(theta);
        invRotMatrix[0][2] = 0;
        invRotMatrix[1][0] = sin(theta);
        invRotMatrix[1][1] = cos(theta);
        invRotMatrix[1][2] = 0;
        invRotMatrix[2][0] = 0;
        invRotMatrix[2][1] = 0;
        invRotMatrix[2][2] = 1;

        angularSpeedLeft = 2 * M_PI * (double)revProfileLeft[n] / 60;
        angularSpeedRight = 2 * M_PI * (double)revProfileRight[n] / 60;

        // calculate velocity of the robot
        vRobot[0] = wRadius / 2 * (angularSpeedLeft + angularSpeedRight);
        vRobot[1] = 0;
        vRobot[2] = wRadius / wDistance * (angularSpeedLeft - angularSpeedRight);

        // calculate velocity in the global system
        vGlobal[0] = invRotMatrix[0][0] * vRobot[0] + invRotMatrix[0][1] * vRobot[1] + invRotMatrix[0][2] * vRobot[2];
        vGlobal[1] = invRotMatrix[1][0] * vRobot[0] + invRotMatrix[1][1] * vRobot[1] + invRotMatrix[1][2] * vRobot[2];
        vGlobal[2] = invRotMatrix[2][0] * vRobot[0] + invRotMatrix[2][1] * vRobot[1] + invRotMatrix[2][2] * vRobot[2];

        theta = theta - vGlobal[2] * dt;

        // calculate global pose for current time
        xGlobal = xGlobal_old + vGlobal[0] * dt;
        yGlobal = yGlobal_old + vGlobal[1] * dt;
        thetaGlobal = thetaGlobal_old - vGlobal[2] * dt;

        fprintf(pFile, "%.6lf;%.9lf;%.9lf;%.9lf\n", t, xGlobal, yGlobal, thetaGlobal);

        xGlobal_old = xGlobal;
        yGlobal_old = yGlobal;
        thetaGlobal_old = thetaGlobal;
    }

    fclose(pFile);

    printf("\nGlobale Roboterpositionen wurden bestimmt!\n");
    printf("\nDatei dataX.csv mit Positionsdaten wurde erzeugt!\n\n");
    fflush(stdin);
    getchar();
}