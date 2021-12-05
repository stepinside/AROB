#include <stdio.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define FILE_PATH "../../Inversekinematik/data/data.csv"
#define NUM_SEGMENTS 20

enum segmentType
{
    TYPE_LINE,
    TYPE_CIRCLE,
    TYPE_ACCELERATED_LINE,
    TYPE_ACCELERATED_CIRCLE
};

enum circleDirection
{
    LEFT,
    RIGHT
};
typedef struct
{
    segmentType type;

    // only valid for line and circle
    double vel;

    // only valid for line and accelerated line
    double length;

    // onlyvalid for accelerated line and accelerated circle
    double vStart;
    double vEnd;

    // only valid for circle and accelerated circle
    double radOfRot;
    circleDirection dir;
    double angle;

    double dt; //* tAB in den Formeln

} Segment;

int main()
{
    Segment segments[NUM_SEGMENTS];

    int numSegments;
    double rWheel;
    double dWheels; //*dWheels = 2d
    char segType;
    char direction;

    double xR0, yR0, theta0;
    double t, dt, totalTime;
    double temp;
    double velLeft, velRight;
    double revLeft, revRight;
    int numValues;
    int numWrittenValues;

    FILE *pFile;

    printf("Differentialantreib: inverse Kinematik\n");
    printf("======================================\n\n");

    printf("Roboterdaten:\n");
    printf("=============\n\n");
    printf("\tRadius des Rades [mm]: ");
    scanf("%lf", &rWheel);
    rWheel = rWheel / 1000; // convert to m
    printf("\tRadabstand [mm]: ");
    scanf("%lf", &dWheels);
    dWheels = dWheels / 1000; // convert to m

    printf("\nStartzustand:\n");
    printf("==============\n");
    printf("\tX-Koordinate beim Start [m]: ");
    scanf("%lf", &xR0);
    printf("\tY-Koordinate beim Start [m]: ");
    scanf("%lf", &yR0);
    printf("\tWinkel Theta beim Start [Grad]: ");
    scanf("%lf", &theta0);

    printf("\n\nSegmente der Trajektorie:\n");
    printf("===========================\n\n");
    printf("Anzahl der Segmente: ");
    scanf("%d", &numSegments);

    if (numSegments > NUM_SEGMENTS)
    {
        printf("Es sind nur maximal %d Segmente erlaubt!\n\n", NUM_SEGMENTS);
        fflush(stdin);
        getchar();
        return 1;
    }

    for (int i = 0; i < numSegments; i++)
    {
        printf("\nSegment %d:\n", i + 1);
        printf("\tSegmenttyp (l ... Linie, k ... Kreis, a ... Linie beschleunigt, c ... Kreis beschleunigt): ");
        fflush(stdin);
        scanf("%c", &segType);

        switch (segType)
        {
        case 'l':
            segments[i].type = TYPE_LINE;
            break;
        case 'k':
            segments[i].type = TYPE_CIRCLE;
            break;
        case 'a':
            segments[i].type = TYPE_ACCELERATED_LINE;
            break;
        case 'c':
            segments[i].type = TYPE_ACCELERATED_CIRCLE;
            break;

        default:
            printf("Falscher Segmenttyp!\n\n");
            fflush(stdin);
            getchar();
            return 1;
            break;
        }

        switch (segments[i].type)
        {
        case TYPE_LINE:
            printf("\tGeschwindigkeit [m/s]: ");
            scanf("%lf", &segments[i].vel);

            printf("\tLaenge [m]: ");
            scanf("%lf", &segments[i].length);
            break;

        case TYPE_ACCELERATED_LINE:
            printf("\tStartgeschwindigkeit [m/s]: ");
            scanf("%lf", &segments[i].vStart);
            printf("\tEndgschwindigkeit [m/s] ");
            scanf("%lf", &segments[i].vEnd);

            printf("\tLaenge [m]: ");
            scanf("%lf", &segments[i].length);
            break;

        case TYPE_CIRCLE:
        case TYPE_ACCELERATED_CIRCLE:
            if (segments[i].type == TYPE_CIRCLE)
            {
                printf("\tGeschwindigkeit [m/s]: ");
                scanf("%lf", &segments[i].vel);
            }
            else
            {
                printf("\tStartgeschwindigkeit [m/s]: ");
                scanf("%lf", &segments[i].vStart);
                printf("\tEndgschwindigkeit [m/s] ");
                scanf("%lf", &segments[i].vEnd);
            }

            printf("\tRotationsradius [m]: ");
            scanf("%lf", &segments[i].radOfRot);
            printf("\tDrehrichtung (l ... Links, r ... Rechts: ");
            fflush(stdin);
            scanf("%c", &direction);
            if (direction == 'l')
                segments[i].dir = LEFT;
            else if (direction == 'r')
                segments[i].dir = RIGHT;
            else
            {
                printf("Falsche Richtungsangabe!\n\n");
                fflush(stdin);
                getchar();
                return 1;
            }

            printf("\tDrehwinkel [Grad]: ");
            scanf("%lf", &segments[i].angle);
            // convert degree in rad
            segments[i].angle = M_PI * segments[i].angle / 180;
            break;
        }
    }

    pFile = fopen(FILE_PATH, "wb");
    if (pFile == NULL)
    {
        printf("Datei kann nicht geoeffnet werden!\n\n");
        fflush(stdin);
        getchar();
        return 1;
    }

    totalTime = 0;
    dt = 0.001; // time resolution = 1 ms
    numWrittenValues = 0;

    fprintf(pFile, "Anzahl Werte = %6d\n", numWrittenValues); // dummy value, correct value is written later
    fprintf(pFile, "r = %6.3lf;2d = %7.3lf\n", rWheel, dWheels);
    fprintf(pFile, "x0 = %8.4lf;y0 = %8.4lf;theta0 = %8.4lf\n", xR0, yR0, theta0);

    // calculate the revolution profile for both wheels
    for (int i = 0; i < numSegments; i++)
    {
        switch (segments[i].type)
        {
        case TYPE_LINE:
            // calculate tAB
            segments[i].dt = segments[i].length / segments[i].vel;

            velLeft = segments[i].vel;
            velRight = segments[i].vel;
            revLeft = (30 * velLeft) / (M_PI * rWheel);
            revRight = revLeft;

            numValues = segments[i].dt / dt;
            t = 0;
            for (int n = 0; n < numValues; n++)
            {
                fprintf(pFile, "%5.3f;%6.2lf;%6.2lf\n", totalTime + t, revLeft, revRight);
                t += dt;
                numWrittenValues++;
            }

            break;
        case TYPE_ACCELERATED_LINE:

            segments[i].dt = 2 * segments[i].length / (segments[i].vStart + segments[i].vEnd);

            numValues = segments[i].dt / dt;
            t = 0;
            for (int n = 0; n < numValues; n++)
            {
                velLeft = segments[i].vStart + (segments[i].vEnd - segments[i].vStart) * t / segments[i].dt;
                velRight = velLeft;

                revLeft = (30 * velLeft) / (M_PI * rWheel);
                revRight = revLeft;

                fprintf(pFile, "%5.3lf; %6.2lf; %6.2lf\n", totalTime + t, revLeft, revRight);
                t += dt;
                numWrittenValues++;
            }
            break;
        case TYPE_CIRCLE:
            segments[i].dt = (segments[i].angle * segments[i].radOfRot) / segments[i].vel;

            velLeft = segments[i].vel * (1 + dWheels / 2 / segments[i].radOfRot);
            velRight = segments[i].vel * (1 - dWheels / 2 / segments[i].radOfRot);

            if (((segments[i].dir == LEFT) && (velLeft > velRight)) ||
                ((segments[i].dir == RIGHT) && (velLeft < velRight)))
            {
                temp = velLeft;
                velLeft = velRight;
                velRight = temp;
            }

            revLeft = (30 * velLeft) / (M_PI * rWheel);
            revRight = (30 * velRight) / (M_PI * rWheel);

            numValues = segments[i].dt / dt;
            t = 0;
            for (int n = 0; n < numValues; n++)
            {
                fprintf(pFile, "%5.3f;%6.2lf;%6.2lf\n", totalTime + t, revLeft, revRight);
                t += dt;
                numWrittenValues++;
            }

            break;
        case TYPE_ACCELERATED_CIRCLE:
            // calculate tAB
            segments[i].dt = 2 * (segments[i].angle * segments[i].radOfRot) / (segments[i].vStart + segments[i].vEnd);

            numValues = segments[i].dt / dt;
            t = 0;
            for (int n = 0; n < numValues; n++)
            {
                velLeft = (segments[i].vStart + (segments[i].vEnd - segments[i].vStart) * t / segments[i].dt) * (1 + dWheels / 2 / segments[i].radOfRot);
                velRight = (segments[i].vStart + (segments[i].vEnd - segments[i].vStart) * t / segments[i].dt) * (1 - dWheels / 2 / segments[i].radOfRot);

                revLeft = (30 * velLeft) / (M_PI * rWheel);
                revRight = (30 * velRight) / (M_PI * rWheel);

                if (((segments[i].dir == LEFT) && (velLeft > velRight)) ||
                    ((segments[i].dir == RIGHT) && (velLeft < velRight)))
                {
                    temp = velLeft;
                    velLeft = velRight;
                    velRight = temp;
                }

                revLeft = (30 * velLeft) / (M_PI * rWheel);
                revRight = (30 * velRight) / (M_PI * rWheel);

                fprintf(pFile, "%5.3f;%6.2lf;%6.2lf\n", totalTime + t, revLeft, revRight);
                t += dt;
                numWrittenValues++;
            }
            break;
        }

        totalTime += segments[i].dt;
    }

    // set file pointer to begin of file and write the number
    // of written values
    fseek(pFile, 0, SEEK_SET);
    fprintf(pFile, "Anzahl Werte = %6d\n", numWrittenValues);
    fclose(pFile);
    printf("\n\nDrehzahlprofil wurde berechnet!\n\n");

    fflush(stdin);
    getchar();
}