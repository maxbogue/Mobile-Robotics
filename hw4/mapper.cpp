// Example of pixel-plotting together with Player/Stage robot control
// zjb 4/10, based on code by John Hawley and original uncredited
//   Player/Stage example code.
// Not guaranteed to be decent code style but seems to work OK.

#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <pthread.h>
#include <cmath>

using namespace std;

#define WIN_X 800
#define WIN_Y 800

const int BLOCK = 8;

const double PI = atan(1.0) * 4;
const double SONAR[8][3] = {
    {  7.5,  13.0,  90*PI/180 },
    { 11.5,  11.5,  50*PI/180 },
    { 15.0,   8.0,  30*PI/180 },
    { 17.0,   2.5,  10*PI/180 },
    { 17.0,  -2.5, -10*PI/180 },
    { 15.0,  -8.0, -30*PI/180 },
    { 11.5, -11.5, -50*PI/180 },
    {  7.5, -13.0, -90*PI/180 }
};

static PlayerCc::PlayerClient *pRobot;
static PlayerCc::Position2dProxy *pPosition;
static PlayerCc::SonarProxy *pSonar;

static int good;

// values here should be between 0 and 1 to plot correctly.
// 0 will plot as white and 1 will plot as black.
static double localMap[WIN_X][WIN_Y];
static double oddsMap[WIN_X][WIN_Y];

static void display() {

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, WIN_X, 0, WIN_Y);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glBegin(GL_POINTS);
    for (int x = 0; x < WIN_X; x++) {
        for (int y = 0; y < WIN_Y; y++) {
            glColor3f(1-localMap[x][y], 1-localMap[x][y], 1-localMap[x][y]);
            glVertex2i(x, y);
        }
    }
    glEnd();

    // Flush the pipeline
    glFlush();

    // Swap the buffers now that we're done drawing.
    glutSwapBuffers();
    good = 1;
}

void redisplay() {
    if (good) glutPostRedisplay();
}

double toOdds(double prob) {
    return prob / (1 - prob);
}

double toProb(double odds) {
    return odds / (odds + 1);
}

void* robotLoop(void* args) {
    for (int i = 0; i < WIN_X; i++) {
        for (int j = 0; j < WIN_Y; j++) {
            localMap[i][j] = 0.5;
            oddsMap[i][j] = 1;
        }
    }
    while (true) {

        redisplay();

        double turnrate, speed;

        // read from the proxies
        try {
            pRobot->Read();
        } catch (PlayerCc::PlayerError e) {
            cout << e << endl;
            continue;
        }

        // Robot positions in cm, angle (yaw) in rads.
        double rx = pPosition->GetXPos() * 100 + WIN_X / 2 * BLOCK;
        double ry = pPosition->GetYPos() * 100 + WIN_Y / 2 * BLOCK;
        double ra = pPosition->GetYaw();

        // Here is where you do your robot stuff
        // including presumably updating your map somehow
        for (int s = 0; s < 8; s++) {
            double sd = (*pSonar)[s] * 100;
            // Sonar positions in cm and angle in rads.
            double sx = rx + SONAR[s][0] * cos(ra) - SONAR[s][1] * sin(ra);
            double sy = ry + SONAR[s][0] * sin(ra) + SONAR[s][1] * cos(ra);
            double sa = ra + SONAR[s][2];

            for (double d = 0; d <= sd + 10; d += BLOCK) {
                double p;
                if (sd >= 500 || d < sd - 10) {
                    p = 0.4;
                } else {
                    p = 3;
                }
                // Math that seems to work well, despite the strange result:
                //   arcLength = 2 * d * pi / 12 = d * pi / 6
                //   numPoints = arcLength / BLOCK (roughly)
                //   radsPerPoint = arc / numPoints
                //     = (pi / 6) / (d * pi / 6 / BLOCK)
                //     = BLOCK / d
                double bInc = BLOCK / d;
                for (double b = -PI / 12; b <= PI / 12; b += bInc) {
                    double a = sa + b;
                    int x = (int)(sx + d * cos(a)) / BLOCK;
                    int y = (int)(sy + d * sin(a)) / BLOCK;
                    if (x >= 0 && x < WIN_X && y >= 0 && y < WIN_Y) {
                        double u = pow(p, 1 - abs(b) * 12 / PI);
                        double o = oddsMap[x][y] * u;
                        oddsMap[x][y] = o;
                        localMap[x][y] = toProb(o);
                    }
                }
            }
        }
    }
}


int main(int argc, char *argv[]) {
    int port = 6665;
    char* host = "localhost";

    if (argc > 1) {
        port = atoi( argv[2] );
        host = argv[1];
    }

    pRobot = new PlayerCc::PlayerClient(host, port);
    pPosition = new PlayerCc::Position2dProxy(pRobot, 0);
    pSonar = new PlayerCc::SonarProxy(pRobot, 0);

    printf("player connected on port %d, proxies allocated\n", port);
    pPosition->SetMotorEnable(1);

    pthread_t thread_id;
    pthread_create(&thread_id, NULL, robotLoop, NULL);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowPosition(50, 50);
    glutInitWindowSize(WIN_X, WIN_Y);
    glutCreateWindow("Map");

    // Callbacks
    glutDisplayFunc(display);
    glutIdleFunc(redisplay);

    glutMainLoop();

    return 0;
}
