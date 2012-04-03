
#include <fstream>
#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace std;

const double PI = atan(1.0) * 4;
const double MIN_DISTANCE = 0.02;
const int SAMPLE_POINTS = 64;
const int SAMPLE_SIZE = 8;

double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double angle(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1);
}

int main(int argc, char *argv[]) {
    char* host = "localhost";
    int port = 9001;
    
    if (argc > 2) {
        host = argv[2];
        port = atoi(argv[3]);
    }
    
    PlayerCc::PlayerClient    robot(host, port);
    PlayerCc::RangerProxy     rp(&robot, 0);
    PlayerCc::Position2dProxy pp(&robot, 0);
    
    cout << "Instantiated proxies." << endl;
    
    ifstream ifs(argv[1]);
    if (!ifs.is_open()) {
        cout << "Waypoint file \"" << argv[1]
                  << "\" not found." << endl;
        return 1;
    }
    
    double xt, yt;
    ifs >> xt;
    ifs >> yt;
    
    cout << "First waypoint: " << xt << ", " << yt << endl;
    
    for (;;) {
        
        robot.Read();
        double x = pp.GetXPos();
        double y = pp.GetYPos();
        double a = pp.GetYaw();
        
        // cout << x << ", " << y << ", " << a / PI << endl;
        
        double d = distance(x, y, xt, yt);
        
        // Check if we're close enough to the waypoint.
        if (d < MIN_DISTANCE) {
            // Check if the file is exhausted.
            if (!ifs.eof()) {
                ifs >> xt;
                ifs >> yt;
                cout << "New waypoint: " << xt << ", " << yt << endl;
            } else {
                break;
            }
        }
        
        double speed = 0;
        double at = angle(x, y, xt, yt);
        double turn = at - a;
        
        // Turn the smaller direction.
        if (turn > PI) turn = 2 * PI - turn;
        if (turn < -PI) turn = 2 * PI + turn;
        
        // If we're with 90 degrees of the angle, go!
        if (abs(turn) < PI / 2) speed = d;
        
        pp.SetSpeed(min(0.5, speed), turn);
        
    }
}
