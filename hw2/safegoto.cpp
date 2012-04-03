
#include <fstream>
#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace std;

const double PI = atan(1.0) * 4;
const double MIN_DISTANCE = 0.02;
const int SAMPLE_POINTS = 64;
const int SAMPLE_SIZE = 8;

class Point {
public:
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

ostream& operator<<(ostream &strm, const Point &p) {
    strm << "(" << p.x << ", " << p.y << ")";
    return strm;
}

double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double angle(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1);
}

void sampleData(PlayerCc::RangerProxy &rp, double rd[]) {
    int START = rp.GetRangeCount() / 2 - SAMPLE_POINTS * SAMPLE_SIZE / 2;
    for (int i = 0; i < SAMPLE_POINTS; i++) {
        rd[i] = 0;
        int groupStart = START + i * SAMPLE_SIZE;
        for (int j = groupStart; j < groupStart + SAMPLE_SIZE; j++) {
            rd[i] += rp[j];
        }
        rd[i] = rd[i] / SAMPLE_SIZE;
    }
}

Point laserToPoint(int i, double d) {
    double a = (0.5 - (double)i / SAMPLE_POINTS) * PI;
    return Point(cos(a) * d, sin(a) * d);
}

vector<Point> localMinima(double rd[]) {
    pair<double,int> last(rd[0], 0);
    bool newMin = true;
    vector<Point> localMins;
    for (int i = 1; i < SAMPLE_POINTS; i++) {
        if (rd[i] > last.first) {
            if (newMin) {
                localMins.push_back(laserToPoint(last.second, last.first));
                newMin = false;
            } else {
                last = pair<double,int>(rd[i], i);
            }
        } else {
            last = pair<double,int>(rd[i], i);
            newMin = true;
        }
    }
    return localMins;
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
    
    double rangerData[SAMPLE_POINTS];
    
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
        
        sampleData(rp, rangerData);
        vector<Point> mins = localMinima(rangerData);
        for (int i = 0; i < mins.size(); i++) {
            cout << mins[i] << ", ";
        }
        cout << endl;
        
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
