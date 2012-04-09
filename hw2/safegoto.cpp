
#include <fstream>
#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace std;

const double PI = atan(1.0) * 4;
const double MIN_DISTANCE = 0.02;
const int SAMPLE_POINTS = 681 / 8;
const int SAMPLE_SIZE = 8;

// This class would be called Vector, but C++ is stupid and
// calls its lists vectors, so I guess I'll go with Point.
class Point {
public:
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
    double d() {
        return sqrt(x * x + y * y);
    }
    double a() {
        return atan2(y, x);
    }
    Point rotate(double da) {
        double ang = a() - da;
        double dis = d();
        return Point(dis * cos(ang), dis * sin(ang));
    }
};

// Operator overloading so I can treat these things like real vectors.

ostream& operator<<(ostream &strm, const Point &p) {
    return strm << "(" << p.x << ", " << p.y << ")";
}

Point operator+(const Point &p1, const Point &p2) {
    return Point(p1.x + p2.x, p1.y + p2.y);
}

Point operator-(const Point &p1, const Point &p2) {
    return Point(p1.x - p2.x, p1.y - p2.y);
}

Point operator*(const Point &p, double s) {
    return Point(p.x * s, p.y * s);
}

Point operator*(double s, const Point &p) {
    return Point(p.x * s, p.y * s);
}

double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double angle(double x1, double y1, double x2, double y2) {
    return atan2(y2 - y1, x2 - x1);
}

// Samples the ranger data down to SAMPLE_POINTS number of points
// by averaging over SAMPLE_SIZE number of original points for each.
void sampleData(PlayerCc::RangerProxy &rp, double rd[]) {
    for (int i = 0; i < SAMPLE_POINTS; i++) {
        rd[i] = 0;
        int groupStart = i * SAMPLE_SIZE;
        for (int j = groupStart; j < groupStart + SAMPLE_SIZE; j++) {
            rd[i] += rp[j];
        }
        rd[i] = rd[i] / SAMPLE_SIZE;
    }
}

Point laserToPoint(int i, double d) {
    double a = 2 * i * PI * SAMPLE_SIZE / 1024 - 2 * PI / 3;
    return Point(cos(a) * d, sin(a) * d);
}

vector< pair<double,int> > localMinima(double rd[]) {
    pair<double,int> last(rd[0], 0);
    bool newMin = true;
    vector< pair<double,int> > localMins;
    for (int i = 1; i < SAMPLE_POINTS; i++) {
        if (rd[i] > last.first) {
            if (newMin) {
                localMins.push_back(last);
                newMin = false;
            }
        } else {
            newMin = true;
        }
        last = pair<double,int>(rd[i], i);
    }
    if (newMin) localMins.push_back(last);
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
        
        const double Kg = 1.0;
        const double Ko = 1.0;
        
        sampleData(rp, rangerData);
        vector< pair<double,int> > objects = localMinima(rangerData);
        Point v = Point(xt - x, yt - y).rotate(a);
        cout << endl;
        cout << v << endl;
        v = v * (Kg / v.d());
        cout << v << endl;
        for (int i = 0; i < objects.size(); i++) {
            if (objects[i].first > 0) {
                cout << objects[i].first << " | " << objects[i].second << endl;
                Point o = laserToPoint(objects[i].second, objects[i].first);
                cout << o << " -> ";
                o = o * (Ko / pow(o.d(), 2));
                cout << o << endl;
                v = v - o;
            }
        }
        cout << v << endl;
        
        double speed = 0;
        double turn = v.a();
        cout << turn << endl;
        cout << v.d() << endl;
        
        // Turn the smaller direction.
        if (turn > PI) turn = 2 * PI - turn;
        if (turn < -PI) turn = 2 * PI + turn;
        
        // If we're within 45 degrees of the angle, go!
        if (abs(turn) < PI / 4) speed = min(0.5, v.d());
        
        pp.SetSpeed(speed, turn);
        
    }
}
