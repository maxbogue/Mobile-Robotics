
#include <fstream>
#include <iostream>
#include <libplayerc++/playerc++.h>

using namespace std;

const double PI = atan(1.0) * 4;
const double MIN_DISTANCE = 0.02;
const int SAMPLE_POINTS = 40;
const int SAMPLE_SIZE = 681 / SAMPLE_POINTS;

class Vector {
public:
    double x, y;
    Vector(double x, double y) : x(x), y(y) {}
    double d() {
        return sqrt(x * x + y * y);
    }
    double a() {
        return atan2(y, x);
    }
    Vector rotate(double da) {
        double ang = a() - da;
        double dis = d();
        return Vector(dis * cos(ang), dis * sin(ang));
    }
};

struct RangerData {
    double d;
    double a;
};

// Operator overloading so I can treat these things like real vectors.

ostream& operator<<(ostream &strm, const Vector &p) {
    return strm << "(" << p.x << ", " << p.y << ")";
}

Vector operator+(const Vector &p1, const Vector &p2) {
    return Vector(p1.x + p2.x, p1.y + p2.y);
}

Vector operator-(const Vector &p1, const Vector &p2) {
    return Vector(p1.x - p2.x, p1.y - p2.y);
}

Vector operator*(const Vector &p, double s) {
    return Vector(p.x * s, p.y * s);
}

Vector operator*(double s, const Vector &p) {
    return Vector(p.x * s, p.y * s);
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

Vector laserToVector(RangerData r) {
    return Vector(cos(r.a) * r.d, sin(r.a) * r.d);
}

vector<RangerData> localMinima(double rd[]) {
    double inc = 4.0 * PI / 3 / SAMPLE_POINTS;
    double ang = (0.5) * inc - 2.0 * PI / 3;
    RangerData last = {rd[0], ang};
    bool newMin = true;
    vector<RangerData> localMins;
    for (int i = 1; i < SAMPLE_POINTS; i++) {
        if (rd[i] > last.d) {
            if (newMin) {
                localMins.push_back(last);
                newMin = false;
            }
        } else {
            newMin = true;
        }
        ang = (i + 0.5) * inc - 2.0 * PI / 3;
        RangerData newLast = {rd[i], ang};
        last = newLast;
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
        
        const double Kg = 5.0;
        const double Ko = 3.0;
        
        sampleData(rp, rangerData);
        vector<RangerData> objects = localMinima(rangerData);
        Vector v = Vector(xt - x, yt - y).rotate(a);
        cout << endl;
        // cout << v << endl;
        v = v * (Kg / v.d());
        // cout << v << endl;
        for (int i = 0; i < objects.size(); i++) {
            if (objects[i].d > 0 && abs(objects[i].a) < PI / 2) {
                // cout << objects[i].d << " | " << objects[i].a << endl;
                Vector o = laserToVector(objects[i]);
                cout << o << endl;
                o = o * (Ko / pow(o.d(), 3));
                // cout << o << endl;
                v = v - o;
            }
        }
        cout << v << endl;
        
        double speed = 0;
        double turn = v.a() / 2;
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
