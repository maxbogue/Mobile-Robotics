
#include <fstream>
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <iomanip> // for setw

const double PI = std::atan(1.0) * 4;
const double MIN_DISTANCE = 0.01;

double distance(double x1, double y1, double x2, double y2) {
    using namespace std;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double angle(double x1, double y1, double x2, double y2) {
    using namespace std;
    return atan2(y2 - y1, x2 - x1);
}

int main(int argc, char *argv[]) {
    using namespace PlayerCc;
    char* host = "localhost";
    int port = 9001;
    PlayerClient    robot(host, port);
    Position2dProxy pp(&robot, 0);
    
    std::cout << "Instantiated proxies" << std::endl;
    
    std::ifstream ifs(argv[1]);
    double xt, yt;
    ifs >> xt;
    ifs >> yt;
    
    std::cout << "First waypoint: " << xt << ", " << yt << std::endl;
    
    for (;;) {
        
        robot.Read();
        double x = pp.GetXPos();
        double y = pp.GetYPos();
        double a = pp.GetYaw();
        
        std::cout << x << ", " << y << ", " << a / PI << std::endl;
        
        double d = distance(x, y, xt, yt);
        
        // Check if we're close enough to the waypoint.
        if (d < MIN_DISTANCE) {
            // Check if the file is exhausted.
            if (!ifs.eof()) {
                ifs >> xt;
                ifs >> yt;
                std::cout << "New waypoint: " << xt << ", " << yt << std::endl;
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
