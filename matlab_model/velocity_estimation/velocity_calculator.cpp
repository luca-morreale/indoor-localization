/**
 * Program used to estimate the velocity
 * of the robot between two adjacent points.
 *
 * Result: 0.00139048 time: 21ms
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#define TRAJECTORY_FILE "trajectory.txt"
#define MAX_TIME 400
#define LOWER_VELOCITY 0.277778     // 1 km/h in m/s
#define UPPER_VELOCITY 2.77778      // 10 km/h in m/s
#define CONVERSION_FACTOR 1000      // m/ms -> m/s

using namespace std;

typedef std::pair<double, double> point;

double dist(point, point);
void calculate_velocity(double, string);

double slow = LOWER_VELOCITY / CONVERSION_FACTOR;
double fast = UPPER_VELOCITY / CONVERSION_FACTOR;

int main() {

    ifstream in(TRAJECTORY_FILE);
    vector<point> trajectory;

    double x, y;
    for(unsigned int i=0; i<300; i++) {
        in >> x >> y;
        trajectory.push_back(make_pair(x, y));
    }
    in.close();

    double d = dist(trajectory[10], trajectory[11]);
    cout << "Distance: " << d << std::endl;

    cout << std::endl << std::endl << "m/ms scale" << std::endl;
    cout << "1 km/h: " << slow << std::endl;
    cout << "10 km/h: " << fast << std::endl << std::endl;
    calculate_velocity(d, "ms");

    slow *= CONVERSION_FACTOR;
    fast *= CONVERSION_FACTOR;

    cout << std::endl << std::endl << "m/s scale" << std::endl;
    cout << "1 km/h: " << slow << std::endl;
    cout << "10 km/h: " << fast << std::endl << std::endl;
    calculate_velocity(d, "s");

    return 0;
}

void calculate_velocity(double distance, string time_unit) {
    double v;

    for(unsigned int time = 1; time < MAX_TIME; time++){
        v = distance / time;
        if(v > slow && v < fast) {
            cout << "velocity: " << v << " time: " << time << time_unit << std::endl;
        }
    }
}

double dist(point a, point b) {

    double x = a.first - b.first;
    cout << "a.x: " << a.first << " b.x: " << b.first << std::endl;
    double y = a.second - b.second;
    cout << "a.y: " << a.second << " b.y: " << b.second << std::endl;

    return sqrt(x*x + y*y);
}
