#include "src/matplotlibcpp.h" //Graph Library
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
#include <filesystem>

namespace plt = matplotlibcpp;

// Landmarks
std::vector<std::pair<double, double>> landmarks = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
std::random_device rd;
std::mt19937 gen(rd());

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();

class Robot {
public:
    Robot()
    {
        // Constructor
        x = gen_real_random() * world_size; // robot's x coordinate
        y = gen_real_random() * world_size; // robot's y coordinate
        orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

        forward_noise = 0.0; //noise of the forward movement
        turn_noise = 0.0; //noise of the turn
        sense_noise = 0.0; //noise of the sensing
    }

    void set(double new_x, double new_y, double new_orient)
    {
        // Set robot new position and orientation
        if (new_x < 0 || new_x >= world_size)
            throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= world_size)
            throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    std::vector<double> sense()
    {
        // Measure the distances from the robot toward the landmarks
        std::vector<double> z(landmarks.size());
        double dist;

        for (uint64_t i = 0; i < landmarks.size(); i++) {
            dist = sqrt(pow((x - landmarks.at(i).first), 2) + pow((y - landmarks.at(i).second), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z.at(i) = dist;
        }
        return z;
    }

    Robot move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);

        // set particle
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    std::string show_pose()
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + std::to_string(x) + " y=" + std::to_string(y) + " orient=" + std::to_string(orient) + "]";
    }

    std::string read_sensors()
    {
        // Returns all the distances from the robot toward the landmarks
        std::vector<double> z = sense();
        std::string readings = "[";
        for (uint64_t i = 0; i < z.size(); i++) {
            readings += std::to_string(z[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;
    }

    double measurement_prob(std::vector<double> measurement)
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;
        double dist;

        for (uint64_t i = 0; i < landmarks.size(); i++) {
            dist = sqrt(pow((x - landmarks.at(i).first), 2) + pow((y - landmarks.at(i).second), 2));
            prob *= gaussian(dist, sense_noise, measurement.at(i));
        }

        return prob;
    }

    double x, y, orient; //robot poses
    double forward_noise, turn_noise, sense_noise; //robot noises

private:
    double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        std::normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Functions
double gen_real_random()
{
    // Generate real random between 0 and 1
    std::uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

double mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double evaluation(Robot r, std::vector<Robot> p, uint64_t n)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (uint64_t i = 0; i < n; i++) 
    {
        //the second part is because of world's cyclicity
        double dx = mod((p.at(i).x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p.at(i).y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}
double max(std::vector<double> arr, uint64_t n)
{
    // Identify the max element in an array
    double max = 0;
    for (uint64_t i = 0; i < n; i++) {
        if (arr.at(i) > max)
            max = arr.at(i);
    }
    return max;
}

void visualization(uint64_t n, Robot robot, int step, std::vector<Robot> p, std::vector<Robot> pr)
{
    //Draw the robot, landmarks, particles and resampled particles on a graph

    //Graph Format
    plt::title("MCL, step " + std::to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);

    //Draw particles in green
    for (uint64_t i = 0; i < n; i++) {
        plt::plot({ p.at(i).x }, { p.at(i).y }, "go");
    }

    //Draw resampled particles in yellow
    for (uint64_t i = 0; i < n; i++) {
        plt::plot({ pr.at(i).x }, { pr.at(i).y }, "yo");
    }

    //Draw landmarks in red
    for (uint64_t i = 0; i < landmarks.size(); i++) {
        plt::plot({ landmarks.at(i).first }, { landmarks.at(i).second }, "ro");
    }

    //Draw robot position in blue
    plt::plot({ robot.x }, { robot.y }, "bo");

    //Save the image and close the plot
    std::filesystem::path img{"../../Images/"};
    if(std::filesystem::is_directory(img)) {
        plt::save(img.string() + std::to_string(step) + ".png");
        plt::clf();
    } else {
        std::filesystem::create_directory(img);
        plt::save(img.string() + std::to_string(step) + ".png");
        plt::clf();
    }
}

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    myrobot.move(-M_PI / 2.0, 10.0);

    // Create a set of particles
    uint64_t n = 1000;
    std::vector<Robot> p(n);

    for (uint64_t i = 0; i < n; i++) 
    {
        p.at(i).set_noise(0.05, 0.05, 5.0);
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
    std::vector<double> z;

    //Iterating 50 times over the set of particles
    uint64_t steps = 50;
    for (uint64_t t = 0; t < steps; t++) 
    {

        //Move the robot and sense the environment afterwards
        myrobot = myrobot.move(0.1, 5.0);
        z = myrobot.sense();

        //Simulate a robot motion for each of these particles
        std::vector<Robot> p2(n);
        for (uint64_t i = 0; i < n; i++) 
        {
            p2.at(i) = p.at(i).move(0.1, 5.0);
            p.at(i) = p2.at(i);
        }

        //Generate particle weights depending on robot's measurement
        std::vector<double> w(n);
        for (uint64_t i = 0; i < n; i++) 
        {
            w.at(i) = p.at(i).measurement_prob(z);
        }

        //Resample the particles with a sample probability proportional to the importance weight
        std::vector<Robot> p3(n);
        int index = gen_real_random() * n;
        double beta = 0.0;
        double mw = max(w, n);
        
        for (uint64_t i = 0; i < n; i++) 
        {
            beta += gen_real_random() * 2.0 * mw;
            while (beta > w.at(index)) 
            {
                beta -= w.at(index);
                index = mod((index + 1), n);
            }
            p3.at(i) = p.at(index);
        }
        for (uint64_t k = 0; k < n; k++) 
        {
            p.at(k) = p3.at(k);
        }

        //Evaluate the Error
        std::cout << "Step = " << t << ", Evaluation = " << evaluation(myrobot, p, n) << std::endl;

        visualization(n, myrobot, t, p2, p3);

    } //End of Steps loop

    return 0;
}
