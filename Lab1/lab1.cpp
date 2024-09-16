#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "utils.h"
#include "render.h"

//===== Main parameters =====
const int width {800}, height {800};        //Width and height of the environment
const int radius {10};                      //Radius of the robot's circular body
const int min_obj_size {50};                //Maximum object dimension
const int max_obj_size {100};               //Maximum object dimension
const int goal_width {100};                 //Goal width
const int goal_height {100};                //Goal heigth
const int robot_tol {200};                  //Tolerance for robot spawn point
const int occupancy_tol {50};               //Minimum distance between all objects that spawn
const int goal_tol {100};                   //Minimum distance in x,y between robot and goal
const int robot_y_min {500};                //Minimum robot y position
const int goal_y_max {300};                 //Maximum goal y position
int obj_x, obj_y, obj_width, obj_height;    //Parameters for object position/size
int num_objects {0};                        //Number of obstacles in environment

// Grid utility class. Students will not use this for lab 1
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator to spawn robot and goal
random_generator rand_gen;

// Vector of robot positions to pass to renderer code. Update this after each time step!
std::vector<std::vector<int>> robot_pos;

// Did mission succeed? Update this to make sure it succeeds if robot reaches goal, failure if it hits wall.
bool succeed;

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT, GOAL, OBJECTS==========
    // create robot
    Object robot = grid.create_object(grid, rand_gen, robot_tol, 2*radius, 2*radius, robot_y_min, height-radius, 1, "robot");

    // create the goal
    Object goal = grid.create_object(grid, rand_gen, goal_tol, goal_width, goal_height, 0, goal_y_max, 3, "goal");

    // create the objects
    std::vector<Object> objects = grid.create_objects(rand_gen, occupancy_tol, num_objects);

    // create copies of robot and goal with their initial positions for purpose of render functions
    Object robot_init = robot;
    Object goal_init = goal;

    // uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv");

    // place the first robot position to robot_pos
    robot_pos.push_back({robot.x, robot.y});

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++WRITE YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Example provided: simply loop 100 times and move the robot right each time. 
        // This will also show a fail message as the succeed variable was never set to true
    // Modify this loop (can be while, for, or any type of loop) and add your code
    // You can define other functions to use in the top of this code
    for (int i=0; i<100; i++)
    {
        robot.x += 1;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++END YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // place the current robot position at the time step to robot_pos
        robot_pos.push_back({robot.x, robot.y});
    }
    
    // send the results of the code to the renderer
    render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);
    return 0;
}
