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
int obj_x, obj_y, obj_width, obj_height;    //P``arameters for object position/size
int num_objects {0};                        //Number of obstacles in environment

// Grid utility class. Students will not use this for lab 1
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator to spawn robot and goal
random_generator rand_gen;

// Vector of robot positions to pass to renderer code. Update this after each time step!
std::vector<std::vector<int>> robot_pos;

// Did mission succeed? Update this to make sure it succeeds if robot reaches goal, failure if it hits wall.
bool succeed;


// Task 7
struct Point{
    float x,y;
};


Point findCorner(const Object& goal, const Object& robot){
    Point corners[4] = {
        {goal.x, goal.y},
        {goal.x + goal.width, goal.y},
        {goal.x, goal.y + goal.height},
        {goal.x + goal.width, goal.y + goal.height}
    };

    Point closest = corners[0];
    float minDist = std::numeric_limits<float>::max();

    for (int i = 0; i < 4; i++) {
        float dx = robot.x - corners[i].x;
        float dy = robot.y - corners[i].y;
        float dist = dx * dx + dy * dy; 

        if (dist < minDist) {
            minDist = dist;
            closest = corners[i];
        }
    }
    return closest;
}

void moveTowards(Object& robot, const Point& target, float speed) {
    float dx = target.x - robot.x;
    float dy = target.y - robot.y;

    float distance = std::sqrt(dx * dx + dy * dy);
    if (distance > 0.0f) {
        robot.x += (dx / distance) * speed;
        robot.y += (dy / distance) * speed;
    }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++WRITE ANY FUNCTIONS OR GLOBAL VARIABLES HERE+++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// e.g. void my_func() {}
// e.g. int a = 5;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Task 1
bool checkBoundary(int x, int y){
    if(x >= width || x <= 0){
        return true;
    } 
    if(y >= height || y <= 0){
        return true;
    } 
    return false;
}

// Task 2
bool reachGoal(const Object& goal, const Object& robot, float radius) {
    return (robot.x + radius >= goal.x &&
            robot.x - radius <= goal.x + goal.width &&
            robot.y + radius >= goal.y &&
            robot.y - radius <= goal.y + goal.height);
}


int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT, GOAL, OBJECTS==========
    // create robot
    Object robot = grid.create_object(grid, rand_gen, robot_tol, 2*radius, 2*radius, robot_y_min, height-radius, 1, "robot");

    // create the goal
    Object goal = grid.create_object(grid, rand_gen, goal_tol, goal_width, goal_height, 0, goal_y_max, 3, "goal");

    // create the objects
    std::vector<Object> objects = grid.create_objects(rand_gen, occupancy_tol, num_objects);

    // create copies of robot and goal with their initial positions 
    Object robot_init = robot;
    Object goal_init = goal;

    // Task 6
    float centerX = goal.x + goal_width / 2.0f;
    float centerY = goal.y + goal_height / 2.0f;

    // place the first robot position to robot_pos
    robot_pos.push_back({robot.x, robot.y});

    // maximum count. Close the loop after 3600 iterations.
    int max_count=0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++DEFINE ANY LOCAL VARIABLE HERE+++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    
    // main while loop
    while (true)
    {
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++WRITE YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Example provided: simply loop 3600 times and move the robot right each time. 
        // This will also show a fail message as the succeed variable was never set to true
    // Do not change the while loop as it's made to end after 1 minute. This is to force this loop to eventually end so students can visualize code that gets stuck
    // You can define other functions to use outside of the main function if you wish
    // You may also define your own local variables inside main in addition to your own global variables. Make sure to know your variable scope

    // Task 5
    Point targetCorner {static_cast<float>(goal.x), static_cast<float>(goal.y)};
    moveTowards(robot, targetCorner, 1.0f);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++END YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // place the current robot position at the time step to robot_pos
        robot_pos.push_back({robot.x, robot.y});
        max_count++;

        // if more than a minute passed (in render window), exit
        if (max_count>=3600) {
            std::cout << "=====1 minute reached with no solution=====" << std::endl;
            break;
        }
        if(checkBoundary(robot.x, robot.y)){
            succeed = false;
            break;
        }

        if(reachGoal(goal, robot, radius)){
            succeed = true;
            break;
        }

    }
    
    // send the results of the code to the renderer
    render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);
    return 0;
}
