#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>

#include "utils.h"
#include "render.h"

#define PI 3.14159265358979323846

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
class my_robot : public Object {
    private:
        int lidar_range{50};
        int radius{10};
        grid_util* true_grid_ptr = nullptr;
        std::vector<std::vector<int>> grid{800, std::vector<int>(800, -1)};
        std::queue<std::vector<int>> paths_queue;
        const float angle = std::cos(45 * PI / 180);
        bool clockwise{true};

    public:

        my_robot(int w, int h, int env_width, int y_min, int x_min, int tolerance) : Object(w, h, env_width, y_min, x_min, tolerance) {}
        
    //TASK 1
    
        void createGrid(grid_util& grid) {
                true_grid_ptr = &grid;
        }

        std::vector<std::vector<int>> getGrid() {
                return this->grid;
        }

        grid_util& true_grid() {
            return *true_grid_ptr;
        }

        void changeDirection() {
            clockwise = !clockwise;
        }

        void findDirection() {
            return clockwise;
        }

        void sensor() {
            int x_c = this->x + 10;
            int y_c = this->y + 10;

            for (int i = x_c - lidar_range; i <= x_c + lidar_range; ++i) {
                for (int j = y_c - lidar_range; j <= y_c + lidar_range; ++j) {
                    if (i >= 0 && i < 800 && j >= 0 && j < 800) {
                        int dx = i - x_c;
                        int dy = j - y_c;
                        if ((dx * dx) + (dy * dy) <= (lidar_range * lidar_range)) {
                            grid[i][j] = Object::grid_value(g, this, i, j, lidar_range);
                        }
                    }
                }
            }
        }

        std::vector<int> detect() {
            std::vector<int> direction(4, 0);
            int x_c = this->x + 10;
            int y_c = this->y + 10;
            int tolerance = 20;
            const int diagTolerance = tolerance * angle;

            if (grid[x_c][y_c - tolerance] == 1) 
                direction[0] = 1;

            else if (grid[x_c][y_c + tolerance] == 1) 
                direction[0] = -1;

            if (grid[x_c + tolerance][y_c] == 1) 
                direction[1] = 1;

            else if (grid[x_c - tolerance][y_c] == 1) 
                direction[1] = -1;

            if (grid[x_c - diagTolerance][y_c - diagTolerance] == 1) 
                direction[2] = 1;

            else if (grid[x_c + diagTolerance][y_c + diagTolerance] == 1) 
                direction[2] = -1;

            if (grid[x_c + diagTolerance][y_c - diagTolerance] == 1) 
                direction[3] = 1;

            else if (grid[x_c - diagTolerance][y_c + diagTolerance] == 1) 
                direction[3] = -1;

            return direction;
        }

        void findDirection(const std::vector<int>& direction) {
            std::vector<int> robot_to_wall(2, 0);

            if (direction[0] == 0 && direction[1] == 0 && direction[2] == 0 && direction[3] == 0) {
                robot_to_wall = {-1, 0};
            } 
            
            else {
                if (clockwise) {
                    if (direction[2] == 1) 
                        robot_to_wall = {1, -1};

                    if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {1, 0};

                    if (direction[3] == 1) 
                        robot_to_wall = {1, 1};

                    if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {0, 1};

                    if (direction[2] == -1) 
                        robot_to_wall = {-1, 1};

                    if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0)
                        robot_to_wall = {-1, 0};

                    if (direction[3] == -1 && direction[2] != 1) 
                        robot_to_wall = {-1, -1};

                    if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {0, -1};

                } 
                
                else {
                    if (direction[2] == 1) 
                        robot_to_wall = {-1, 1};

                    if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {0, 1};

                    if (direction[3] == -1)
                    robot_to_wall = {1, 1};

                    if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {1, 0};

                    if (direction[2] == -1) 
                        robot_to_wall = {1, -1};

                    if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {0, -1};

                    if (direction[3] == 1 && direction[2] != 1) 
                        robot_to_wall = {-1, -1};

                    if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0) 
                        robot_to_wall = {-1, 0};
                }
            }

            paths_queue.push(robot_to_wall);
        }

        void move() {
            if (!paths_queue.empty()) {
                std::vector<int> movement = paths_queue.front();
                paths_queue.pop();
                x += movement[0];
                y += movement[1];
            }
        }

        // function to save predicted grid
        void save_grid_csv() {
            std::string filename = "grid_pred.csv";
            std::ofstream file(filename);

            if (!file.is_open()) {
                std::cerr << "Error: Could not open file " << filename << std::endl;
                return;
            }

            // determine the maximum row size by finding the size of the longest inner vector
            size_t maxRowSize = 0;
            for (const auto& col : grid) {
                if (col.size() > maxRowSize) {
                    maxRowSize = col.size();
                }
            }

            // output the grid in transposed form (columns become rows in CSV)
            for (size_t row = 0; row < maxRowSize; ++row) {
                for (size_t col = 0; col < grid.size(); ++col) {
                    if (row < grid[col].size()) {
                        file << grid[col][row];
                    }
                    if (col < grid.size() - 1) {
                        file << ","; // Add comma except after the last element
                    }
                }
                file << "\n"; // New line after each row
            }

            file.close();
            std::cout << "Robot's grid written to " << filename << std::endl;
        }      
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//===== Main parameters =====
const int env_width {800}, env_height {800};        //Width and height of the environment
const int radius {10};                              //Radius of the robot's circular body
const int min_obj_size {50};                        //Maximum object dimension. Not required for lab 3/4
const int max_obj_size {100};                       //Maximum object dimension. Not required for lab 3/4
int lidar_range{50};                                //Lidar range, radiating from center of robot

// Grid utility class
grid_util grid(env_width, env_height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++DEFINE ANY GLOBAL VARIABLES/FUNCTIONS HERE+++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT AND WALLS==========

    // read config file
    std::tuple<std::string, bool, int, int> config = read_csv();

    // create the walls
    std::vector<Object*> walls;

    // normal perpendicular walls
    if (std::get<3>(config) == 4) {
        walls = grid.create_walls(std::get<0>(config));
    }
    // angled walls
    else {
        walls = grid.create_angled_walls(std::get<0>(config));
    }

    // get minimum/maximum y values for the robot to spawn
    int min_y_spawn = grid.get_min_y();
    int max_y_spawn = grid.get_max_y();

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv"); 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++DEFINE ANY LOCAL VARIABLES HERE+++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++Modify the instantiation of robot+++++++++++++++++++++++++++
//+++++robot should be a my_robot class instead of an Object class++++++++++++++++
//+++++++++++The constructor signature can be however you like++++++++++++++++++++
//+++Make sure to pass min_y_spawn and max_y_spawn to the constructor of Object+++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // 2*radius is used for width/height of robot
    Object robot(2*radius, 2*radius, env_width, min_y_spawn, max_y_spawn, radius+5);

    // create a copy. change this to a my_robot class as well
    Object robot_init = robot;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;
    // run the program indefinitely until robot hits the goal or an obstacle
    while (true)
    {
        limit_count++;
//+++++++++++++++WRITE YOUR MAIN LOOP CODE HERE++++++++++++++++++++++
//++++++++++++++EXAMPLE: ROBOT SIMPLY MOVES LEFT+++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        robot.sensor();

        robot.detect();
        robot.findDirection(robot.detect());
        robot.move();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
        robot_pos.push_back({robot.x, robot.y});

        if (limit_count>=7200) {
            std::cout << "====Program terminated after 3600 iterations====" << std::endl;
            break;
        }
    }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++For now, an 800x800 vector, vec, initialized to -1 is placed here+++++++++
//+Modify lines 180-181 so robot.grid is passed to both functions instead of vec++
//++++++Modify line 187 so the third argument to render_grid() is robot.grid++++++
//++++++After you make the robot instance in line 139 a my_robot class with 
//      a grid member, lines 182-183 will have robot.grid as its argument
//      and line 189 will have robot.grid as its third argument.++++++++++++++++++
//++++++++++++++++++++++++++Then, you can remove vec++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    std::vector<std::vector<int>> vec(800, std::vector<int>(800, -1));
    std::cout << std::fixed << std::setprecision(2);        // set precision for printing
    float wall_accuracy = grid.wall_accuracy(vec);          // for task 1: outer walls. replace vec with your robot's grid
    float accuracy = grid.grid_accuracy(vec);               // for task 2: entire environment inside walls. replace vec with your robot's grid
    std::cout << "Percent of walls correctly mapped: " << wall_accuracy*100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy*100.0 << "%" << std::endl;
    if (std::get<1>(config)){
        render_window(robot_pos, walls, robot_init, env_width, env_height, std::get<2>(config));
    }
    render_grid(robot_init, robot_pos, vec, env_width, env_height, radius, lidar_range, std::get<2>(config));
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    return 0;
}
