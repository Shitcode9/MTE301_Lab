#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>

#include "utils.h"
#include "render.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
class my_robot : public Object {
public:
    int tolerance = 20;
    int v = 0, h = 0;
    std::vector<std::vector<int>> grid;
    int lidar_range;

    my_robot(int width, int height, int env_width, int env_height, int lidar_range)
        : Object(width, height, env_width, env_height), lidar_range(lidar_range) {
        grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
    }

    // TASK 1

    void sensor(grid_util& g) {
        int x_c = this->x + this->width / 2;
        int y_c = this->y + this->height / 2;

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

        // function to save predicted grid to CSV
    void save_grid_csv() {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        // determine the maximum row size by finding the size of the longest inner vector
        size_t maxRowSize = 0;
        for (const auto &col : grid) {
            if (col.size() > maxRowSize) {
                maxRowSize = col.size();
            }
        }

        // output the grid in transposed form (columns become rows in CSV)
        for (size_t row = 0; row < maxRowSize; ++row) {
            for (size_t col = 0; col < grid.size(); ++col) {
                if (row < grid[col].size()) {
                    file << grid[col][row];
                } else {
                    file << -1; // unknown if out of range
                }
                if (col < grid.size() - 1) {
                    file << ",";
                }
            }
            file << "\n";
        }

        file.close();
        std::cout << "Robot's grid written to " << filename << std::endl;
    }

    // TASK 2

    void detect(int r) {
        int x_c = this->x + 10;
        int y_c = this->y + 10;

        if (y - tolerance >= 0 && this->grid[x][y - tolerance] == 1) 
        v = 1;

        else if (y + tolerance < 800 && this->grid[x][y + tolerance] == 1) 
        v = -1;

        else v = 0;

        if (x + tolerance < 800 && this->grid[x + tolerance][y] == 1) 
        h = 1;

        else if (x - tolerance >= 0 && this->grid[x - tolerance][y] == 1) 
        h = -1;

        else h = 0;
    }

    void move(int h, int v) {
        if (h == 1 && v == 1) 
        this->y += 1;

        else if (h == 1 && v == 0) 
        this->y += 1;

        else if (h == 1 && v == -1)
        this->x -= 1;

        else if (h == 0 && v == 1) 
        this->x += 1;

        else if (h == 0 && v == 0) 
        this->x -= 1;

        else if (h == 0 && v == -1) 
        this->x -= 1;

        else if (h == -1 && v == 1) 
        this->x += 1;

        else if (h == -1 && v == 0) 
        this->y -= 1;

        else if (h == -1 && v == -1) 
        this->y -= 1;
    }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//===== Main parameters =====
const int env_width {800}, env_height {800};        //Width and height of the environment
const int radius {10};                              //Radius of the robot's circular body
const int min_obj_size {50};                        //Maximum object dimension. Not required for lab 3
const int max_obj_size {100};                       //Maximum object dimension. Not required for lab 3
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
    std::pair<std::string, bool> config = read_csv();
    std::vector<Object*> walls;

    if (config.first == "environment1.csv") {
        walls = grid.create_walls(config.first);
    } else {
        walls = grid.create_angled_walls(config.first);
    }

    my_robot robot(2 * radius, 2 * radius, env_width, env_height, lidar_range);
    my_robot robot_init = robot;

    // push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++++++ WALL FOLLOW LOGIC +++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    while (true) {
        limit_count++;
        robot.sensor(grid);
        robot.detect(radius);
        robot.move(robot.h, robot.v);

        robot_pos.push_back({robot.x, robot.y});

        if (limit_count >= 3600) {
            std::cout << "==== wall follow done after 3600 ====" << std::endl;
            break;
        }
    }

    float accuracy = grid.grid_accuracy(robot.grid);
    std::cout << "Percent of walls correctly mapped: " << accuracy * 100.0 << "%" << std::endl;

    if (config.second) {
        render_window(robot_pos, walls, robot_init, env_width, env_height);
    }

    render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range);
    return 0;
}
