#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>
#include <queue>
#include "utils.h"
#include "render.h"

#define kPI 3.14159265358979323846

class my_robot : public Object {
private:
    int lidar_range{50};
    grid_util* true_grid_ptr = nullptr;
    std::vector<std::vector<int>> grid{800, std::vector<int>(800, -1)};
    std::queue<std::vector<int>> paths_queue;
    const float angle = std::cos(45 * kPI / 180);
    bool clockwise{true};

public:
    my_robot(int w, int h, int ew, int min_y, int min_x, int tolerance)
        : Object(w, h, ew, min_y, min_x, tolerance) {}

    void createGrid(grid_util& grid) {
        true_grid_ptr = &grid;
    }

    std::vector<std::vector<int>> getGrid() {
        return this->grid;
    }

    grid_util& true_grid() {
        return *true_grid_ptr;
    }

    void switch_dir() {
        clockwise = !clockwise;
    }

    bool get_dir() {
        return clockwise;
    }

    void sensor() {
        int x_c = x + 10;
        int y_c = y + 10;

        for (int i = std::max(0, x_c - lidar_range); i < std::min(800, x_c + lidar_range); i++) {
            for (int j = std::max(0, y_c - lidar_range); j < std::min(800, y_c + lidar_range); j++) {
                if ((i - x_c) * (i - x_c) + (j - y_c) * (j - y_c) <= lidar_range * lidar_range) {
                    grid[i][j] = grid_value(true_grid(), this, i, j, lidar_range);
                }
            }
        }
    }

    std::vector<int> detect() {
        std::vector<int> direction(4, 0);
        int x_c = x + 10;
        int y_c = y + 10;
        int tolerance = 20;
        const int diag_tolerance = tolerance * angle;

        if (grid[x_c][y_c - tolerance] == 1) 
            direction[0] = 1;
        else if (grid[x_c][y_c + tolerance] == 1) 
            direction[0] = -1;

        if (grid[x_c + tolerance][y_c] == 1) 
            direction[1] = 1;
        else if (grid[x_c - tolerance][y_c] == 1) 
            direction[1] = -1;

        if (grid[x_c - diag_tolerance][y_c - diag_tolerance] == 1) 
            direction[2] = 1;
        else if (grid[x_c + diag_tolerance][y_c + diag_tolerance] == 1) 
            direction[2] = -1;

        if (grid[x_c + diag_tolerance][y_c - diag_tolerance] == 1) 
            direction[3] = 1;
        else if (grid[x_c - diag_tolerance][y_c + diag_tolerance] == 1) 
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

    void save_grid_csv() {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        size_t maxRowSize = 0;
        for (const auto& col : grid) {
            if (col.size() > maxRowSize) maxRowSize = col.size();
        }

        for (size_t row = 0; row < maxRowSize; ++row) {
            for (size_t col = 0; col < grid.size(); ++col) {
                if (row < grid[col].size()) file << grid[col][row];
                if (col < grid.size() - 1) file << ",";
            }
            file << "\n";
        }

        file.close();
        std::cout << "Robot's grid written to " << filename << std::endl;
    }
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++ Main Program +++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

const int env_width{800}, env_height{800};
const int radius{10};
const int min_obj_size{50};
const int max_obj_size{100};
int lidar_range{50};

grid_util grid(env_width, env_height, min_obj_size, max_obj_size);
random_generator rand_gen;
std::vector<std::vector<int>> robot_pos;

int main(int argc, char const* argv[]) {
    std::tuple<std::string, bool, int, int> config = read_csv();
    std::vector<Object*> walls;

    if (std::get<3>(config) == 4)
        walls = grid.create_walls(std::get<0>(config));
    else
        walls = grid.create_angled_walls(std::get<0>(config));

    int min_y_spawn = grid.get_min_y();
    int max_y_spawn = grid.get_max_y();

    // Instantiate as my_robot now
    my_robot robot(2 * radius, 2 * radius, env_width, min_y_spawn, max_y_spawn, radius + 5);
    my_robot robot_init = robot;

    // Link robot to environment
    robot.createGrid(grid);

    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;

    // Main loop
    while (true) {
        limit_count++;

        robot.sensor();
        robot.findDirection(robot.detect());
        robot.move();

        robot_pos.push_back({robot.x, robot.y});

        if (limit_count >= 7200) {
            std::cout << "==== Program terminated after 7200 iterations ====" << std::endl;
            break;
        }
    }

    std::cout << std::fixed << std::setprecision(2);
    float wall_accuracy = grid.wall_accuracy(robot.getGrid());
    float accuracy = grid.grid_accuracy(robot.getGrid());
    std::cout << "Percent of walls correctly mapped: " << wall_accuracy * 100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy * 100.0 << "%" << std::endl;

    if (std::get<1>(config)) {
        render_window(robot_pos, walls, robot_init, env_width, env_height, std::get<2>(config));
    }

    render_grid(robot_init, robot_pos, robot.getGrid(), env_width, env_height, radius, lidar_range, std::get<2>(config));
    return 0;
}
