#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>
#include <queue>   // ✅ Added missing include for std::queue

#include "utils.h"
#include "render.h"

#define PI 3.14159265358979323846

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++ my_robot class definition ++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
    // ✅ Properly forward parameters to Object constructor
    my_robot(int w, int h, int env_width, int y_min, int x_min, int tolerance)
        : Object(w, h, env_width, y_min, x_min, tolerance) {}

    // Store pointer to true grid for sensing
    void createGrid(grid_util& grid) {
        true_grid_ptr = &grid;
    }

    // Accessors
    std::vector<std::vector<int>> getGrid() {
        return this->grid;
    }

    grid_util& true_grid() {
        return *true_grid_ptr;
    }

    // Toggle direction
    void changeDirection() {
        clockwise = !clockwise;
    }

    bool findDirection() {   // ✅ Changed to return bool instead of void
        return clockwise;
    }

    // Sense environment and fill predicted grid
    void sensor() {
        if (!true_grid_ptr) {
            std::cerr << "Error: true_grid_ptr not initialized.\n";
            return;
        }

        int x_c = this->x + radius;
        int y_c = this->y + radius;

        for (int i = x_c - lidar_range; i <= x_c + lidar_range; ++i) {
            for (int j = y_c - lidar_range; j <= y_c + lidar_range; ++j) {
                if (i >= 0 && i < 800 && j >= 0 && j < 800) {
                    int dx = i - x_c;
                    int dy = j - y_c;
                    if ((dx * dx) + (dy * dy) <= (lidar_range * lidar_range)) {
                        grid[i][j] = Object::grid_value(*true_grid_ptr, this, i, j, lidar_range);
                    }
                }
            }
        }
    }

    // Detect nearby walls (returns 4-directional vector)
    std::vector<int> detect() {
        std::vector<int> direction(4, 0);
        int x_c = this->x + radius;
        int y_c = this->y + radius;
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
        } else {
            if (clockwise) {
                if (direction[2] == 1)
                    robot_to_wall = {1, -1};
                else if (direction[0] == 1)
                    robot_to_wall = {1, 0};
                else if (direction[3] == 1)
                    robot_to_wall = {1, 1};
                else if (direction[1] == 1)
                    robot_to_wall = {0, 1};
                else if (direction[2] == -1)
                    robot_to_wall = {-1, 1};
                else if (direction[0] == -1)
                    robot_to_wall = {-1, 0};
                else if (direction[3] == -1)
                    robot_to_wall = {-1, -1};
                else if (direction[1] == -1)
                    robot_to_wall = {0, -1};
            } else {
                if (direction[2] == 1)
                    robot_to_wall = {-1, 1};
                else if (direction[1] == -1)
                    robot_to_wall = {0, 1};
                else if (direction[3] == -1)
                    robot_to_wall = {1, 1};
                else if (direction[0] == -1)
                    robot_to_wall = {1, 0};
                else if (direction[2] == -1)
                    robot_to_wall = {1, -1};
                else if (direction[1] == 1)
                    robot_to_wall = {0, -1};
                else if (direction[3] == 1)
                    robot_to_wall = {-1, -1};
                else if (direction[0] == 1)
                    robot_to_wall = {-1, 0};
            }
        }

        paths_queue.push(robot_to_wall);
    }

    // Move the robot according to queued path
    void move() {
        if (!paths_queue.empty()) {
            std::vector<int> movement = paths_queue.front();
            paths_queue.pop();
            x += movement[0];
            y += movement[1];
        }
    }

    // Save predicted grid to CSV
    void save_grid_csv() {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        size_t maxRowSize = 0;
        for (const auto& col : grid) {
            if (col.size() > maxRowSize) {
                maxRowSize = col.size();
            }
        }

        for (size_t row = 0; row < maxRowSize; ++row) {
            for (size_t col = 0; col < grid.size(); ++col) {
                if (row < grid[col].size())
                    file << grid[col][row];
                if (col < grid.size() - 1)
                    file << ",";
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
        std::vector<int> dirs = robot.detect();
        robot.findDirection(dirs);
        robot.move(robot_to_wall);

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
