 #include "SearchGrid.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <utility>
#include <optional>

SearchGrid::SearchGrid()
    : d_yaw_rad((55.0 / 32.0)* (M_PI / 180.0)),
    d_pitch_rad((35.0 / 24.0)* (M_PI / 180.0)) {
    // Allocate exactly 5,000 cells
    thermal_map.resize(GRID_COLS * GRID_ROWS);
}

SearchGrid::Vector3D SearchGrid::rotateBodyToWorld(const Vector3D& v, double roll, double pitch, double yaw) const {
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw), sy = std::sin(yaw);

    double R11 = cp * cy, R12 = sr * sp * cy - cr * sy, R13 = cr * sp * cy + sr * sy;
    double R21 = cp * sy, R22 = sr * sp * sy + cr * cy, R23 = cr * sp * sy - sr * cy;
    double R31 = -sp, R32 = sr * cp, R33 = cr * cp;

    return {
        R11 * v.x + R12 * v.y + R13 * v.z,
        R21 * v.x + R22 * v.y + R23 * v.z,
        R31 * v.x + R32 * v.y + R33 * v.z
    };
}

bool SearchGrid::getGroundIntersection(double yaw_angle, double pitch_angle,
    double drone_x, double drone_y, double drone_z,
    double roll, double pitch, double yaw,
    double& hit_x, double& hit_y) const {

    Vector3D ray_body = { std::tan(pitch_angle), std::tan(yaw_angle), 1.0 };
    Vector3D ray_world = rotateBodyToWorld(ray_body, roll, pitch, yaw);

    if (ray_world.z <= 0.001) return false;

    double scale = (-drone_z) / ray_world.z;
    hit_x = drone_x + (ray_world.x * scale);
    hit_y = drone_y + (ray_world.y * scale);
    return true;
}

bool SearchGrid::processFrame(const std::array<double, 768>& thermal_frame, const Eigen::Matrix<double, 6, 1>& state) {
    
    // State was originally formatted as [x, y, z, roll, pitch, yaw]
    // Changed to reflect EKF state formatting
    double drone_x = state(3);
    double drone_y = state(4);
    double drone_z = state(5);
    double roll = state(0);
    double pitch = state(1);
    double yaw = state(2);

    bool frame_has_hotspot = false;

    for (int y = 0; y < 24; ++y) {
        for (int x = 0; x < 32; ++x) {
            double pixel_temp = thermal_frame[y * 32 + x];

            if (pixel_temp < 25.0) continue; // Low Pass filter
            
            // If there is a pixel that we detect, then allow blob finder to map
            if (pixel_temp >= (TEMP_THRESHOLD * 0.9)) {
                frame_has_hotspot = true;
            }

            double yaw_left = (x - 16.0) * d_yaw_rad;
            double yaw_right = (x - 15.0) * d_yaw_rad;
            double pitch_top = (y - 12.0) * d_pitch_rad;
            double pitch_bot = (y - 11.0) * d_pitch_rad;

            double hx[4], hy[4];
            bool valid = true;
            valid &= getGroundIntersection(yaw_left, pitch_top, drone_x, drone_y, drone_z, roll, pitch, yaw, hx[0], hy[0]);
            valid &= getGroundIntersection(yaw_right, pitch_top, drone_x, drone_y, drone_z, roll, pitch, yaw, hx[1], hy[1]);
            valid &= getGroundIntersection(yaw_right, pitch_bot, drone_x, drone_y, drone_z, roll, pitch, yaw, hx[2], hy[2]);
            valid &= getGroundIntersection(yaw_left, pitch_bot, drone_x, drone_y, drone_z, roll, pitch, yaw, hx[3], hy[3]);

            if (!valid) continue;

            double min_x = std::min({ hx[0], hx[1], hx[2], hx[3] });
            double max_x = std::max({ hx[0], hx[1], hx[2], hx[3] });
            double min_y = std::min({ hy[0], hy[1], hy[2], hy[3] });
            double max_y = std::max({ hy[0], hy[1], hy[2], hy[3] });

            // Removes cells that exist outside of the search grid
            if (max_x < -OFFSET_X || min_x > OFFSET_X || max_y < -OFFSET_Y || min_y > OFFSET_Y) {
                continue;
            }

            int col_start = std::max(0, std::min(static_cast<int>(std::floor((min_x + OFFSET_X) / GRID_RES)), GRID_COLS - 1));
            int col_end = std::max(0, std::min(static_cast<int>(std::floor((max_x + OFFSET_X) / GRID_RES)), GRID_COLS - 1));
            int row_start = std::max(0, std::min(static_cast<int>(std::floor((min_y + OFFSET_Y) / GRID_RES)), GRID_ROWS - 1));
            int row_end = std::max(0, std::min(static_cast<int>(std::floor((max_y + OFFSET_Y) / GRID_RES)), GRID_ROWS - 1));

            // Deposit temperatures
            for (int r = row_start; r <= row_end; ++r) {
                for (int c = col_start; c <= col_end; ++c) {
                    int idx = r * GRID_COLS + c;
                    thermal_map[idx].accumulated_temp += pixel_temp;
                    thermal_map[idx].visit_count++;
                }
            }
        }
    }

    if (frame_has_hotspot) {
        if (blob_finder()) {
            return true;
        }
    }

    return false;
}


bool SearchGrid::blob_finder() {
    std::vector<int> local_largest_blob;

    // Keep track of which cells the algorithm has already processed
    std::vector<bool> algorithm_visited(GRID_ROWS * GRID_COLS, false);

    // The 8 directional offsets for an 8-connected grid (row_offset, col_offset)
    int d_row[] = { -1, -1, -1,  0, 0,  1, 1, 1 };
    int d_col[] = { -1,  0,  1, -1, 1, -1, 0, 1 };

    // Scan every cell in the grid
    for (int r = 0; r < GRID_ROWS; ++r) {
        for (int c = 0; c < GRID_COLS; ++c) {
            int idx = r * GRID_COLS + c;

            // Start a search if we find a hot, scanned cell that hasn't been checked yet
            if (!algorithm_visited[idx] && thermal_map[idx].visit_count > 0 &&
                thermal_map[idx].getAverageTemp() >= TEMP_THRESHOLD) {
                // TEMP_THRESHOLD defined in Header file

                std::vector<int> current_blob;
                std::vector<int> stack;

                bool is_fully_bounded = true; // Assume true until it touches the unknown

                // Start the flood fill from this cell
                stack.push_back(idx);
                algorithm_visited[idx] = true;

                while (!stack.empty()) {
                    int curr_idx = stack.back();
                    stack.pop_back();

                    // Add this cell to our growing fire blob
                    current_blob.push_back(curr_idx);

                    // Convert 1D index back to 2D to check neighbors
                    int curr_r = curr_idx / GRID_COLS;
                    int curr_c = curr_idx % GRID_COLS;

                    // Check all 8 surrounding cells for the fire's border
                    for (int i = 0; i < 8; ++i) {
                        int neighbor_r = curr_r + d_row[i];
                        int neighbor_c = curr_c + d_col[i];

                        // Ensure the neighbor is actually inside the 5x10m grid area
                        if (neighbor_r >= 0 && neighbor_r < GRID_ROWS && neighbor_c >= 0 && neighbor_c < GRID_COLS) {
                            int neighbor_idx = neighbor_r * GRID_COLS + neighbor_c;

                            // Checks if camera has ever seen this cell. Ensures no positive ID for incomplete scan
                            if (thermal_map[neighbor_idx].visit_count == 0) {
                                is_fully_bounded = false;
                            }

                            // Checks if cell is part of the fire
                            else if (!algorithm_visited[neighbor_idx] &&
                                thermal_map[neighbor_idx].getAverageTemp() >= TEMP_THRESHOLD) {

                                algorithm_visited[neighbor_idx] = true; // Mark immediately to prevent duplicates
                                stack.push_back(neighbor_idx);
                            }
                        }
                    }
                }

                // ONLY accept the blob if it is fully enclosed by cold cells AND large enough
                if (is_fully_bounded && current_blob.size() >= 5) {
                    if (current_blob.size() > local_largest_blob.size()) {
                        local_largest_blob = std::move(current_blob);
                    }
                }
            }
        }
    }

    // If we found a valid fire, store it in the class member and return true
    if (!local_largest_blob.empty()) {
        target_blob = std::move(local_largest_blob);
        return true;
    }

    return false; // No fully bounded fires found this frame
}


std::optional<Eigen::Vector2d> SearchGrid::getCenter() const {
    if (target_blob.empty()) {
        return std::nullopt; // Safely returns "None"
    }

    double sum_x = 0.0;
    double sum_y = 0.0;

    for (int idx : target_blob) {
        int r = idx / GRID_COLS;
        int c = idx % GRID_COLS;

        double cell_x = (c * GRID_RES) - OFFSET_X + (GRID_RES / 2.0);
        double cell_y = (r * GRID_RES) - OFFSET_Y + (GRID_RES / 2.0);

        sum_x += cell_x;
        sum_y += cell_y;
    }

    double avg_x = sum_x / target_blob.size();
    double avg_y = sum_y / target_blob.size();

    return Eigen::Vector2d(avg_x, avg_y);
}