#pragma once

#include <vector>
#include <array>
#include <string>
#include <Eigen/Dense>
#include <optional>

// Represents a single 0.1m x 0.1m square of ground
struct GridCell {
    double accumulated_temp = 0.0;
    int visit_count = 0;

    double getAverageTemp() const {
        return (visit_count == 0) ? 0.0 : (accumulated_temp / visit_count);
    }
};

class SearchGrid {
public:
    SearchGrid();

    // Takes a raw 32x24 camera frame and the EKF state, projecting and mapping the data
    bool processFrame(const std::array<double, 768>& thermal_frame, const Eigen::Matrix<double, 6, 1>& state);

    // Scans the map for fully bounded fires. Returns true if a valid target is stored.
    bool blob_finder();

    std::optional<Eigen::Vector2d> getCenter() const;

    // Exports the current thermal map to a CSV file
    void exportToCSV(std::string filename) const;

private:
    // Map Configuration (5m East x 10m North to fully enclose the flight area)
    const double GRID_RES = 0.1;
    const int GRID_COLS = 50;              // East: 0.0 to 5.0
    const int GRID_ROWS = 100;             // North: 0.0 to 10.0

    // Detection Configuration
    const double TEMP_THRESHOLD = 35.0;

    // The actual memory buffer for the map
    std::vector<GridCell> thermal_map;

    // Stores the grid indices of the confirmed fire
    std::vector<int> target_blob;

    // Optical constants for the MLX90640
    const double d_yaw_rad;
    const double d_pitch_rad;

    // Math Helpers
    struct Vector3D { double x, y, z; };
    Vector3D rotateBodyToWorld(const Vector3D& v, double roll, double pitch, double yaw) const;
    
    bool getGroundIntersection(double yaw_angle, double pitch_angle,
        double drone_n, double drone_e, double drone_d,
        double roll, double pitch, double yaw,
        double& hit_n, double& hit_e) const;
};