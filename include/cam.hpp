#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

enum moveType {
    SIN_ACCELERATION,
    COS_ACCELERATION,
    LINEAR
};

struct CamResults {
    std::vector<double> phi_range;
    std::vector<double> s_points;
    std::vector<double> v_points;
    std::vector<double> a_points;
    std::vector<double> ds_dphi_points;
    std::vector<double> alpha_points;
    std::vector<double> rho_points;
    std::vector<double> x_points;
    std::vector<double> y_points;
    std::vector<double> dx_points;
    std::vector<double> dy_points;
    std::vector<double> x_1_points;
    std::vector<double> y_1_points;
};

class Cam {
private:
    double h;
    double phi_1;
    double alpha_1;
    double phi_2;
    double alpha_2;
    double phi_s1;
    double phi_s2;
    double omega_1;
    double e;
    double r_0;
    moveType rise_move_type;
    moveType return_move_type;

    double deg2rad(double deg) const {
        return deg * M_PI / 180.0;
    }

public:
    Cam(double h, double phi_1, double alpha_1, double phi_2, double alpha_2,
        double phi_s1, double phi_s2, double omega_1, double e = 10.0,
        double r_0 = 10.0, moveType rise_move_type = moveType::SIN_ACCELERATION,
        moveType return_move_type = moveType::COS_ACCELERATION)
        : h(h), phi_1(phi_1), alpha_1(alpha_1), phi_2(phi_2), alpha_2(alpha_2),
          phi_s1(phi_s1), phi_s2(phi_s2), omega_1(omega_1), e(e), r_0(r_0),
          rise_move_type(rise_move_type), return_move_type(return_move_type) {}

    CamResults calculate() {
        CamResults results;
        // Generate phi range (0-360 degrees)
        for (int deg = 0; deg <= 360; ++deg) {
            results.phi_range.push_back(static_cast<double>(deg));
        }
        size_t num_points = results.phi_range.size();

        // Initialize all vectors
        results.s_points.resize(num_points, 0.0);
        results.v_points.resize(num_points, 0.0);
        results.a_points.resize(num_points, 0.0);
        results.ds_dphi_points.resize(num_points, 0.0);
        results.alpha_points.resize(num_points, 0.0);
        results.rho_points.resize(num_points, 0.0);
        results.x_points.resize(num_points, 0.0);
        results.y_points.resize(num_points, 0.0);
        results.dx_points.resize(num_points, 0.0);
        results.dy_points.resize(num_points, 0.0);
        results.x_1_points.resize(num_points, 0.0);
        results.y_1_points.resize(num_points, 0.0);

        std::vector<double> r_0_candidates(num_points, 0.0);

        // First pass: Calculate motion parameters and r0 candidates
        for (size_t idx = 0; idx < num_points; ++idx) {
            double phi = results.phi_range[idx];
            double& s = results.s_points[idx];
            double& v = results.v_points[idx];
            double& a = results.a_points[idx];
            double& ds_dphi = results.ds_dphi_points[idx];
            double& alpha = results.alpha_points[idx];

            // Motion type calculations
            if (phi <= phi_1) { // Rise phase
                if (rise_move_type == moveType::SIN_ACCELERATION) {
                    s = h * (phi/phi_1 - std::sin(2*M_PI*phi/phi_1)/(2*M_PI));
                    v = h*omega_1/phi_1 * (1 - std::cos(2*M_PI*phi/phi_1));
                    a = 2*M_PI*h*omega_1*omega_1/(phi_1*phi_1) * std::sin(2*M_PI*phi/phi_1);
                } 
                else if (rise_move_type == moveType::COS_ACCELERATION) {
                    s = h/2 * (1 - std::cos(M_PI*phi/phi_1));
                    v = M_PI*h*omega_1/(2*phi_1) * std::sin(M_PI*phi/phi_1);
                    a = M_PI*M_PI*h*omega_1*omega_1/(2*phi_1*phi_1) * std::cos(M_PI*phi/phi_1);
                }
                alpha = alpha_1;
            }
            else if (phi > phi_1 && phi < phi_1 + phi_s1) { // Dwell 1
                s = h;
                v = 0.0;
                a = 0.0;
                alpha = alpha_1;
            }
            else if (phi >= phi_1 + phi_s1 && phi <= phi_1 + phi_s1 + phi_2) { // Return phase
                double T = phi - (phi_1 + phi_s1);
                if (return_move_type == moveType::SIN_ACCELERATION) {
                    s = h * (1 - T/phi_2 + std::sin(2*M_PI*T/phi_2)/(2*M_PI));
                    v = -h*omega_1/phi_2 * (1 - std::cos(2*M_PI*T/phi_2));
                    a = -2*M_PI*h*omega_1*omega_1/(phi_2*phi_2) * std::sin(2*M_PI*T/phi_2);
                }
                else if (return_move_type == moveType::COS_ACCELERATION) {
                    s = h/2 * (1 + std::cos(M_PI*T/phi_2));
                    v = -M_PI*h*omega_1/(2*phi_2) * std::sin(M_PI*T/phi_2);
                    a = -M_PI*M_PI*h*omega_1*omega_1/(2*phi_2*phi_2) * std::cos(M_PI*T/phi_2);
                }
                alpha = alpha_2;
            }
            else { // Dwell 2 or invalid range
                s = 0.0;
                v = 0.0;
                a = 0.0;
                alpha = alpha_2;
            }

            ds_dphi = v / omega_1;

            // Calculate r0 candidate
            double temp_r0 = e + 0.1;
            while (true) {
                double s0_temp = std::sqrt(temp_r0*temp_r0 - e*e);
                double compare = std::abs(ds_dphi - e)/(s0_temp + s) - std::tan(deg2rad(alpha));
                if (compare < 0.0) break;
                temp_r0 += 0.1;
            }
            r_0_candidates[idx] = temp_r0;
        }

        // Determine final r0
        r_0 = *std::max_element(r_0_candidates.begin(), r_0_candidates.end());
        double s0 = std::sqrt(r_0*r_0 - e*e);

        // Second pass: Calculate geometry parameters
        for (size_t idx = 0; idx < num_points; ++idx) {
            double phi = results.phi_range[idx];
            double s = results.s_points[idx];
            double ds_dphi = results.ds_dphi_points[idx];
            double dds_ddphi = results.a_points[idx] / omega_1/omega_1;

            // Recalculate pressure angle
            double alpha_rad = std::atan(std::abs(ds_dphi - e)/(s0 + s));
            results.alpha_points[idx] = alpha_rad * 180.0 / M_PI;

            // Calculate coordinates
            double phi_rad = deg2rad(phi);
            results.x_points[idx] = -(s0 + s)*std::sin(phi_rad) - e*std::cos(phi_rad);
            results.y_points[idx] = (s0 + s)*std::cos(phi_rad) - e*std::sin(phi_rad);

            // Calculate derivatives
            results.dx_points[idx] = -(s0 + s)*std::cos(phi_rad) - (ds_dphi - e)*std::sin(phi_rad);
            results.dy_points[idx] = -(s0 + s)*std::sin(phi_rad) + (ds_dphi - e)*std::cos(phi_rad);

            // Calculate curvature
            double ddx = -(2*ds_dphi - e)*std::cos(phi_rad) - (dds_ddphi - s0 - s)*std::sin(phi_rad);
            double ddy = (dds_ddphi - s0 - s)*std::cos(phi_rad) - (2*ds_dphi - e)*std::sin(phi_rad);
            double numerator = std::pow(results.dx_points[idx]*results.dx_points[idx] + 
                                      results.dy_points[idx]*results.dy_points[idx], 1.5);
            double denominator = results.dx_points[idx]*ddy - results.dy_points[idx]*ddx;
            results.rho_points[idx] = numerator / denominator;
        }

        // Calculate roller profile
        double rho_min = *std::min_element(results.rho_points.begin(), results.rho_points.end());
        double r_r = std::floor(rho_min / 2.0);

        for (size_t idx = 0; idx < num_points; ++idx) {
            double dx = results.dx_points[idx];
            double dy = results.dy_points[idx];
            double norm = std::hypot(dx, dy);
            if (norm == 0) throw std::runtime_error("Zero derivative in roller calculation");
            
            results.x_1_points[idx] = results.x_points[idx] + r_r * dy / norm;
            results.y_1_points[idx] = results.y_points[idx] - r_r * dx / norm;
        }

        return results;
    }
};