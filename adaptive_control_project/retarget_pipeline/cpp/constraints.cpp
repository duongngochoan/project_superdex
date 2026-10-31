// Native trajectory constraint checks and geometric-path-preserving time scaling.
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local std::string constraint_error;

struct Results {
    std::vector<double> per; // margin, violation, |v|, |a|, v ratio, a ratio, scale, |effort|, effort ratio
    std::vector<int> counts; // position, velocity, acceleration, effort violations per joint
    double summary[13]{};
};

void validate(int rows, int joints, const double* q, const double* t,
              const double* lower, const double* upper, const double* vmax,
              const double* amax, double position_tol, double rate_tol) {
    if (rows < 2 || joints < 1 || joints > 10000 || !q || !t || !lower || !upper || !vmax || !amax)
        throw std::invalid_argument("expected at least two samples, positive joint count and complete buffers");
    if (!std::isfinite(position_tol) || position_tol < 0 || !std::isfinite(rate_tol) || rate_tol < 0)
        throw std::invalid_argument("constraint tolerances must be finite and nonnegative");
    for (int j = 0; j < joints; ++j)
        if (!std::isfinite(lower[j]) || !std::isfinite(upper[j]) || lower[j] >= upper[j] ||
            !std::isfinite(vmax[j]) || vmax[j] <= 0 || !std::isfinite(amax[j]) || amax[j] <= 0)
            throw std::invalid_argument("joint bounds and positive velocity/acceleration limits must be finite");
    for (int i = 0; i < rows; ++i) {
        if (!std::isfinite(t[i]) || (i && t[i] <= t[i - 1]))
            throw std::invalid_argument("timestamps must be finite and strictly increasing");
        for (int j = 0; j < joints; ++j)
            if (!std::isfinite(q[static_cast<size_t>(i) * joints + j]))
                throw std::invalid_argument("joint positions must be finite");
    }
}

Results audit(int rows, int joints, const double* q, const double* t,
              const double* lower, const double* upper, const double* vmax,
              const double* amax, const double* supplied_velocity,
              bool rest_ends, double position_tol, double rate_tol,
              const double* supplied_effort = nullptr, const double* effort_limits = nullptr) {
    validate(rows, joints, q, t, lower, upper, vmax, amax, position_tol, rate_tol);
    Results result;
    result.per.resize(static_cast<size_t>(joints) * 9, 0.0);
    result.counts.resize(static_cast<size_t>(joints) * 4, 0);
    result.summary[1] = std::numeric_limits<double>::infinity();
    result.summary[6] = 1.0;
    if (supplied_effort && !effort_limits)
        throw std::invalid_argument("measured effort requires effort limits");
    if (supplied_effort)
        for (int j = 0; j < joints; ++j)
            if (!std::isfinite(effort_limits[j]) || effort_limits[j] <= 0)
                throw std::invalid_argument("effort limits must be positive and finite");
    const int velocity_rows = supplied_velocity ? rows : rows - 1;
    std::vector<double> velocity(static_cast<size_t>(velocity_rows) * joints);
    for (int i = 0; i < velocity_rows; ++i)
        for (int j = 0; j < joints; ++j) {
            const size_t k = static_cast<size_t>(i) * joints + j;
            double value = supplied_velocity ? supplied_velocity[k] :
                (q[k + joints] - q[k]) / (t[i + 1] - t[i]);
            if (!std::isfinite(value)) throw std::invalid_argument("joint velocities must be finite");
            velocity[k] = value;
        }
    for (int j = 0; j < joints; ++j) {
        double* p = result.per.data() + static_cast<size_t>(j) * 9;
        int* c = result.counts.data() + static_cast<size_t>(j) * 4;
        p[0] = std::numeric_limits<double>::infinity();
        for (int i = 0; i < rows; ++i) {
            const double value = q[static_cast<size_t>(i) * joints + j];
            const double margin = std::min(value - lower[j], upper[j] - value);
            p[0] = std::min(p[0], margin);
            p[1] = std::max(p[1], -margin);
            c[0] += margin < -position_tol;
            if (supplied_effort) {
                const double value = std::abs(supplied_effort[static_cast<size_t>(i) * joints + j]);
                if (!std::isfinite(value)) throw std::invalid_argument("joint efforts must be finite");
                p[7] = std::max(p[7], value);
                c[3] += value > effort_limits[j] * (1.0 + rate_tol);
            }
        }
        for (int i = 0; i < velocity_rows; ++i) {
            const double value = std::abs(velocity[static_cast<size_t>(i) * joints + j]);
            p[2] = std::max(p[2], value);
            c[1] += value > vmax[j] * (1.0 + rate_tol);
        }
        auto acceleration = [&](double value) {
            value = std::abs(value);
            p[3] = std::max(p[3], value);
            c[2] += value > amax[j] * (1.0 + rate_tol);
        };
        for (int i = 0; i < velocity_rows - 1; ++i) {
            const double dt = supplied_velocity ? t[i + 1] - t[i] : 0.5 * (t[i + 2] - t[i]);
            acceleration((velocity[static_cast<size_t>(i + 1) * joints + j] -
                          velocity[static_cast<size_t>(i) * joints + j]) / dt);
        }
        if (rest_ends) {
            acceleration(2.0 * velocity[j] / (t[1] - t[0]));
            acceleration(2.0 * velocity[static_cast<size_t>(velocity_rows - 1) * joints + j] /
                         (t[rows - 1] - t[rows - 2]));
        }
        p[4] = p[2] / vmax[j];
        p[5] = p[3] / amax[j];
        p[6] = std::max({1.0, p[4], std::sqrt(p[5])});
        p[8] = supplied_effort ? p[7] / effort_limits[j] : 0.;
        result.summary[10] = std::max(result.summary[10], p[8]);
        result.summary[11] = std::max(result.summary[11], p[7]);
        result.summary[12] += c[3];
        result.summary[0] = std::max(result.summary[0], p[1]);
        result.summary[1] = std::min(result.summary[1], p[0]);
        result.summary[2] = std::max(result.summary[2], p[4]);
        result.summary[3] = std::max(result.summary[3], p[5]);
        result.summary[4] = std::max(result.summary[4], p[2]);
        result.summary[5] = std::max(result.summary[5], p[3]);
        result.summary[6] = std::max(result.summary[6], p[6]);
        for (int k = 0; k < 3; ++k) result.summary[7 + k] += c[k];
    }
    return result;
}

template<class F> int guarded(F&& function) {
    try { function(); constraint_error.clear(); return 0; }
    catch (const std::exception& error) { constraint_error = error.what(); return -1; }
}
}

extern "C" {
const char* ar_constraints_last_error() { return constraint_error.c_str(); }

int ar_constraints_audit(int rows, int joints, const double* q, const double* t,
                         const double* lower, const double* upper, const double* vmax,
                         const double* amax, const double* velocity, const double* effort, const double* effort_limits, int rest_ends,
                         double position_tol, double rate_tol, double* summary,
                         double* per_joint, int* counts) {
    return guarded([&] {
        if (!summary || !per_joint || !counts) throw std::invalid_argument("missing audit output buffers");
        const Results result = audit(rows, joints, q, t, lower, upper, vmax, amax, velocity,
                                     rest_ends != 0, position_tol, rate_tol, effort, effort_limits);
        std::copy(result.summary, result.summary + 13, summary);
        std::copy(result.per.begin(), result.per.end(), per_joint);
        std::copy(result.counts.begin(), result.counts.end(), counts);
    });
}

int ar_constraints_retime(int rows, int joints, const double* q, const double* t,
                          const double* lower, const double* upper, const double* vmax,
                          const double* amax, int rest_ends, double position_tol,
                          double safety_factor, double* output_times, double* scale) {
    return guarded([&] {
        if (!output_times || !scale || !std::isfinite(safety_factor) || safety_factor < 1.0)
            throw std::invalid_argument("time scale safety factor must be >= 1 and outputs must exist");
        const Results result = audit(rows, joints, q, t, lower, upper, vmax, amax, nullptr,
                                     rest_ends != 0, position_tol, 1e-9);
        if (result.summary[7] != 0)
            throw std::invalid_argument("retiming cannot repair position-limit violations; project or re-plan first");
        *scale = result.summary[6] > 1.0 ? result.summary[6] * safety_factor : 1.0;
        for (int i = 0; i < rows; ++i)
            output_times[i] = t[0] + (t[i] - t[0]) * *scale;
    });
}

int ar_constraints_project(int rows, int joints, const double* input, const double* lower,
                           const double* upper, const double* margin, double* output) {
    return guarded([&] {
        if (rows < 1 || joints < 1 || !input || !lower || !upper || !margin || !output)
            throw std::invalid_argument("missing position projection buffers");
        for (int j = 0; j < joints; ++j)
            if (!std::isfinite(lower[j]) || !std::isfinite(upper[j]) ||
                !std::isfinite(margin[j]) || margin[j] < 0 || lower[j] + margin[j] > upper[j] - margin[j])
                throw std::invalid_argument("invalid joint limits or position margins");
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < joints; ++j) {
                const size_t k = static_cast<size_t>(i) * joints + j;
                if (!std::isfinite(input[k])) throw std::invalid_argument("joint positions must be finite");
                output[k] = std::clamp(input[k], lower[j] + margin[j], upper[j] - margin[j]);
            }
    });
}
}

namespace {
struct PreparedPath {
    std::vector<double> values;
    std::vector<double> slopes;
    double time_scale = 1.;
    double projection_change = 0.;
    double smoothing_change = 0.;
    double peak_velocity = 0.;
    double peak_acceleration = 0.;
    double velocity_ratio = 0.;
    double acceleration_ratio = 0.;
};

PreparedPath prepare_path(int rows, int joints, const double* input, const double* t,
                         const double* lower, const double* upper, const double* vmax,
                         const double* amax, const double* margin, double sigma,
                         double safety_factor) {
    validate(rows, joints, input, t, lower, upper, vmax, amax, 0., 0.);
    if (!margin || !std::isfinite(sigma) || sigma < 0 ||
        !std::isfinite(safety_factor) || safety_factor < 1.)
        throw std::invalid_argument("invalid trajectory preparation settings");
    PreparedPath path;
    const size_t size = static_cast<size_t>(rows) * joints;
    path.values.resize(size);
    path.slopes.resize(size, 0.);
    for (int j = 0; j < joints; ++j)
        if (!std::isfinite(margin[j]) || margin[j] < 0 || lower[j] + margin[j] > upper[j] - margin[j])
            throw std::invalid_argument("position margin cannot fit inside joint bounds");
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < joints; ++j) {
            const size_t k = static_cast<size_t>(i) * joints + j;
            path.values[k] = std::clamp(input[k], lower[j] + margin[j], upper[j] - margin[j]);
            path.projection_change = std::max(path.projection_change, std::abs(path.values[k] - input[k]));
        }
    if (sigma > 0) {
        const auto projected = path.values;
        int begin = 0, end = 0;
        std::vector<double> weights(rows);
        for (int k = 0; k < rows; ++k)
            weights[k] = k == 0 ? t[1] - t[0] :
                         k == rows - 1 ? t[rows - 1] - t[rows - 2] : 0.5 * (t[k + 1] - t[k - 1]);
        for (int i = 1; i < rows - 1; ++i) {
            while (begin < rows - 1 && t[begin] < t[i] - 4. * sigma) ++begin;
            while (end < rows && t[end] <= t[i] + 4. * sigma) ++end;
            double denominator = 0.;
            std::fill(path.values.begin() + static_cast<size_t>(i) * joints,
                      path.values.begin() + static_cast<size_t>(i + 1) * joints, 0.);
            for (int k = begin; k < end; ++k) {
                const double delta = (t[k] - t[i]) / sigma;
                const double weight = std::exp(-0.5 * delta * delta) * weights[k];
                denominator += weight;
                for (int j = 0; j < joints; ++j)
                    path.values[static_cast<size_t>(i) * joints + j] +=
                        weight * projected[static_cast<size_t>(k) * joints + j];
            }
            if (!(denominator > 0)) throw std::runtime_error("empty Gaussian trajectory kernel");
            for (int j = 0; j < joints; ++j) {
                const size_t k = static_cast<size_t>(i) * joints + j;
                // Fade smoothing at fixed endpoints to avoid introducing an
                // artificial first-sample jump into an otherwise resting path.
                const double r = std::min({1., (t[i] - t[0]) / (4. * sigma),
                                          (t[rows - 1] - t[i]) / (4. * sigma)});
                const double blend = r * r * r * (10. + r * (-15. + 6. * r));
                path.values[k] = projected[k] + blend * (path.values[k] / denominator - projected[k]);
                path.smoothing_change = std::max(path.smoothing_change, std::abs(path.values[k] - projected[k]));
            }
        }
    }
    // Fritsch-Carlson/PCHIP weighted harmonic slopes preserve each scalar bound.
    // Both ends are deliberately at rest for physical replay.
    for (int i = 1; i < rows - 1; ++i) {
        const double left_h = t[i] - t[i - 1], right_h = t[i + 1] - t[i];
        const double w1 = 2. * right_h + left_h, w2 = right_h + 2. * left_h;
        for (int j = 0; j < joints; ++j) {
            const size_t k = static_cast<size_t>(i) * joints + j;
            const double left = (path.values[k] - path.values[k - joints]) / left_h;
            const double right = (path.values[k + joints] - path.values[k]) / right_h;
            if (left * right > 0.) path.slopes[k] = (w1 + w2) / (w1 / left + w2 / right);
        }
    }
    for (int i = 0; i < rows - 1; ++i) {
        const double h = t[i + 1] - t[i];
        for (int j = 0; j < joints; ++j) {
            const size_t k = static_cast<size_t>(i) * joints + j;
            const double p0 = path.values[k], p1 = path.values[k + joints];
            const double m0 = path.slopes[k], m1 = path.slopes[k + joints];
            const double a = 2. * p0 - 2. * p1 + h * (m0 + m1);
            const double b = -3. * p0 + 3. * p1 - h * (2. * m0 + m1);
            const double c = h * m0;
            auto speed = [&](double s) { return std::abs((3. * a * s * s + 2. * b * s + c) / h); };
            double v = std::max(speed(0.), speed(1.));
            if (a != 0.) {
                const double critical = -b / (3. * a);
                if (critical > 0. && critical < 1.) v = std::max(v, speed(critical));
            }
            const double acceleration = std::max(std::abs(2. * b), std::abs(6. * a + 2. * b)) / (h * h);
            path.peak_velocity = std::max(path.peak_velocity, v);
            path.peak_acceleration = std::max(path.peak_acceleration, acceleration);
            path.velocity_ratio = std::max(path.velocity_ratio, v / vmax[j]);
            path.acceleration_ratio = std::max(path.acceleration_ratio, acceleration / amax[j]);
        }
    }
    const double required = std::max({1., path.velocity_ratio, std::sqrt(path.acceleration_ratio)});
    path.time_scale = required > 1. ? required * safety_factor : 1.;
    return path;
}
}

extern "C" int ar_constraints_prepare(int rows, int joints, const double* input, const double* t,
                                      const double* lower, const double* upper, const double* vmax,
                                      const double* amax, const double* margin, double sigma,
                                      double dt, double safety_factor, int capacity, int* count,
                                      double* output_times, double* output_positions, double* metadata) {
    return guarded([&] {
        if (!count || !metadata || !std::isfinite(dt) || dt <= 0 || capacity < 0)
            throw std::invalid_argument("invalid uniform sampling period or output buffers");
        const auto path = prepare_path(rows, joints, input, t, lower, upper, vmax, amax, margin, sigma, safety_factor);
        const double exact_duration = (t[rows - 1] - t[0]) * path.time_scale;
        const double last_index = std::ceil(exact_duration / dt - 1e-10);
        if (!std::isfinite(last_index) || last_index < 1. || last_index > 5000000.)
            throw std::invalid_argument("retimed trajectory exceeds supported sample count");
        *count = static_cast<int>(last_index) + 1;
        metadata[0] = path.time_scale;
        metadata[1] = path.projection_change;
        metadata[2] = path.smoothing_change;
        metadata[3] = path.peak_velocity / path.time_scale;
        metadata[4] = path.peak_acceleration / (path.time_scale * path.time_scale);
        metadata[5] = path.velocity_ratio / path.time_scale;
        metadata[6] = path.acceleration_ratio / (path.time_scale * path.time_scale);
        metadata[7] = exact_duration;
        if (capacity == 0) return;
        if (capacity < *count || !output_times || !output_positions)
            throw std::invalid_argument("uniform trajectory output capacity too small");
        int segment = 0;
        for (int i = 0; i < *count; ++i) {
            output_times[i] = i * dt;
            const double source_time = t[0] + output_times[i] / path.time_scale;
            if (source_time >= t[rows - 1]) {
                std::copy(path.values.end() - joints, path.values.end(), output_positions + static_cast<size_t>(i) * joints);
                continue;
            }
            while (segment < rows - 2 && source_time > t[segment + 1]) ++segment;
            const double h = t[segment + 1] - t[segment];
            const double s = std::clamp((source_time - t[segment]) / h, 0., 1.);
            for (int j = 0; j < joints; ++j) {
                const size_t k = static_cast<size_t>(segment) * joints + j;
                const double p0 = path.values[k], p1 = path.values[k + joints];
                const double m0 = path.slopes[k], m1 = path.slopes[k + joints];
                const double a = 2. * p0 - 2. * p1 + h * (m0 + m1);
                const double b = -3. * p0 + 3. * p1 - h * (2. * m0 + m1);
                output_positions[static_cast<size_t>(i) * joints + j] = ((a * s + b) * s + h * m0) * s + p0;
            }
        }
    });
}
