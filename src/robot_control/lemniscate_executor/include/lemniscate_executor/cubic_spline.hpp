#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

namespace lemniscate_executor
{

/// Coefficients of one cubic segment.
/// q(u) = a + b*u + c*u^2 + d*u^3,  u in [0, h]
struct SplineSegment
{
  double a, b, c, d;  ///< polynomial coefficients
  double h;           ///< segment duration [nominal seconds]
};

/// Header-only periodic cubic spline (1D, joint-space).
///
/// Computes a C^2 periodic cubic spline through N via-points.  The curve is
/// parameterised by cumulative **nominal time** (h_i = nominal duration of
/// segment i at speed_scale = 1.0).
///
/// Periodicity conditions:  q'(0) = q'(T),  q''(0) = q''(T)
/// - enforced via a cyclic tridiagonal NxN linear system solved with Eigen.
/// For N ~ 200 this takes < 1 ms.
class CubicSpline
{
public:
  /// Fit a periodic cubic spline.
  ///
  /// @param via_points  N+1 values; via_points[N] must equal via_points[0]
  ///                    (caller ensures periodic closure).
  /// @param durations   N positive segment durations h_i [s].
  /// @return            N SplineSegments ready for evaluate().
  static std::vector<SplineSegment> fit(
    const std::vector<double> & via_points,
    const std::vector<double> & durations)
  {
    const int N = static_cast<int>(durations.size());
    if (static_cast<int>(via_points.size()) != N + 1) {
      throw std::invalid_argument(
        "CubicSpline::fit: via_points.size() must equal durations.size() + 1");
    }
    if (N < 3) {
      throw std::invalid_argument(
        "CubicSpline::fit: need at least 3 segments for a periodic spline");
    }
    // Caller must supply a closed via_points vector: via_points[N] == via_points[0].
    if (std::abs(via_points.front() - via_points.back()) > 1e-9) {
      throw std::invalid_argument(
        "CubicSpline::fit: via_points must be periodically closed "
        "(via_points[0] must equal via_points[N])");
    }

    // Wrap any integer index (including negative) into [0, N).
    const auto mod_idx = [N](int i) {return ((i % N) + N) % N;};

    // Modular accessors - delegate wrapping to mod_idx for clarity.
    const auto q = [&](int i) -> double {return via_points[mod_idx(i)];};
    const auto h = [&](int i) -> double {return durations[mod_idx(i)];};

    // Build cyclic tridiagonal system for second derivatives M[i]
    //
    // Equation at row i (C^2 continuity at via-point i):
    //   h[i-1]*M[i-1]  +  2*(h[i-1]+h[i])*M[i]  +  h[i]*M[i+1]  =  6*delta[i]
    //
    //   delta[i] = (q[i+1]-q[i])/h[i]  -  (q[i]-q[i-1])/h[i-1]
    //
    // All indices modulo N (periodic).  The corner entries A(0,N-1) and
    // A(N-1,0) make this cyclic.

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, N);
    Eigen::VectorXd rhs(N);

    for (int i = 0; i < N; ++i) {
      const int prev = (i - 1 + N) % N;
      const int next = (i + 1) % N;

      A(i, prev) += h(prev);
      A(i, i)   += 2.0 * (h(prev) + h(i));
      A(i, next) += h(i);

      rhs(i) = 6.0 * ((q(i + 1) - q(i)) / h(i) - (q(i) - q(i - 1)) / h(prev));
    }

    const Eigen::VectorXd M = A.lu().solve(rhs);

    // Compute per-segment polynomial coefficients
    std::vector<SplineSegment> segments(N);
    for (int i = 0; i < N; ++i) {
      const int  next = (i + 1) % N;
      const double hi  = durations[i];
      const double Mi  = M(i);
      const double Mn  = M(next);
      const double qi  = via_points[i];
      const double qn  = via_points[i + 1];   // via_points has N+1 entries

      segments[i].h = hi;
      segments[i].a = qi;
      segments[i].b = (qn - qi) / hi - hi * (2.0 * Mi + Mn) / 6.0;
      segments[i].c = Mi / 2.0;
      segments[i].d = (Mn - Mi) / (6.0 * hi);
    }

    return segments;
  }

  /// Evaluate spline position, nominal velocity, and nominal acceleration
  /// at cumulative nominal time @p t.
  ///
  /// @param segments  Segment vector returned by fit().
  /// @param t         Cumulative nominal time in [0, sum(h_i)).
  ///                  The caller is responsible for wrapping into this range.
  /// @param[out] pos  Position q(t).
  /// @param[out] vel  dq/dt at nominal parameterisation.
  /// @param[out] acc  d^2q/dt^2 at nominal parameterisation.
  ///
  /// @note  When replaying at a different speed_scale s:
  ///        actual_vel = vel * s,  actual_acc = acc * s^2,
  ///        actual_time_from_start = t_nominal / s.
  static void evaluate(
    const std::vector<SplineSegment> & segments,
    double t,
    double & pos, double & vel, double & acc)
  {
    pos = vel = acc = 0.0;  // safe default if segments is empty (satisfies -Wmaybe-uninitialized)
    t = std::max(t, 0.0);  // guard against tiny negative values from fmod
    double t_start = 0.0;
    for (std::size_t i = 0; i < segments.size(); ++i) {
      const double t_end = t_start + segments[i].h;
      if (t < t_end || i == segments.size() - 1) {
        const double u = t - t_start;
        const double & a = segments[i].a;
        const double & b = segments[i].b;
        const double & c = segments[i].c;
        const double & d = segments[i].d;
        pos = a + u * (b + u * (c + u * d));
        vel = b + u * (2.0 * c + 3.0 * u * d);
        acc = 2.0 * c + 6.0 * u * d;
        return;
      }
      t_start = t_end;
    }
  }
};

}  // namespace lemniscate_executor
