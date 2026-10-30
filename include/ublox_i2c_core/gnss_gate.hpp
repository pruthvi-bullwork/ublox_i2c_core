// Output-side quality / jump gate for GNSS fixes.
//
// Reconstructed from the ublox-i2c-core 1.3.0 arm64 build (symbols gnss::*).
#pragma once

#include <cmath>
#include <cstdint>

namespace gnss {

enum class Verdict {
  Accept = 0,
  ReAnchor,       // jump streak exceeded: accepted, discontinuity passed through
  FailOpen,       // threshold streak exceeded: accepted with inflated covariance
  FixNotOk,
  HAccOverLimit,
  PDopOverLimit,
  Jump,
};

// True for every verdict that results in the fix being published.
inline bool accepted(Verdict v)
{
  return v == Verdict::Accept || v == Verdict::ReAnchor || v == Verdict::FailOpen;
}

inline const char *verdict_str(Verdict v)
{
  switch (v) {
    case Verdict::Accept:        return "accept";
    case Verdict::ReAnchor:      return "re-anchor";
    case Verdict::FailOpen:      return "FAIL-OPEN (threshold streak)";
    case Verdict::FixNotOk:      return "gnssFixOk clear";
    case Verdict::HAccOverLimit: return "hAcc over limit";
    case Verdict::PDopOverLimit: return "pDOP over limit";
    case Verdict::Jump:          return "position jump";
  }
  return "unknown";
}

struct GateParams {
  bool enable = true;
  bool require_fix_ok = true;
  double max_hacc_m = 5.0;            // <= 0 disables
  double max_pdop = 4.0;              // <= 0 disables
  double max_jump_speed_mps = 15.0;
  double jump_margin_sigma = 3.0;     // margin = clamp(sigma * hAcc, min, max)
  double jump_margin_m = 0.5;
  double jump_margin_max_m = 5.0;
  int max_reject_streak = 25;
  double float_cov_scale = 4.0;
  double spp_cov_scale = 9.0;
  double fail_open_cov_scale = 25.0;
};

struct GateInput {
  double lat_deg;
  double lon_deg;
  double t_sec;
  bool fix_ok;
  double hacc_m;
  double pdop;
  uint8_t carr_soln;  // NAV-PVT carrSoln: 0 none, 1 float, 2 fixed
  uint8_t num_sv;
};

class Gate {
public:
  explicit Gate(const GateParams &p) : p_(p) {}

  void set_params(const GateParams &p) { p_ = p; }
  const GateParams &params() const { return p_; }

  Verdict check(const GateInput &in)
  {
    if (!p_.enable) {
      anchor(in);
      return Verdict::Accept;
    }

    if (p_.require_fix_ok && !in.fix_ok) {
      threshold_streak_ = 0;
      return reject(Verdict::FixNotOk);
    }

    const bool hacc_bad = p_.max_hacc_m > 0.0 && in.hacc_m > p_.max_hacc_m;
    const bool pdop_bad = p_.max_pdop > 0.0 && in.pdop > p_.max_pdop;
    if (hacc_bad || pdop_bad) {
      // A threshold that sits below this receiver's baseline would reject
      // every epoch forever. Fail open after a streak, loudly.
      ++threshold_streak_;
      if (p_.max_reject_streak > 0 && threshold_streak_ > p_.max_reject_streak) {
        anchor(in);
        jump_streak_ = 0;
        failing_open_ = true;
        ++rejected_;
        return Verdict::FailOpen;
      }
      return reject(hacc_bad ? Verdict::HAccOverLimit : Verdict::PDopOverLimit);
    }
    threshold_streak_ = 0;

    if (have_anchor_) {
      double dt = in.t_sec - anchor_t_;
      if (!(dt > 0.0 && dt < 60.0)) dt = 0.2;

      double margin = p_.jump_margin_sigma * in.hacc_m;
      if (margin < p_.jump_margin_m) margin = p_.jump_margin_m;
      if (margin > p_.jump_margin_max_m) margin = p_.jump_margin_max_m;
      last_margin_m_ = margin;

      const double allowed = margin + p_.max_jump_speed_mps * dt;
      last_jump_m_ = distance_m(anchor_lat_, anchor_lon_, in.lat_deg, in.lon_deg);
      if (allowed < last_jump_m_) {
        // Re-anchor after a streak so a genuine large correction can never
        // wedge the gate shut.
        if (++jump_streak_ > p_.max_reject_streak) {
          anchor(in);
          jump_streak_ = 0;
          ++rejected_;
          return Verdict::ReAnchor;
        }
        return reject(Verdict::Jump);
      }
    }

    anchor(in);
    jump_streak_ = 0;
    failing_open_ = false;
    return Verdict::Accept;
  }

  // Covariance multiplier for an accepted fix, by NAV-PVT carrSoln.
  double cov_scale(uint8_t carr_soln) const
  {
    double s = (carr_soln == 2) ? 1.0
             : (carr_soln == 1) ? p_.float_cov_scale
                                : p_.spp_cov_scale;
    if (failing_open_ && s < p_.fail_open_cov_scale) s = p_.fail_open_cov_scale;
    return s;
  }

  uint64_t rejected_total() const { return rejected_; }
  uint64_t accepted_total() const { return accepted_; }
  double last_jump_m() const { return last_jump_m_; }
  double last_margin_m() const { return last_margin_m_; }

private:
  Verdict reject(Verdict v)
  {
    ++rejected_;
    return v;
  }

  void anchor(const GateInput &in)
  {
    anchor_lat_ = in.lat_deg;
    anchor_lon_ = in.lon_deg;
    anchor_t_ = in.t_sec;
    have_anchor_ = true;
    ++accepted_;
  }

  // Equirectangular approximation; ample at the metre scale between epochs.
  static double distance_m(double lat1, double lon1, double lat2, double lon2)
  {
    constexpr double kMetresPerDeg = 111320.0;
    const double mean_lat = (lat1 + lat2) * 0.5 * M_PI / 180.0;
    const double dn = (lat2 - lat1) * kMetresPerDeg;
    const double de = (lon2 - lon1) * kMetresPerDeg * std::cos(mean_lat);
    return std::sqrt(dn * dn + de * de);
  }

  GateParams p_;
  bool have_anchor_ = false;
  double anchor_lat_ = 0.0;
  double anchor_lon_ = 0.0;
  double anchor_t_ = 0.0;
  int jump_streak_ = 0;
  int threshold_streak_ = 0;
  bool failing_open_ = false;
  double last_jump_m_ = 0.0;
  double last_margin_m_ = 0.0;
  uint64_t rejected_ = 0;
  uint64_t accepted_ = 0;
};

}  // namespace gnss
