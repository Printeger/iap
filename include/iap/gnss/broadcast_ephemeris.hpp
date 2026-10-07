#pragma once

#include <Eigen/Geometry>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/rinex_helper.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace iap::broadcast {

// Shared simulator/front-end model seam. The ordinary ECEF orbit already
// includes Rz(-omega*tk). Undo it, tilt the GEO frame, then rotate to ECEF.
// Mathematical composition, independently compared with RTKLIB 2.4.3
// ephemeris.c at 180043ee24b6d2b168f98b64be15f69d50046b1a (not copied code).
inline Eigen::Vector3d position(
    gnss_comm::gtime_t time, const gnss_comm::EphemPtr& eph, double* clock = nullptr) {
  if (!eph) throw std::invalid_argument("missing broadcast ephemeris");
  Eigen::Vector3d p = gnss_comm::eph2pos(time, eph, clock);
  uint32_t prn = 0;
  if (gnss_comm::satsys(eph->sat, &prn) == SYS_BDS && prn >= 59 && prn <= 63) {
    double tk = gnss_comm::time_diff(time, eph->toe);
    if (tk > 302400) tk -= 604800;
    if (tk < -302400) tk += 604800;
    const double angle = 7.292115e-5 * tk;
    p = Eigen::AngleAxisd(-angle, Eigen::Vector3d::UnitZ()) *
        (Eigen::AngleAxisd(5.0 * std::acos(-1.0) / 180.0, Eigen::Vector3d::UnitX()) *
         (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) * p));
  }
  return p;
}

inline Eigen::Vector3d velocity(
    gnss_comm::gtime_t time, const gnss_comm::EphemPtr& eph, double* clock_rate = nullptr) {
  // Differentiate the same position model used by pseudorange. This also
  // avoids the upstream analytic vertical derivative's extra i_dot factor.
  constexpr double h = .01;
  double minus_clock = 0, plus_clock = 0;
  const auto minus = position(gnss_comm::time_add(time, -h), eph, &minus_clock);
  const auto plus = position(gnss_comm::time_add(time, h), eph, &plus_clock);
  if (clock_rate) *clock_rate = (plus_clock - minus_clock) / (2 * h);
  return (plus - minus) / (2 * h);
}

// Health, original TOE age and actual broadcast availability are separate
// qualifications. A future TOE may be usable; a future transmission is not.
inline bool available(const gnss_comm::EphemBasePtr& eph,
                      gnss_comm::gtime_t time, double max_age_s) {
  return eph && eph->sat != 0 && eph->health == 0 && eph->ttr.time != 0 &&
         std::isfinite(max_age_s) && max_age_s > 0 &&
         gnss_comm::time_diff(time, eph->ttr) >= 0 &&
         std::abs(gnss_comm::time_diff(time, eph->toe)) <= max_age_s;
}

inline bool all_requested_present(const std::vector<uint32_t>& requested,
                                 const std::vector<uint32_t>& present) {
  return !requested.empty() && std::all_of(requested.begin(), requested.end(),
      [&](uint32_t sys) {
        return std::find(present.begin(), present.end(), sys) != present.end();
      });
}

inline double nav_number(const std::string& line, size_t offset) {
  if (line.size() < offset + 19) throw std::invalid_argument("truncated NAV numeric field");
  std::string field = line.substr(offset, 19);
  std::replace(field.begin(), field.end(), 'D', 'e');
  size_t used = 0;
  const double value = std::stod(field, &used);
  if (!std::isfinite(value) || field.find_first_not_of(' ', used) != std::string::npos)
    throw std::invalid_argument("nonfinite or malformed NAV numeric field");
  return value;
}

inline uint32_t nav_unsigned(const std::string& line, size_t offset) {
  const double value = nav_number(line, offset);
  if (value < 0 || value > std::numeric_limits<uint32_t>::max() || std::floor(value) != value)
    throw std::invalid_argument("noninteger or out-of-range NAV qualification field");
  return static_cast<uint32_t>(value);
}

// Retain the installed decoder, but validate before its fatal checks and fix
// the two observed BDS metadata omissions at the input authority. No timestamp
// is replaced by query time. RINEX 3.04 appendix A14 defines AODE and BDT TTR.
inline std::map<uint32_t, std::vector<gnss_comm::EphemBasePtr>> load_nav(
    const std::string& path,
    const std::vector<uint32_t>& requested = {SYS_GPS, SYS_BDS, SYS_GAL, SYS_GLO}) {
  std::ifstream file(path);
  if (!file) throw std::invalid_argument("cannot open RINEX NAV file");
  std::string line;
  bool version = false, leap = false, end = false;
  while (std::getline(file, line)) {
    if (line.find("RINEX VERSION / TYPE") != std::string::npos) {
      version = line.find("3.04") != std::string::npos &&
          (line.find("NAV") != std::string::npos || line.find("N:") != std::string::npos);
    }
    if (line.find("LEAP SECONDS") != std::string::npos &&
        line.find("BDS") == std::string::npos) {
      size_t consumed = 0;
      const std::string field = line.substr(4, 6);
      const int seconds = std::stoi(field, &consumed);
      if (field.find_first_not_of(' ', consumed) != std::string::npos)
        throw std::invalid_argument("noninteger LEAP SECONDS");
      leap = seconds >= 0 && seconds <= 100;
    }
    if (line.find("END OF HEADER") != std::string::npos) { end = true; break; }
  }
  if (!version || !leap || !end)
    throw std::invalid_argument("requires RINEX 3.04 NAV, LEAP SECONDS and END OF HEADER");
  struct BdsMetadata { gnss_comm::gtime_t toc; uint32_t aode; uint32_t aodc; };
  std::map<uint32_t, std::vector<BdsMetadata>> bds;
  while (std::getline(file, line)) {
    if (line.size() < 23 || std::string("GCRESJI").find(line[0]) == std::string::npos)
      throw std::invalid_argument("unsupported or malformed NAV record identity");
    const char system = line[0];
    const uint32_t sys = system == 'G' ? SYS_GPS : system == 'C' ? SYS_BDS :
                         system == 'E' ? SYS_GAL : system == 'R' ? SYS_GLO : SYS_NONE;
    const bool selected = std::find(requested.begin(), requested.end(), sys) != requested.end();
    const size_t count = system == 'R' || system == 'S' ? 4 : 8;
    std::array<std::string, 8> block;
    block[0] = line;
    for (size_t i = 1; i < count; ++i)
      if (!std::getline(file, block[i]) || block[i].empty())
        throw std::invalid_argument("truncated NAV record");
    // SBAS/QZSS/IRNSS records may coexist in a mixed file. The installed
    // decoder does not consume them; they grant no requested-system presence.
    if (system == 'S' || system == 'J' || system == 'I') continue;
    double epoch[6];
    std::istringstream stamp(block[0].substr(4, 19));
    for (double& part : epoch)
      if (!(stamp >> part)) throw std::invalid_argument("malformed NAV epoch");
    if (epoch[0] < 1980 || epoch[1] < 1 || epoch[1] > 12 || epoch[2] < 1 ||
        epoch[2] > 31 || epoch[3] < 0 || epoch[3] >= 24 || epoch[4] < 0 ||
        epoch[4] >= 60 || epoch[5] < 0 || epoch[5] >= 60)
      throw std::invalid_argument("unsupported NAV epoch");
    if (block[0][1] < '0' || block[0][1] > '9' ||
        block[0][2] < '0' || block[0][2] > '9')
      throw std::invalid_argument("malformed NAV satellite identity");
    const int prn = std::stoi(block[0].substr(1, 2));
    if (prn <= 0) throw std::invalid_argument("invalid NAV satellite identity");
    if (selected && !gnss_comm::sat_no(sys, prn))
      throw std::invalid_argument("unsupported requested NAV satellite identity");
    for (size_t n = 0; n < (system == 'R' ? 2u : 3u); ++n)
      nav_number(block[0], 23 + 19 * n);
    for (size_t i = 1; i < count; ++i) {
      const size_t fields = i == 7 ? 1 : 4;
      for (size_t n = 0; n < fields; ++n) {
        // These spare fields are not consumed by the installed decoder.
        if (i == 5 && (n == 3 || (n == 1 && system != 'E'))) continue;
        nav_number(block[i], 4 + 19 * n);
      }
    }
    // These decoder casts must not turn fractional health into healthy zero,
    // or silently wrap week/issue/source identity before qualification.
    if (system == 'R') {
      nav_unsigned(block[1], 61);
      nav_unsigned(block[3], 61);
      const double channel = nav_number(block[2], 61);
      if (std::floor(channel) != channel || channel < -7 || channel > 6)
        throw std::invalid_argument("unsupported GLONASS frequency channel");
    } else {
      nav_unsigned(block[5], 42);
      nav_unsigned(block[6], 23);
      if (system == 'G') { nav_unsigned(block[1], 4); nav_unsigned(block[6], 61); }
      if (system == 'E') nav_unsigned(block[5], 23);
    }
    if (system == 'C' && selected) {
      const double aode = nav_number(block[1], 4), aodc = nav_number(block[7], 23);
      if (aode < 0 || aode > 31 || aodc < 0 || aodc > 31 ||
          std::floor(aode) != aode || std::floor(aodc) != aodc)
        throw std::invalid_argument("unsupported BDS AODE/AODC");
      const auto sat = gnss_comm::sat_no(SYS_BDS, prn);
      if (!sat) throw std::invalid_argument("unsupported BDS satellite identity");
      bds[sat].push_back({gnss_comm::time_add(gnss_comm::epoch2time(epoch), 14),
                         static_cast<uint32_t>(aode), static_cast<uint32_t>(aodc)});
    }
  }
  std::map<uint32_t, std::vector<gnss_comm::EphemBasePtr>> records;
  gnss_comm::rinex2ephems(path, records);
  for (auto it = records.begin(); it != records.end();) {
    if (std::find(requested.begin(), requested.end(),
                  gnss_comm::satsys(it->first, nullptr)) == requested.end()) it = records.erase(it);
    else ++it;
  }
  for (auto& [sat, entries] : records) {
    const bool is_bds = gnss_comm::satsys(sat, nullptr) == SYS_BDS;
    size_t index = 0;
    for (auto& base : entries) {
      const auto eph = std::dynamic_pointer_cast<gnss_comm::Ephem>(base);
      if (!eph) continue;
      if (is_bds) {
        if (index >= bds[sat].size() ||
            std::abs(gnss_comm::time_diff(eph->toc, bds[sat][index].toc)) > 1e-6)
          throw std::invalid_argument("BDS metadata/decoder record mismatch");
        eph->iode = bds[sat][index].aode;
        eph->iodc = bds[sat][index++].aodc;
        eph->ttr = gnss_comm::time_add(eph->ttr, 14);
      }
      // RINEX TTR may use the adjacent week's seconds. Bind it to original TOC.
      const double delta = gnss_comm::time_diff(eph->ttr, eph->toc);
      if (delta > 302400) eph->ttr = gnss_comm::time_add(eph->ttr, -604800);
      if (delta < -302400) eph->ttr = gnss_comm::time_add(eph->ttr, 604800);
    }
  }
  return records;
}

}  // namespace iap::broadcast
