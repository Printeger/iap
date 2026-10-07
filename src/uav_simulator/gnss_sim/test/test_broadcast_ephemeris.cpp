#include <gtest/gtest.h>
#include <iap/gnss/broadcast_ephemeris.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>

namespace {
gnss_comm::EphemPtr c59() {
  // RINEX C59 2022-07-06 12:00 BDT, original NAV SHA256
  // 42e87bd2edff3bb66e6d58b01c1710e9547270cfe795203ce947b8d88356bd32.
  auto e = std::make_shared<gnss_comm::Ephem>();
  e->sat = gnss_comm::sat_no(SYS_BDS, 59);
  double epoch[] = {2022, 7, 6, 12, 0, 0};
  e->toc = e->toe = gnss_comm::time_add(gnss_comm::epoch2time(epoch), 14);
  e->toe_tow = gnss_comm::time2gpst(e->toe, &e->week);
  e->A = std::pow(6.493351522446e3, 2);
  e->e = 3.880668664351e-4;
  e->i0 = 7.999031248902e-2;
  e->omg = -1.979536071166;
  e->OMG0 = -1.052177466729e-1;
  e->M0 = 1.446131433705;
  e->delta_n = 2.242593413017e-9;
  e->OMG_dot = -1.326126667070e-9;
  e->i_dot = -2.367955777720e-10;
  e->cuc = -3.363005816936e-6;
  e->cus = -1.573096960783e-5;
  e->crc = 4.8375e2;
  e->crs = -1.08328125e2;
  e->cic = -1.722946763039e-7;
  e->cis = -4.656612873077e-10;
  e->af0 = 1.025618985295e-7;
  e->af1 = -1.989519660128e-13;
  e->af2 = 0;
  return e;
}

TEST(BroadcastEphemeris, CapturedC59AgainstIndependentReference) {
  double epoch[] = {2022, 7, 6, 12, 0, 0};
  auto t = gnss_comm::utc2gpst(gnss_comm::epoch2time(epoch));
  double clock = 0;
  auto p = iap::broadcast::position(t, c59(), &clock);
  // Pinned RTKLIB readrnx/eph2pos, same record and query, independently decoded.
  EXPECT_LT((p - Eigen::Vector3d(-32311466.625519741, 27077146.864740886,
                                651702.68747207685)).norm(), 1e-5);
  EXPECT_NEAR(clock, 1.0145017411001905e-7, 1e-16);
  auto v = iap::broadcast::velocity(t, c59());
  // Reference velocity uses a 1ms forward difference; IAP uses a central one.
  EXPECT_LT((v - Eigen::Vector3d(-1.3182871043682098, .2034306526184082,
                                6.2914500012993813)).norm(), .001);
}

TEST(BroadcastEphemeris, MissingEphemerisRejects) {
  EXPECT_THROW(iap::broadcast::position({}, nullptr), std::invalid_argument);
  EXPECT_THROW(iap::broadcast::velocity({}, nullptr), std::invalid_argument);
}

struct NavFile {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("iap_nav_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ~NavFile() { std::filesystem::remove(path); }
  void write(const std::string& text) { std::ofstream(path) << text; }
};

const char* kNav = R"NAV(     3.04           N: GNSS NAV DATA    M: MIXED              RINEX VERSION / TYPE
    18                                                      LEAP SECONDS
                                                            END OF HEADER
C59 2022 07 06 12 00 00 1.025618985295e-07-1.989519660128e-13 0.000000000000e+00
     1.000000000000e+00-1.083281250000e+02 2.242593413017e-09 1.446131433705e+00
    -3.363005816936e-06 3.880668664351e-04-1.573096960783e-05 6.493351522446e+03
     3.024000000000e+05-1.722946763039e-07-1.052177466729e-01-4.656612873077e-10
     7.999031248902e-02 4.837500000000e+02-1.979536071166e+00-1.326126667070e-09
    -2.367955777720e-10 0.000000000000e+00 8.610000000000e+02 0.000000000000e+00
     2.000000000000e+00 0.000000000000e+00 4.280000000000e-08 4.280000000000e-08
     3.024000000000e+05 1.000000000000e+00
)NAV";

TEST(BroadcastEphemeris, BdtTransmissionAndIssueIdentityComeFromOriginalRecord) {
  NavFile file;
  file.write(kNav);
  const auto records = iap::broadcast::load_nav(file.path.string());
  const auto e = std::dynamic_pointer_cast<gnss_comm::Ephem>(records.at(
      gnss_comm::sat_no(SYS_BDS, 59)).front());
  ASSERT_TRUE(e);
  EXPECT_EQ(e->iode, 1u);
  EXPECT_EQ(e->iodc, 1u);
  EXPECT_DOUBLE_EQ(gnss_comm::time_diff(e->ttr, e->toc), 0);
  EXPECT_EQ(e->ttr.time, 1657108814);
  EXPECT_FALSE(iap::broadcast::available(e, gnss_comm::time_add(e->ttr, -1), 7200));
  EXPECT_TRUE(iap::broadcast::available(e, e->ttr, 7200));
  EXPECT_FALSE(iap::broadcast::available(e, gnss_comm::time_add(e->toe, 7201), 7200));
  e->health = 1;
  EXPECT_FALSE(iap::broadcast::available(e, e->ttr, 7200));
}

TEST(BroadcastEphemeris, MissingAnyRequestedConstellationRejects) {
  EXPECT_TRUE(iap::broadcast::all_requested_present({SYS_GPS, SYS_BDS}, {SYS_GPS, SYS_BDS}));
  EXPECT_FALSE(iap::broadcast::all_requested_present({SYS_GPS, SYS_BDS}, {SYS_GPS}));
  EXPECT_FALSE(iap::broadcast::all_requested_present({SYS_GPS, SYS_BDS}, {SYS_BDS}));
  EXPECT_FALSE(iap::broadcast::all_requested_present({SYS_GPS, SYS_BDS}, {}));
  EXPECT_FALSE(iap::broadcast::all_requested_present({}, {SYS_GPS}));
}

TEST(BroadcastEphemeris, UnusedSystemsInMixedFileDoNotQualifyMissingConstellation) {
  NavFile file;
  file.write(std::string(kNav) + "S23 2022 07 06 12 00 00 unused SBAS record\n"
      "    ignored SBAS position\n    ignored SBAS velocity\n    ignored SBAS acceleration\n");
  const auto records = iap::broadcast::load_nav(file.path.string());
  EXPECT_EQ(records.size(), 1u);
  EXPECT_EQ(gnss_comm::satsys(records.begin()->first, nullptr), SYS_BDS);
  EXPECT_FALSE(iap::broadcast::all_requested_present({SYS_GPS, SYS_BDS}, {SYS_BDS}));
}

TEST(BroadcastEphemeris, MalformedRecordRejectsBeforeFatalUpstreamDecoder) {
  NavFile file;
  file.write("invalid header\n");
  EXPECT_THROW(iap::broadcast::load_nav(file.path.string()), std::exception);
  file.write(std::string(kNav).substr(0, std::string(kNav).find("C59") + 81));
  EXPECT_THROW(iap::broadcast::load_nav(file.path.string()), std::exception);
  auto text = std::string(kNav);
  text.replace(text.find("1.025618985295e-07"), 17, "              nan");
  file.write(text);
  EXPECT_THROW(iap::broadcast::load_nav(file.path.string()), std::exception);
}

TEST(BroadcastEphemeris, FractionalNegativeAndOverflowHealthNeverBecomeHealthy) {
  NavFile file;
  for (const std::string health : {" 5.000000000000e-01", "-1.000000000000e+00",
                                  " 1.000000000000e+20"}) {
    auto text = std::string(kNav);
    const auto row = text.find("     2.000000000000e+00");
    text.replace(row + 23, 19, health);
    file.write(text);
    EXPECT_THROW(iap::broadcast::load_nav(file.path.string()), std::exception) << health;
  }
}

TEST(BroadcastEphemeris, UnsupportedDisabledBdsIssueDoesNotBlockGpsOnly) {
  NavFile file;
  auto text = std::string(kNav);
  text.replace(text.find("     1.000000000000e+00") + 4, 19, " 3.200000000000e+01");
  file.write(text);
  EXPECT_TRUE(iap::broadcast::load_nav(file.path.string(), {SYS_GPS}).empty());
  EXPECT_THROW(iap::broadcast::load_nav(file.path.string(), {SYS_BDS}), std::exception);
}
}  // namespace
