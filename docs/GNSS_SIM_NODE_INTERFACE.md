# GNSS simulation node interface sketch for IAP (ROS2)

This document provides a direct integration sketch for current IAP GNSS extension.
It focuses on:

- ROS2 publisher list (topic + message type + suggested rate)
- Minimal field-fill template for each topic
- A startup sequence that works with current IAP code path

## 1. Contract expected by current IAP

IAP GNSS extension subscribes to these exact topics:

- /ublox_driver/range_meas (gnss_comm/msg/GnssMeasMsg)
- /ublox_driver/ephem (gnss_comm/msg/GnssEphemMsg)
- /ublox_driver/glo_ephem (gnss_comm/msg/GnssGloEphemMsg)
- /ublox_driver/receiver_lla (sensor_msgs/msg/NavSatFix)
- /ublox_driver/iono_params (gnss_comm/msg/GnssIonosphereParameter)

Notes:

- Topic names should match exactly unless you change IAP source.
- Satellite id field sat must follow gnss_comm/gnss_utility.hpp sat_no convention.
- Epoch time in GnssObsMsg.time should be GNSS week + tow.

## 2. Publisher list (recommended minimal rates)

| Topic | Type | Suggested rate | Required for run | Purpose |
|---|---|---:|---|---|
| /ublox_driver/receiver_lla | sensor_msgs/msg/NavSatFix | 1 Hz (or once before run) | Yes | Seed world-to-ECEF anchor |
| /ublox_driver/ephem | gnss_comm/msg/GnssEphemMsg | 0.2 to 1 Hz per sat (on update) | Yes (GPS/GAL/BDS sats) | Orbit/clock for sat position and correction |
| /ublox_driver/glo_ephem | gnss_comm/msg/GnssGloEphemMsg | 0.2 to 1 Hz per sat (on update) | If using GLONASS | Orbit/clock for GLONASS |
| /ublox_driver/iono_params | gnss_comm/msg/GnssIonosphereParameter | 0.2 to 1 Hz | Strongly recommended | Klobuchar ionosphere correction |
| /ublox_driver/range_meas | gnss_comm/msg/GnssMeasMsg | 5 to 20 Hz (align with GNSS epoch) | Yes | Raw pseudorange and Doppler observables |

## 3. Minimal field template per topic

## 3.1 /ublox_driver/receiver_lla

Message type: sensor_msgs/msg/NavSatFix

Minimum fields consumed by IAP:

- latitude (deg)
- longitude (deg)
- altitude (m)

Template:

```python
from sensor_msgs.msg import NavSatFix

msg = NavSatFix()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "wgs84"
msg.latitude = 31.2304
msg.longitude = 121.4737
msg.altitude = 25.0
# Other fields can keep defaults for minimal integration.
pub_receiver_lla.publish(msg)
```

## 3.2 /ublox_driver/iono_params

Message type: gnss_comm/msg/GnssIonosphereParameter

Minimum fields:

- type: 0 (GPS), 1 (BDS), 2 (Galileo)
- parameters: at least 8 values for Klobuchar (alpha0..3, beta0..3)

Template:

```python
from gnss_comm.msg import GnssIonosphereParameter

msg = GnssIonosphereParameter()
msg.header.stamp = node.get_clock().now().to_msg()
msg.type = 0
msg.parameters = [
    1.0e-8, 0.0, 0.0, 0.0,
    9.0e4, 0.0, 0.0, 0.0,
]
pub_iono.publish(msg)
```

## 3.3 /ublox_driver/ephem (GPS/GAL/BDS)

Message type: gnss_comm/msg/GnssEphemMsg

Important:

- IAP computes satellite position/velocity from this message.
- Do not send all-zero placeholders in real run.
- Fields used by orbit and clock model must be physically valid.

Minimal structural template (fill with real simulated broadcast ephemeris):

```python
from gnss_comm.msg import GnssEphemMsg

msg = GnssEphemMsg()
msg.sat = sat_id              # must follow sat_no convention

msg.ttr.week = gps_week
msg.ttr.tow = gps_tow
msg.toe.week = gps_week
msg.toe.tow = toe_tow
msg.toc.week = gps_week
msg.toc.tow = toc_tow
msg.toe_tow = toe_tow
msg.week = gps_week

msg.iode = iode
msg.iodc = iodc
msg.health = 0
msg.code = code
msg.ura = ura

# Kepler and harmonic parameters
msg.a = a
msg.e = e
msg.i0 = i0
msg.omg = omg
msg.omg0 = omg0
msg.m0 = m0
msg.delta_n = delta_n
msg.omg_dot = omg_dot
msg.i_dot = i_dot
msg.cuc = cuc
msg.cus = cus
msg.crc = crc
msg.crs = crs
msg.cic = cic
msg.cis = cis

# Satellite clock/group delay
msg.af0 = af0
msg.af1 = af1
msg.af2 = af2
msg.tgd0 = tgd0
msg.tgd1 = tgd1

# Optional rates
msg.a_dot = a_dot
msg.n_dot = n_dot

pub_ephem.publish(msg)
```

## 3.4 /ublox_driver/glo_ephem (GLONASS)

Message type: gnss_comm/msg/GnssGloEphemMsg

Template:

```python
from gnss_comm.msg import GnssGloEphemMsg

msg = GnssGloEphemMsg()
msg.sat = sat_id
msg.ttr.week = gps_week
msg.ttr.tow = gps_tow
msg.toe.week = gps_week
msg.toe.tow = toe_tow

msg.freqo = freqo
msg.iode = iode
msg.health = 0
msg.age = age
msg.ura = ura

msg.pos_x = pos_x
msg.pos_y = pos_y
msg.pos_z = pos_z
msg.vel_x = vel_x
msg.vel_y = vel_y
msg.vel_z = vel_z
msg.acc_x = acc_x
msg.acc_y = acc_y
msg.acc_z = acc_z

msg.tau_n = tau_n
msg.gamma = gamma
msg.delta_tau_n = delta_tau_n

pub_glo_ephem.publish(msg)
```

## 3.5 /ublox_driver/range_meas

Message type: gnss_comm/msg/GnssMeasMsg

IAP minimal checks on each satellite measurement:

- must have L1 frequency index from freqs
- pseudorange psr[L1] must be finite and > 0
- Doppler can be used (recommended to provide dopp[L1])

Single-satellite minimal template inside one epoch:

```python
from gnss_comm.msg import GnssMeasMsg, GnssObsMsg

obs = GnssObsMsg()
obs.time.week = gps_week
obs.time.tow = gps_tow
obs.sat = sat_id

# Keep arrays aligned by index. Index 0 used as L1 in this minimal case.
obs.freqs = [1575.42e6]       # GPS L1
obs.cn0 = [45.0]
obs.lli = [0]
obs.code = [0]

obs.psr = [2.35e7]            # meters, > 0
obs.psr_std = [2.0]           # meters
obs.cp = [0.0]                # optional for current IAP path
obs.cp_std = [0.0]
obs.dopp = [-1200.0]          # Hz
obs.dopp_std = [0.5]          # Hz
obs.status = [1]              # bit0: psr valid

epoch_msg = GnssMeasMsg()
epoch_msg.meas = [obs_sat1, obs_sat2, obs_sat3]  # at least 4 sats recommended
pub_range_meas.publish(epoch_msg)
```

## 4. Minimal node skeleton (Python, ROS2)

```python
class GnssSimNode(Node):
    def __init__(self):
        super().__init__("gnss_sim_node")

        self.pub_receiver_lla = self.create_publisher(NavSatFix, "/ublox_driver/receiver_lla", 10)
        self.pub_ephem = self.create_publisher(GnssEphemMsg, "/ublox_driver/ephem", 50)
        self.pub_glo_ephem = self.create_publisher(GnssGloEphemMsg, "/ublox_driver/glo_ephem", 50)
        self.pub_iono = self.create_publisher(GnssIonosphereParameter, "/ublox_driver/iono_params", 10)
        self.pub_range_meas = self.create_publisher(GnssMeasMsg, "/ublox_driver/range_meas", 100)

        self.timer_fast = self.create_timer(0.1, self.publish_range_meas)   # 10 Hz
        self.timer_slow = self.create_timer(1.0, self.publish_nav_iono_ephem)
```

## 5. Startup sequence that works with current IAP

1. Publish /ublox_driver/receiver_lla at least once before GNSS factor injection.
2. Keep ephemeris cache warm: publish /ephem and /glo_ephem continuously or on update.
3. Publish /iono_params (8-parameter Klobuchar) before or during range stream.
4. Start /range_meas at stable epoch rate.
5. Ensure GNSS epoch time is aligned with LiDAR frame time within config_gnss time_tolerance.

## 6. Practical acceptance checklist

- IAP logs no warning about missing receiver_lla seed.
- IAP logs GNSS epoch inserted with non-zero satellite count.
- Integrity report topic /iap/integrity is being published when integrity module enabled.
- If ARAIM enabled, report fields hpl/vpl/im update over time.

## 7. Common integration pitfalls

- sat id not using sat_no convention, causing wrong constellation mapping.
- Publishing range_meas without valid ephemeris cache.
- All-zero ephemeris fields leading to invalid satellite position.
- GNSS time base not aligned with bag or LiDAR timestamps, causing epoch drops.
- Too few visible satellites (< 4) for meaningful geometry and integrity metrics.
