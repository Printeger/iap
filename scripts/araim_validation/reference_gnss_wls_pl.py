#!/usr/bin/env python3
import numpy as np
from statistics import NormalDist


SATS = [
    {"prn": "G01", "constellation": "GPS", "az_deg": 0, "el_deg": 60},
    {"prn": "G02", "constellation": "GPS", "az_deg": 60, "el_deg": 50},
    {"prn": "G03", "constellation": "GPS", "az_deg": 120, "el_deg": 55},
    {"prn": "G04", "constellation": "GPS", "az_deg": 200, "el_deg": 45},
    {"prn": "G05", "constellation": "GPS", "az_deg": 280, "el_deg": 50},
    {"prn": "G06", "constellation": "GPS", "az_deg": 330, "el_deg": 65},
]

SIGMA_M = 1.5
RESIDUAL_M = np.zeros(len(SATS), dtype=float)

ARAIM_PARAMS = {
    "PHMI": 1e-7,
    "PHMI_alloc_fault_free": 5e-8,
    "PFA": 1e-5,
    "satellite_fault_prior": 1e-5,
    "constellation_fault_prior": 1e-4,
}


def los_from_az_el(az_deg, el_deg):
    az = np.deg2rad(az_deg)
    el = np.deg2rad(el_deg)

    # ENU unit line-of-sight direction
    e = np.array([
        np.cos(el) * np.sin(az),  # East
        np.cos(el) * np.cos(az),  # North
        np.sin(el),               # Up
    ])
    return e / np.linalg.norm(e)


def qinv_tail(p):
    # Q(x)=p => x=Phi^{-1}(1-p)
    return NormalDist().inv_cdf(1.0 - p)


def build_G(sats):
    rows = []
    for sat in sats:
        e = los_from_az_el(sat["az_deg"], sat["el_deg"])
        # Common GNSS linearized pseudorange geometry:
        # residual ~= [-e_E, -e_N, -e_U, 1] * [dE, dN, dU, clock]
        rows.append([-e[0], -e[1], -e[2], 1.0])
    return np.array(rows, dtype=float)


def compute_fault_free_pl(G, sigma_m=SIGMA_M,
                          phmi_alloc_0=ARAIM_PARAMS["PHMI_alloc_fault_free"]):
    n = G.shape[0]
    W = np.eye(n) / (sigma_m ** 2)

    N = G.T @ W @ G
    Q = np.linalg.inv(N)

    # position covariance block E,N,U
    Qp = Q[:3, :3]

    sigma_e = np.sqrt(Qp[0, 0])
    sigma_n = np.sqrt(Qp[1, 1])
    sigma_u = np.sqrt(Qp[2, 2])

    # Two-sided tail: Q(K)=PHMI/2
    Kff = qinv_tail(phmi_alloc_0 / 2.0)

    pl_e = Kff * sigma_e
    pl_n = Kff * sigma_n
    pl_u = Kff * sigma_u

    hpl = max(pl_e, pl_n)
    vpl = pl_u

    return {
        "Kff": Kff,
        "sigma_e": sigma_e,
        "sigma_n": sigma_n,
        "sigma_u": sigma_u,
        "PL_E": pl_e,
        "PL_N": pl_n,
        "PL_U": pl_u,
        "HPL": hpl,
        "VPL": vpl,
        "W": W,
        "Q": Q,
    }


def main():
    np.set_printoptions(precision=12, suppress=False)

    G = build_G(SATS)
    result = compute_fault_free_pl(G)

    print("satellites =")
    for i, sat in enumerate(SATS):
        print(
            f"{i}: prn={sat['prn']} constellation={sat['constellation']} "
            f"az_deg={sat['az_deg']} el_deg={sat['el_deg']} sigma_m={SIGMA_M}"
        )
    print("")

    print("ARAIM_PARAMS =")
    for k, v in ARAIM_PARAMS.items():
        print(f"{k}: {v}")
    print("")

    print("residual_m =")
    print(RESIDUAL_M)
    print("")

    print("G =")
    print(G)
    print("")

    print("W =")
    print(result["W"])
    print("")

    print("Q =")
    print(result["Q"])
    print("")

    for k, v in result.items():
        if k not in ("W", "Q"):
            print(f"{k}: {v}")


if __name__ == "__main__":
    main()
