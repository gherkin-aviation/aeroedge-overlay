import math
import requests
from geopy.point import Point
from geopy.distance import distance
from geopy.point import Point as GeoPoint
import numpy as np
from math import degrees
from geopy import Point as GeoPoint
from geopy.distance import geodesic as geo_dist
from math import radians, tan
from geopy import Point as GeoPoint
from geopy.distance import geodesic as geo_dist, distance
from math import radians, tan, degrees, sin, cos, atan2

# Constants
g = 32.174  # ft/s^2
R = 1716.0  # ft*lbf/(slug*R)
T_sl = 518.67  # Rankine
P_sl = 2116.22  # lbf/ft^2
rho_sl = 0.0023769  # slugs/ft^3
OVERHEAD_THRESH_FT = 4000
# Final leg geometry constants
FINAL_MIN_DIST_NM = 0.05          # shortest acceptable final leg
FINAL_MAX_DIST_NM = 0.8          # longest acceptable final leg
FINAL_CROSSING_HEIGHT_FT = 50.0   # height to cross the threshold
FINAL_ALIGN_TOL_DEG = 30.0        # alignment tolerance for breakout heading


def compute_density_altitude(oat_c, pressure_alt_ft):
    isa_temp_c = 15 - 2 * (pressure_alt_ft / 1000.0)
    density_alt = pressure_alt_ft + (120 * (oat_c - isa_temp_c))
    return density_alt

def compute_pressure_altitude(indicated_alt_ft, altimeter_inhg):
    return indicated_alt_ft + (29.92 - altimeter_inhg) * 1000

def compute_true_airspeed(ias_kts, pressure_alt_ft, oat_c):
    """
    Convert IAS to TAS using standard IAS→TAS scaling with local density.

    pressure_alt_ft: pressure altitude (not density altitude)
    oat_c: actual outside air temperature in °C
    """
    rho = compute_air_density(pressure_alt_ft, oat_c)
    return ias_kts / math.sqrt(rho / rho_sl)

def compute_turn_radius(tas_kts, bank_deg):
    tas_fps = tas_kts * 1.68781
    bank_rad = math.radians(bank_deg)
    return (tas_fps ** 2) / (g * math.tan(bank_rad))

def compute_required_bank(tas_kts, radius_ft):
    tas_fps = tas_kts * 1.68781
    return math.degrees(math.atan(tas_fps ** 2 / (g * radius_ft)))

def compute_glide_ratio(base_ratio, flap_config, gear_type, prop_config):
    flap_drag = {"clean": 1.0, "takeoff": 1.1, "landing": 1.25}.get(flap_config, 1.0)
    gear_drag = 1.1 if gear_type == "retractable" else 1.0
    prop_drag = {"feathered": 1.0, "stationary": 1.2, "windmilling": 1.3, "idle": 1.05}.get(prop_config, 1.0)
    total_drag = flap_drag * gear_drag * prop_drag
    return base_ratio / total_drag

def compute_descent_angle_deg(glide_ratio):
    return math.degrees(math.atan(1 / glide_ratio))

def compute_Ps(thrust_lbf, drag_lbf, tas_fps, weight_lbf):
    return ((thrust_lbf - drag_lbf) * tas_fps) / weight_lbf

def compute_lift_limit_speed(cl_max, weight_lbf, rho, S):
    return math.sqrt((2 * weight_lbf) / (rho * cl_max * S))

def compute_load_factor(bank_deg):
    return 1 / math.cos(math.radians(bank_deg))

def compute_stall_speed(clean_stall_kias, load_factor):
    return clean_stall_kias * math.sqrt(load_factor)

def knots_to_fps(knots):
    return knots * 1.68781

def fps_to_knots(fps):
    return fps / 1.68781

def fpm_to_fps(fpm):
    return fpm / 60.0

def compute_air_density(pressure_alt_ft, oat_c):
    temp_r = (oat_c + 273.15) * 9/5
    pressure_psf = P_sl * (1 - 0.0000068756 * pressure_alt_ft) ** 5.2561  # lbf/ft^2
    rho = pressure_psf / (R * temp_r)
    return rho

def adjust_glide_ratio_for_density(glide_ratio, rho):
    factor = (rho / rho_sl) ** 0.3
    return max(glide_ratio * factor, 3.5)

def point_from(p, bearing_deg, dist_nm):
    return distance(nautical=dist_nm).destination(p, bearing_deg)

def calculate_initial_compass_bearing(pointA, pointB):
    lat1 = math.radians(pointA.latitude)
    lat2 = math.radians(pointB.latitude)
    diff_long = math.radians(pointB.longitude - pointA.longitude)
    x = math.sin(diff_long) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diff_long))
    initial_bearing = math.atan2(x, y)
    return (math.degrees(initial_bearing) + 360) % 360

def wind_components(wind_dir_deg, wind_speed_kt):
    wind_rad = math.radians(wind_dir_deg)
    return wind_speed_kt * math.cos(wind_rad), wind_speed_kt * math.sin(wind_rad)


def estimate_energy_bleed_distance(start_ias_kias, best_glide_kias, tau, tas_fps):
    """
    Estimate the distance required to bleed speed from start IAS to best glide IAS
    using exponential decay model (tau-based), assuming a level glide segment.
    Returns distance in meters.
    """
    print("\n[ENERGY BLEED ESTIMATION]")
    print(f"  Start IAS:       {start_ias_kias:.1f} kt")
    print(f"  Best Glide IAS:  {best_glide_kias:.1f} kt")

    if start_ias_kias <= best_glide_kias + 1:
        print("  No energy bleed needed (IAS close to best glide).")
        return 0.0

    # Target IAS is slightly above best glide to ensure decay stability
    decay_target_ias = max(best_glide_kias + 5, best_glide_kias + 0.05 * (start_ias_kias - best_glide_kias))
    decay_fraction = 1 - ((decay_target_ias - best_glide_kias) / (start_ias_kias - best_glide_kias))
    decay_fraction = max(decay_fraction, 1e-6)

    t_seconds = -tau * math.log(decay_fraction)
    distance_m = tas_fps * t_seconds

    print(f"  Target IAS for decay: {decay_target_ias:.1f} kt")
    print(f"  Decay τ:              {tau:.2f} sec")
    print(f"  Time to bleed:        {t_seconds:.1f} sec")
    print(f"  Estimated distance:   {distance_m:.1f} m\n")

    return round(distance_m, 1)

import math

FT_PER_NM = 6076.12
G_FPS2 = 32.174


def _wrap_360(deg: float) -> float:
    deg = float(deg) % 360.0
    return deg + 360.0 if deg < 0 else deg

def _angle_diff_deg(a: float, b: float) -> float:
    # signed shortest diff a-b in degrees, range [-180,180]
    d = (float(a) - float(b) + 180.0) % 360.0 - 180.0
    return d


def _bearing_to_unit_ne(bearing_deg: float):
    # bearing: 0=N, 90=E
    br = math.radians(_wrap_360(bearing_deg))
    return (math.cos(br), math.sin(br))  # (north, east) unit

def _cross_track_distance_ft(point_a, line_origin, line_bearing_deg: float) -> float:
    """
    Signed cross track distance from point to infinite line through line_origin with bearing line_bearing_deg.
    Positive means point is right of the line (looking along the bearing).
    Uses local tangent-plane approximation using your existing distance+bearing.
    """
    # Vector from origin to point in local NE (ft)
    dist_ft = distance(line_origin, point_a).feet
    brg = calculate_initial_compass_bearing(line_origin, point_a)
    dn, de = _bearing_to_unit_ne(brg)
    p_n = dist_ft * dn
    p_e = dist_ft * de

    ln, le = _bearing_to_unit_ne(line_bearing_deg)
    # Cross product z = L x P (in 2D NE coords), gives signed perpendicular magnitude
    # For NE coords, z = ln*pe - le*pn
    xtrack = (ln * p_e) - (le * p_n)
    return xtrack

def _heading_from_track_components(vn_fps: float, ve_fps: float) -> float:
    # Track degrees, 0 = north, 90 = east
    if abs(vn_fps) < 1e-9 and abs(ve_fps) < 1e-9:
        return 0.0
    return _wrap_360(math.degrees(math.atan2(ve_fps, vn_fps)))


def _wind_components_from_dir(wind_from_deg: float, wind_speed_kt: float):
    # Aviation wind is FROM. Convert to TO for velocity vector.
    wind_to_deg = _wrap_360(wind_from_deg + 180.0)
    w_fps = float(wind_speed_kt) * 1.68781
    w_to = math.radians(wind_to_deg)
    wn = w_fps * math.cos(w_to)
    we = w_fps * math.sin(w_to)
    return wn, we


def _best_glide_speed_kias(ac: dict, engine_option: str = None) -> float:
    # Mirror your existing structure: single_engine_limits["best_glide"] is commonly present.
    # Fallbacks are conservative defaults.
    try:
        se = ac.get("single_engine_limits", {})
        bg = se.get("best_glide", None)
        if bg is not None:
            return float(bg)
    except Exception:
        pass
    return 80.0


def _local_xy_ft(origin_pt, pt):
    """
    Very small area approximation: equirectangular projection around origin.
    origin_pt and pt are geopy Point-like with .latitude .longitude.
    Returns (x_east_ft, y_north_ft)
    """
    lat0 = math.radians(origin_pt.latitude)
    dlat = math.radians(pt.latitude - origin_pt.latitude)
    dlon = math.radians(pt.longitude - origin_pt.longitude)
    R = 6371000.0  # meters

    y_m = dlat * R
    x_m = dlon * R * math.cos(lat0)

    ft_per_m = 3.28084
    return x_m * ft_per_m, y_m * ft_per_m

def _canon_flap_config(val: str) -> str:
    v = (val or "clean").strip().lower()
    if v in {"clean", "takeoff", "landing"}:
        return v
    return "clean"


def _canon_prop_config(val: str) -> str:
    v = (val or "windmilling").strip().lower()
    # accept UI terms exactly
    if v in {"idle", "windmilling", "stationary", "feathered"}:
        return v

    # common aliases
    if v in {"stopped", "propstopped", "prop_stopped", "prop stopped"}:
        return "stationary"

    return "windmilling"

def _cross_track_to_centerline_ft(start_pt, cur_pt, runway_heading_deg):
    """
    Centerline passes through start_pt with direction runway_heading_deg.
    Returns signed cross track (ft) and along track (ft) relative to start.
    """
    x, y = _local_xy_ft(start_pt, cur_pt)

    theta = math.radians(_wrap_360(runway_heading_deg))
    ux = math.sin(theta)  # east component
    uy = math.cos(theta)  # north component

    along = x * ux + y * uy
    cross = x * uy - y * ux
    return cross, along

import math

def _ref_weight_lb(ac: dict):
    for k in ("max_gross_lb", "max_gross_weight_lb", "mtow_lb", "gross_weight_lb"):
        v = ac.get(k)
        if isinstance(v, (int, float)) and v > 0:
            return float(v)
    return None

def _runtime_total_weight_lb(ac: dict):
    # Runtime only: set this from your UI total weight callback.
    for k in ("total_weight_lb", "current_total_weight_lb", "selected_weight_lb"):
        v = ac.get(k)
        if isinstance(v, (int, float)) and v > 0:
            return float(v)
    return None

def _weight_adjust_speed_kias(v_ref_kias: float, ac: dict) -> float:
    W = _runtime_total_weight_lb(ac)
    Wref = _ref_weight_lb(ac)
    if W is None or Wref is None or W <= 0 or Wref <= 0:
        return float(v_ref_kias)
    return float(v_ref_kias) * math.sqrt(W / Wref)

def _get_best_glide_and_ratio(ac: dict, engine_option: str, flap_config: str, prop_config: str):
    """
    Returns (best_glide_kias, base_glide_ratio).
    Uses OEI block for multi engine when available, otherwise single engine limits.
    Applies weight-based best-glide scaling if ac contains a runtime total_weight_lb.
    """
    flap_config = _canon_flap_config(flap_config)
    prop_config = _canon_prop_config(prop_config)

    se_limits = ac.get("single_engine_limits", {}) or {}
    base_ratio = float(se_limits.get("best_glide_ratio", 9.0))

    bg_kias = None

    # Multi engine: prefer OEI performance if present
    if int(ac.get("engine_count", 1)) > 1 and engine_option:
        try:
            eo = (ac.get("engine_options", {}) or {}).get(engine_option, {}) or {}
            oei = eo.get("oei_performance", {}) or {}

            key = f"{flap_config}_up"
            block = oei.get(key)

            # Fallbacks if your JSON does not have every flap state
            if block is None:
                block = oei.get("clean_up") or oei.get("up") or oei.get("clean") or None

            if block is not None:
                perf = block.get(prop_config)
                if perf is None:
                    perf = block.get("windmilling") or block.get("idle") or block.get("feathered") or block.get("stationary")

                if perf is not None:
                    bg = perf.get("best_glide_speed_kias")
                    if bg is not None:
                        bg_kias = float(bg)
        except Exception:
            # Fall back to single engine limits
            bg_kias = None

    # Single engine fallback (or OEI incomplete)
    if bg_kias is None:
        bg = se_limits.get("best_glide", None)
        bg_kias = float(bg) if bg is not None else 80.0

    # Weight-adjust final answer
    bg_kias = _weight_adjust_speed_kias(bg_kias, ac)
    return bg_kias, base_ratio



def _run_impossible_turn_once(
    start_point,
    runway_heading_deg: float,
    turn_dir: str,
    bank_deg: float,                 # phase 1 intercept bank command (magnitude)
    reaction_sec: float,
    start_ias_kias: float,
    altitude_agl: float,
    align_window_deg: float,
    ac: dict,
    engine_option: str,
    weight_lbs: float,
    oat_c: float,
    altimeter_inhg: float,
    wind_dir: float,
    wind_speed: float,
    timestep_sec: float,
    flap_config: str = "clean",
    prop_config: str = "windmilling",
    touchdown_elev_ft: float = 0.0,
    min_turn_deg_before_capture: float = 190.0,
    centerline_xtol_ft: float = 150.0,
    max_time_sec: float = 240.0,

    # Phased behavior
    intercept_angle_deg: float = 25.0,
    xtrack_align_gate_ft: float = 1000.0,
    along_align_gate_ft: float = 2000.0,
    jink_bank_cap_deg: float = 30.0,
    jink_hdg_tol_deg: float = 30.0,
    jink_xtrack_tol_ft: float = 50.0,

    # NEW smoothing and wind-corrected tracking
    bank_response_tau_sec: float = 2.0,      # how quickly bank ramps to target
    straight_track_bank_cap_deg: float = 15.0,  # small bank allowed in "straight" for track holding
    xtrack_intercept_scale_ft: float = 1300.0,   # larger = gentler intercept command
    intercept_max_deg: float = 45.0,
):
    dt = float(timestep_sec) if timestep_sec and timestep_sec > 0 else 0.5

    _centerline_xtol_ft = float(centerline_xtol_ft)
    _align_window_deg  = float(align_window_deg)

    runway_hdg = _wrap_360(float(runway_heading_deg))
    hdg = runway_hdg

    # Desired final ground track back is reciprocal (light-blue line direction)
    final_course_hdg = _wrap_360(runway_hdg + 180.0)

    best_glide_kias, base_glide_ratio = _get_best_glide_and_ratio(ac, engine_option, flap_config, prop_config)
    flap_config = _canon_flap_config(flap_config)
    prop_config = _canon_prop_config(prop_config)
    gear_type = ac.get("gear_type", "fixed")


    # Environment
    alt_msl_ft = float(touchdown_elev_ft) + max(0.0, float(altitude_agl))
    pressure_alt_ft = compute_pressure_altitude(alt_msl_ft, float(altimeter_inhg))
    rho = compute_air_density(pressure_alt_ft, float(oat_c))


    # Global glide ratio model
    straight_gr = compute_glide_ratio(base_glide_ratio, flap_config, gear_type, prop_config)
    straight_gr = adjust_glide_ratio_for_density(straight_gr, rho)
    straight_gr = max(3.0, min(straight_gr, 25.0))

    wn_fps, we_fps = _wind_components_from_dir(float(wind_dir), float(wind_speed))

    tau_sec = 4.0
    ias = float(start_ias_kias) if start_ias_kias and float(start_ias_kias) > 1 else best_glide_kias

    alt = float(altitude_agl)
    cur = start_point
    t = 0.0

    turn_dir = "left" if str(turn_dir).lower().startswith("l") else "right"
    sign_turn1 = -1.0 if turn_dir == "left" else 1.0
    sign_turn2 = -sign_turn1

    phase = "reaction"
    reaction_remaining = max(0.0, float(reaction_sec))

    total_turn_1 = 0.0
    total_turn_2 = 0.0

    # Track closest approach to a valid capture state (even if we never succeed)
    best_miss = None
    best_abs_xtrack = None

    # Capture then fly to touchdown
    captured = False
    captured_at_time = None

    path = []
    hover = []

    # Bank is now stateful and ramps to avoid hard jinks
    bank_state_deg = 0.0

    def wind_corrected_heading_for_track(desired_track_deg: float, tas_fps: float) -> float:
        """
        Returns a heading that produces the desired ground track (if feasible) using a wind correction angle.
        Uses standard wind triangle approximation:
          WCA = asin( (wind_cross) / TAS )
          HDG = TRK + WCA
        If infeasible (crosswind > TAS), we clamp WCA to +/- 90 degrees.
        """
        trk = math.radians(_wrap_360(desired_track_deg))
        # Wind components in the track-aligned frame
        # Crosswind positive when wind pushes aircraft to the right of desired track
        w_cross = (-wn_fps * math.sin(trk)) + (we_fps * math.cos(trk))
        ratio = 0.0
        if tas_fps > 1.0:
            ratio = max(-1.0, min(1.0, w_cross / tas_fps))
        wca = math.asin(ratio)
        hdg_out = _wrap_360(desired_track_deg + math.degrees(wca))
        return hdg_out

    def record(gs_kt, aob_deg, vs_fpm, track_deg, drift_deg=None):
        hover.append({
            "time": float(t),
            "alt": float(max(0.0, alt)),
            "tas": float(tas),
            "gs": float(gs_kt),
            "aob": float(aob_deg),
            "vs": float(vs_fpm),
            "track": float(track_deg),
            "heading": float(hdg),
            "drift": float(drift_deg) if drift_deg is not None else None,
            "phase": phase,
        })
        path.append([cur.latitude, cur.longitude])

    def _finalize_meta(success: bool, reason: str, impact_marker=None, xtrack_ft=None, along_ft=None, align_err_deg=None):
        m = {
            "success": bool(success),
            "impact_marker": impact_marker,
            "reason": str(reason),
            "bank_deg": float(bank_deg),
            "jink_bank_cap_deg": float(jink_bank_cap_deg),
            "time_sec": float(t),
            "end_alt_agl_ft": float(max(0.0, alt)),
            "best_xtrack_ft": float(best_abs_xtrack) if best_abs_xtrack is not None else None,
            "best_miss": best_miss,
            "captured": bool(captured),
            "captured_time_sec": float(captured_at_time) if captured_at_time is not None else None,
            "flap_config": str(flap_config),
            "prop_config": str(prop_config),
            "centerline_xtol_ft": _centerline_xtol_ft,
            "align_window_deg": _align_window_deg,
        }
        if xtrack_ft is not None:
            m["final_xtrack_ft"] = float(xtrack_ft)
        if along_ft is not None:
            m["final_along_ft"] = float(along_ft)
        if align_err_deg is not None:
            m["final_hdg_err_deg"] = float(align_err_deg)
        return m

    while t <= max_time_sec:
        # bleed IAS toward best glide
        ias += (best_glide_kias - ias) * min(1.0, dt / tau_sec)

        # TAS from PA + OAT
        try:
            alt_msl = float(touchdown_elev_ft) + max(0.0, alt)
            palt = compute_pressure_altitude(alt_msl, float(altimeter_inhg))
            tas = compute_true_airspeed(float(ias), float(palt), float(oat_c))
            tas = float(tas) if tas and tas > 1 else float(ias)
        except Exception:
            tas = float(ias)

        tas_fps = tas * 1.68781

        # Centerline geometry (reciprocal course through start_point)
        xtrack_ft, along_ft = _cross_track_to_centerline_ft(start_point, cur, final_course_hdg)
        if best_abs_xtrack is None or abs(float(xtrack_ft)) < best_abs_xtrack:
            best_abs_xtrack = abs(float(xtrack_ft))

        # Desired track logic by phase
        # We drive TRACK, not heading, then compute a wind-corrected heading to achieve it.
        if phase == "turn1":
            # aggressive initial turn, no special track target needed (heading changes by turn dynamics)
            desired_track_deg = None
            bank_target_deg = abs(float(bank_deg))
            sign = sign_turn1

        elif phase == "straight":
            # Wings-level-ish segment, but allow small bank to hold an intercept track toward centerline
            intercept_offset = -max(-1.0, min(1.0, float(xtrack_ft) / float(xtrack_intercept_scale_ft))) * float(intercept_max_deg)
            desired_track_deg = _wrap_360(final_course_hdg + intercept_offset)

            # Convert track error into a small bank command (smooth closure, no hard jink)
            # Signed track error: positive means need to turn right, negative means turn left
            # Use current ground track (computed below) once available; for now compute after vg.
            bank_target_deg = None
            sign = 0.0

        elif phase == "turn2":
            # Shallow alignment correction toward the final track
            desired_track_deg = final_course_hdg
            bank_target_deg = abs(min(abs(float(bank_deg)), abs(float(jink_bank_cap_deg))))
            sign = sign_turn2

        elif phase == "final":
            # Fly the reciprocal all the way to touchdown, wings-level (bank tends to 0)
            desired_track_deg = final_course_hdg
            bank_target_deg = 0.0
            sign = 0.0

        else:
            # reaction
            desired_track_deg = None
            bank_target_deg = 0.0
            sign = 0.0

        # Smooth bank ramp
        if bank_response_tau_sec and bank_response_tau_sec > 0:
            alpha = min(1.0, dt / float(bank_response_tau_sec))
        else:
            alpha = 1.0

        # If straight phase, we compute bank_target after we compute current track. Temporarily hold last bank_state.
        if phase != "straight":
            bank_state_deg = bank_state_deg + (float(bank_target_deg) - bank_state_deg) * alpha

        # Use bank_state for turn dynamics
        aob = float(bank_state_deg)

        # If we are in a "track-driven" phase (straight/final), we overwrite heading with a wind corrected heading
        # so that wings-level segments actually fly the intended ground track in wind.
        # For straight, we still allow small bank to change heading gradually, so we do NOT instantly snap heading.
        # For final, we do set heading to the wind corrected heading since bank is near zero anyway.
        # We implement heading correction as a limited-rate yaw toward the wind corrected heading.
        max_hdg_rate_dps = 12.0  # conservative, prevents teleporting headings

        # turning dynamics (coordinated turn)
        if abs(aob) > 0.1 and phase in ["turn1", "turn2", "straight"]:
            turn_rate_rps = (G_FPS2 * math.tan(math.radians(abs(aob)))) / max(1.0, tas_fps)
            turn_rate_dps = math.degrees(turn_rate_rps)
            dpsi = turn_rate_dps * dt
            # For "straight", sign will be set after we compute track error (below). Default to 0 until then.
            if phase in ["turn1", "turn2"]:
                hdg = _wrap_360(hdg + sign * dpsi)
                if phase == "turn1":
                    total_turn_1 += dpsi
                else:
                    total_turn_2 += dpsi
        else:
            turn_rate_dps = 0.0

        # air velocity from heading (north/east)
        hdg_rad = math.radians(hdg)
        va_n = tas_fps * math.cos(hdg_rad)
        va_e = tas_fps * math.sin(hdg_rad)

        # ground velocity with wind
        vg_n = va_n + wn_fps
        vg_e = va_e + we_fps
        gs_fps = math.hypot(vg_n, vg_e)
        gs_kt = gs_fps / 1.68781
        track_deg = _heading_from_track_components(vg_n, vg_e)
        drift_deg = _angle_diff_deg(track_deg, hdg)
        align_err = abs(_angle_diff_deg(track_deg, final_course_hdg))

        # Now that we know track_deg, finish "straight" controls
        if phase == "straight":
            intercept_offset = -max(-1.0, min(1.0, float(xtrack_ft) / float(xtrack_intercept_scale_ft))) * float(intercept_max_deg)
            desired_track_deg = _wrap_360(final_course_hdg + intercept_offset)

            track_err = _angle_diff_deg(desired_track_deg, track_deg)  # signed
            # proportional to track error, capped
            k_bank = 0.35
            cmd = max(-float(straight_track_bank_cap_deg), min(float(straight_track_bank_cap_deg), k_bank * float(track_err)))
            bank_target_signed = float(cmd)

            bank_target_mag = abs(bank_target_signed)
            sign = -1.0 if bank_target_signed < 0 else (1.0 if bank_target_signed > 0 else 0.0)

            bank_state_deg = bank_state_deg + (bank_target_mag - bank_state_deg) * alpha
            aob = float(bank_state_deg)

            # Apply coordinated turn for straight after sign is known
            if abs(aob) > 0.1:
                turn_rate_rps = (G_FPS2 * math.tan(math.radians(abs(aob)))) / max(1.0, tas_fps)
                turn_rate_dps = math.degrees(turn_rate_rps)
                dpsi = turn_rate_dps * dt
                hdg = _wrap_360(hdg + sign * dpsi)

                # Recompute velocities after heading update
                hdg_rad = math.radians(hdg)
                va_n = tas_fps * math.cos(hdg_rad)
                va_e = tas_fps * math.sin(hdg_rad)
                vg_n = va_n + wn_fps
                vg_e = va_e + we_fps
                gs_fps = math.hypot(vg_n, vg_e)
                gs_kt = gs_fps / 1.68781
                track_deg = _heading_from_track_components(vg_n, vg_e)
                drift_deg = _angle_diff_deg(track_deg, hdg)
                align_err = abs(_angle_diff_deg(track_deg, final_course_hdg))

        # In final, hold the desired ground track using wind correction (no banking, just correct heading)
        if phase == "final":
            hdg_cmd = wind_corrected_heading_for_track(final_course_hdg, tas_fps)
            hdg_err = _angle_diff_deg(hdg_cmd, hdg)
            hdg_step = max(-max_hdg_rate_dps * dt, min(max_hdg_rate_dps * dt, hdg_err))
            hdg = _wrap_360(hdg + hdg_step)

            # Recompute velocities after heading update
            hdg_rad = math.radians(hdg)
            va_n = tas_fps * math.cos(hdg_rad)
            va_e = tas_fps * math.sin(hdg_rad)
            vg_n = va_n + wn_fps
            vg_e = va_e + we_fps
            gs_fps = math.hypot(vg_n, vg_e)
            gs_kt = gs_fps / 1.68781
            track_deg = _heading_from_track_components(vg_n, vg_e)
            drift_deg = _angle_diff_deg(track_deg, hdg)
            align_err = abs(_angle_diff_deg(track_deg, final_course_hdg))

        # sink with bank penalty
        if abs(aob) > 0.1:
            n = compute_load_factor(abs(aob))
            glide_eff = straight_gr / max(n, 1.0)
        else:
            glide_eff = straight_gr
        glide_eff = max(2.0, glide_eff)

        vs_fps = tas_fps / glide_eff
        alt -= vs_fps * dt
        vs_fpm = vs_fps * 60.0

        # record
        record(
            gs_kt=gs_kt,
            aob_deg=(aob if abs(aob) > 0.1 else 0.0),
            vs_fpm=vs_fpm,
            track_deg=track_deg,
            drift_deg=drift_deg,
        )



        # Track best "almost-capture" state (used by bank optimizer on failures)
        if phase in ["turn2", "straight", "final"] or (phase == "turn1" and total_turn_1 >= float(min_turn_deg_before_capture)):
            behind_penalty = 20000.0 if along_ft <= 0.0 else 0.0
            miss_score = (
                abs(float(xtrack_ft)) +
                200.0 * abs(float(align_err)) +
                behind_penalty -
                0.5 * max(0.0, float(alt))
            )
            # heading/track alignment error to desired final course
           
            if best_miss is None or miss_score < best_miss["miss_score"]:
                best_miss = {
                    "miss_score": float(miss_score),
                    "xtrack_ft": float(xtrack_ft),
                    "align_err_deg": float(align_err),
                    "along_ft": float(along_ft),
                    "alt": float(max(0.0, alt)),
                    "time": float(t),
                    "phase": str(phase),
                }

        # touchdown or impact
        if alt <= 0.0:
            # If we had captured and were flying final, count touchdown as success
            if captured:
                return path, hover, _finalize_meta(
                    success=True,
                    reason="touchdown_after_capture",
                    impact_marker=(cur.latitude, cur.longitude),
                    xtrack_ft=xtrack_ft,
                    along_ft=along_ft,
                    align_err_deg=align_err,
                )
            return path, hover, _finalize_meta(
                success=False,
                reason="impact",
                impact_marker=(cur.latitude, cur.longitude),
                xtrack_ft=xtrack_ft,
                along_ft=along_ft,
                align_err_deg=align_err,
            )

        # phase transitions
        if phase == "reaction":
            reaction_remaining -= dt
            if reaction_remaining <= 0.0:
                phase = "turn1"

        elif phase == "turn1":
            # Exit to "straight" once minimum turn is met and intercept track is roughly achieved
            intercept_offset = -max(-1.0, min(1.0, float(xtrack_ft) / float(xtrack_intercept_scale_ft))) * float(intercept_max_deg)
            intercept_trk = _wrap_360(final_course_hdg + intercept_offset)
            intercept_err = abs(_angle_diff_deg(intercept_trk, track_deg))

            if total_turn_1 >= float(min_turn_deg_before_capture) and intercept_err <= float(intercept_angle_deg):
                phase = "straight"

        elif phase == "straight":
            # If already essentially captured, transition to final instead of ending
            if along_ft > 0.0:
                if align_err <= float(align_window_deg) and abs(xtrack_ft) <= float(centerline_xtol_ft):
                    captured = True
                    if captured_at_time is None:
                        captured_at_time = float(t)
                    phase = "final"
                else:
                    # Gate jink
                    if abs(xtrack_ft) <= float(xtrack_align_gate_ft) and float(along_ft) >= float(along_align_gate_ft):
                        phase = "turn2"

        elif phase == "turn2":
            # Smoothly capture, then transition to final and fly to touchdown
            hdg_tol = min(float(align_window_deg), float(jink_hdg_tol_deg)) if float(jink_hdg_tol_deg) > 0 else float(align_window_deg)
            xtol = min(float(centerline_xtol_ft), float(jink_xtrack_tol_ft)) if float(jink_xtrack_tol_ft) > 0 else float(centerline_xtol_ft)

            if align_err <= hdg_tol and abs(xtrack_ft) <= xtol and along_ft > 0.0:
                captured = True
                if captured_at_time is None:
                    captured_at_time = float(t)
                phase = "final"

            # If we are basically captured but jink is making it worse, go to final
            if abs(xtrack_ft) <= float(centerline_xtol_ft) and align_err <= float(align_window_deg) and along_ft > 0.0:
                captured = True
                if captured_at_time is None:
                    captured_at_time = float(t)
                phase = "final"

        # propagate position along GROUND TRACK
        step_nm = (gs_fps * dt) / FT_PER_NM
        cur = point_from(cur, track_deg, step_nm)

        # update geometry after motion for gates
        xtrack_ft, along_ft = _cross_track_to_centerline_ft(start_point, cur, final_course_hdg)
        align_err = abs(_angle_diff_deg(track_deg, final_course_hdg))

        t += dt

    # timeout
    xtrack_ft, along_ft = _cross_track_to_centerline_ft(start_point, cur, final_course_hdg)
    align_err = abs(_angle_diff_deg(track_deg, final_course_hdg)) if hover else None

    return path, hover, _finalize_meta(
        success=False,
        reason="timeout",
        impact_marker=(cur.latitude, cur.longitude),
        xtrack_ft=xtrack_ft,
        along_ft=along_ft,
        align_err_deg=align_err,
    )


DEFAULT_ALIGN_WINDOW_DEG = 10.0

def simulate_impossible_turn(
    start_point,
    runway_heading_deg: float,
    turn_dir: str,
    reaction_sec: float,
    start_ias_kias: float,
    altitude_agl: float,
    align_window_deg: float = DEFAULT_ALIGN_WINDOW_DEG,
    ac: dict = None,
    engine_option: str = None,
    weight_lbs: float = None,
    oat_c: float = None,
    altimeter_inhg: float = None,
    wind_dir: float = None,
    wind_speed: float = None,
    timestep_sec: float = 0.5,
    flap_config: str = "clean",
    prop_config: str = "windmilling",
    touchdown_elev_ft: float = 0.0,

    # Phase 1 intercept bank search
    bank_min_deg: float = 15.0,
    bank_max_deg: float = 45.0,
    bank_step_deg: float = 1.0,

    # Phase 2/3 gating
    intercept_angle_deg: float = 25.0,
    xtrack_align_gate_ft: float = 600.0,
    along_align_gate_ft: float = 2000.0,

    # Phase 3 jink control
    jink_bank_cap_deg: float = 15.0,
    jink_hdg_tol_deg: float = 3.0,
    jink_xtrack_tol_ft: float = 50.0,

    # Min altitude solver
    find_min_alt: bool = True,
    min_alt_floor_agl: float = 100.0,
    max_alt_ceiling_agl: float = 2000.0,
    min_alt_resolution_ft: float = 10.0,
):
    """
    Returns (path, hover, meta)

    meta includes:
      - success (bool)
      - reason (str)
      - impact_marker (lat, lon) or None
      - bank_deg (float) chosen intercept bank
      - jink_bank_cap_deg (float)
      - min_feasible_alt_agl (float or None)
      - flap_config / prop_config (canonical strings)
    """

    if start_point is None:
        return [], [], {"success": False, "reason": "no_start_point"}

    if ac is None:
        return [], [], {"success": False, "reason": "no_aircraft_data"}

    runway_hdg = float(runway_heading_deg or 0.0)
    turn_dir = "left" if str(turn_dir).strip().lower().startswith("l") else "right"

    # Canonicalize UI strings once, here
    flap_config = _canon_flap_config(flap_config)
    prop_config = _canon_prop_config(prop_config)

    # Safe numeric defaults (prevents float(None) crashes)
    weight_lbs_f = float(weight_lbs or 0.0)
    oat_c_f = float(oat_c if oat_c is not None else 15.0)
    altimeter_inhg_f = float(altimeter_inhg if altimeter_inhg is not None else 29.92)
    wind_dir_f = float(wind_dir or 0.0)
    wind_speed_f = float(wind_speed or 0.0)
    timestep_f = float(timestep_sec) if timestep_sec and float(timestep_sec) > 0 else 0.5
    reaction_f = float(reaction_sec or 0.0)
    start_ias_f = float(start_ias_kias or 0.0)
    altitude_agl_f = float(altitude_agl or 0.0)
    align_window_f = float(align_window_deg if align_window_deg is not None else DEFAULT_ALIGN_WINDOW_DEG)
    touchdown_elev_f = float(touchdown_elev_ft or 0.0)

    # If your aircraft model needs a valid engine_option for multi-engine OEI blocks,
    # do not guess. Let the sim fail cleanly instead of silently using the wrong data.
    if ac.get("engine_count", 1) > 1 and not engine_option:
        return [], [], {"success": False, "reason": "missing_engine_option_for_multiengine"}

    def eval_at(alt_agl: float, intercept_bank_deg: float):
        return _run_impossible_turn_once(
            start_point=start_point,
            runway_heading_deg=runway_hdg,
            turn_dir=turn_dir,
            bank_deg=float(intercept_bank_deg),
            reaction_sec=reaction_f,
            start_ias_kias=start_ias_f,
            altitude_agl=float(alt_agl),
            align_window_deg=align_window_f,
            ac=ac,
            engine_option=engine_option,
            weight_lbs=weight_lbs_f,
            oat_c=oat_c_f,
            altimeter_inhg=altimeter_inhg_f,
            wind_dir=wind_dir_f,
            wind_speed=wind_speed_f,
            timestep_sec=timestep_f,
            flap_config=flap_config,
            prop_config=prop_config,
            touchdown_elev_ft=touchdown_elev_f,
            intercept_angle_deg=float(intercept_angle_deg),
            xtrack_align_gate_ft=float(xtrack_align_gate_ft),
            along_align_gate_ft=float(along_align_gate_ft),
            jink_bank_cap_deg=float(jink_bank_cap_deg),
            jink_hdg_tol_deg=float(jink_hdg_tol_deg),
            jink_xtrack_tol_ft=float(jink_xtrack_tol_ft),
        )

    def _turn1_time_sec(hover: list) -> float:
        if not hover:
            return 0.0
        dt = timestep_f
        return dt * sum(1 for h in hover if str(h.get("phase", "")).lower() == "turn1")

    def score_success(meta: dict, hover: list, intercept_bank_deg: float) -> float:
        xerr = abs(float(meta.get("final_xtrack_ft", 1e9)))
        herr = abs(float(meta.get("final_hdg_err_deg", 1e9)))

        total_t = float(meta.get("time_sec", len(hover) * timestep_f))
        t_turn1 = _turn1_time_sec(hover)

        bank_penalty = float(intercept_bank_deg)
        return float(
            - (xerr * 10.0)
            - (herr * 80.0)
            - (bank_penalty * 6.0)
            - (t_turn1 * 2.0)
            - (total_t * 0.25)
        )

    def score_failure(meta: dict, hover: list, intercept_bank_deg: float) -> float:
        best_xerr = abs(float(meta.get("best_xtrack_ft", 1e9)))
        total_t = float(meta.get("time_sec", len(hover) * timestep_f))
        return float(
            - (best_xerr * 10.0)
            + (total_t * 1.0)
            - (float(intercept_bank_deg) * 2.0)
        )

    def better_choice(candidate: dict, incumbent: dict) -> bool:
        if incumbent is None:
            return True

        eps = 1e-6
        cs = float(candidate["score"])
        iscore = float(incumbent["score"])

        if cs > iscore + eps:
            return True

        # tie break: smaller bank wins if essentially same score
        if abs(cs - iscore) <= 0.25:
            return float(candidate["bank"]) < float(incumbent["bank"])

        return False

    def find_best_bank_for_alt(alt_agl: float):
        """
        Safety-biased chooser:
          - Iterate bank low to high.
          - Return the FIRST bank that produces a true success AND meets alignment thresholds.
          - If no such bank exists, return the best failure (closest + longest).
        """
        best_fail = None

        b = float(bank_min_deg)
        bmax = float(bank_max_deg)
        bstep = max(0.1, float(bank_step_deg))

        while b <= bmax + 1e-9:
            path, hover, meta = eval_at(alt_agl, b)
            meta = meta if isinstance(meta, dict) else {}

            if meta.get("success", False) and meta.get("captured", False):
                xerr = abs(float(meta.get("final_xtrack_ft", 1e9)))
                herr = abs(float(meta.get("final_hdg_err_deg", 1e9)))

                # Must meet the actual alignment definition.
                # Use the same tolerances the sim uses to declare capture.
                xtol = float(meta.get("centerline_xtol_ft", 150.0))
                if (xerr <= xtol) and (herr <= float(align_window_f)):
                    return {"bank": b, "path": path, "hover": hover, "meta": meta, "score": 0.0}

            # Track best failure as fallback
            sf = score_failure(meta, hover, b)
            candf = {"bank": b, "path": path, "hover": hover, "meta": meta, "score": sf}
            if better_choice(candf, best_fail):
                best_fail = candf

            b += bstep

        return best_fail

    # Primary run at requested altitude with optimized intercept bank
    best_run = find_best_bank_for_alt(altitude_agl_f)
    if not best_run:
        return [], [], {"success": False, "reason": "bank_search_failed"}

    path = best_run["path"]
    hover = best_run["hover"]
    meta = best_run["meta"] if isinstance(best_run["meta"], dict) else {}

    meta["bank_deg"] = float(best_run["bank"])
    meta["jink_bank_cap_deg"] = float(jink_bank_cap_deg)
    meta["flap_config"] = str(flap_config)
    meta["prop_config"] = str(prop_config)

    # Min feasible altitude search (binary)
    min_feasible = None
    if find_min_alt:
        low = float(min_alt_floor_agl)
        high = float(max_alt_ceiling_agl)
        res = max(1.0, float(min_alt_resolution_ft))

        hi_run = find_best_bank_for_alt(high)
        if hi_run and hi_run["meta"].get("success", False):
            lo_run = find_best_bank_for_alt(low)
            if lo_run and lo_run["meta"].get("success", False):
                min_feasible = low
            else:
                while (high - low) > res:
                    mid = 0.5 * (low + high)
                    mid_run = find_best_bank_for_alt(mid)
                    if mid_run and mid_run["meta"].get("success", False):
                        high = mid
                    else:
                        low = mid
                min_feasible = high

    meta["min_feasible_alt_agl"] = min_feasible
    return path, hover, meta

def find_required_aob_for_arc_fit(
    arc_start,
    arc_angle_rad,  # kept for compatibility, not used
    target_point,
    start_heading,
    turn_dir,
    tas_knots,
    max_bank_deg=60,
    tolerance_pct=0.002,
    roll_in_correction_total=0.0  # kept for compatibility, not used
):
    """
    Returns (best_aob_deg, best_arc_angle_deg) that best fits the chord between arc_start and target_point.

    Critical: All geometry in this function is done in meters for geodesic calls.
    """
    # Normalize turn_dir to +/- 1
    if isinstance(turn_dir, str):
        td = turn_dir.strip().lower()
        turn_dir = -1 if td.startswith("l") else 1
    else:
        turn_dir = -1 if float(turn_dir) < 0 else 1

    # Use SI inside this function to match geodesic meters
    g_mps2 = 9.80665
    tas_mps = float(tas_knots) * 0.514444

    def compute_turn_radius_m(aob_deg: float) -> float:
        aob_deg = max(0.1, float(aob_deg))
        return (tas_mps ** 2) / (g_mps2 * math.tan(math.radians(aob_deg)))

    def arc_endpoint(center_radius_m: float, arc_angle_deg: float, entry_heading_deg: float):
        # Center is 90 degrees to the turn side from the entry heading
        center_bearing = (float(entry_heading_deg) + turn_dir * 90.0) % 360.0
        center = geo_dist(meters=center_radius_m).destination(arc_start, center_bearing)

        start_angle = calculate_initial_compass_bearing(center, arc_start)
        end_angle = (start_angle + turn_dir * float(arc_angle_deg)) % 360.0
        return geo_dist(meters=center_radius_m).destination(center, end_angle)

    chord_dist_m = geo_dist(
        (arc_start.latitude, arc_start.longitude),
        (target_point.latitude, target_point.longitude)
    ).meters

    tolerance_m = max(min(chord_dist_m * float(tolerance_pct), 5.0), 1.0)

    best_aob = None
    best_arc_angle_deg = None
    best_error = float("inf")

    for aob in np.arange(5.0, float(max_bank_deg) + 0.0001, 0.05):
        Rm = compute_turn_radius_m(aob)

        # For a circle, chord = 2R sin(theta/2)  -> theta = 2 asin(chord/(2R))
        ratio = chord_dist_m / max(2.0 * Rm, 1e-6)
        ratio = max(0.0, min(1.0, ratio))

        arc_rad = 2.0 * math.asin(ratio)
        arc_rad = min(arc_rad, math.radians(175.0))
        arc_deg = math.degrees(arc_rad)

        endpoint = arc_endpoint(Rm, arc_deg, start_heading)
        error = geo_dist(
            (endpoint.latitude, endpoint.longitude),
            (target_point.latitude, target_point.longitude)
        ).meters

        if error < best_error:
            best_error = error
            best_aob = float(aob)
            best_arc_angle_deg = float(arc_deg)

        if error <= tolerance_m:
            break

    return (best_aob if best_aob is not None else float(max_bank_deg),
            best_arc_angle_deg if best_arc_angle_deg is not None else 90.0)

def simulate_glide_path_to_target(
    start_point,
    start_heading,
    touchdown_point,
    touchdown_heading,
    ac,
    engine_option,
    weight_lbs,
    flap_config,
    prop_config,
    oat_c,
    altimeter_inhg,
    wind_dir,
    wind_speed,
    start_ias_kias,
    altitude_agl,
    pattern_dir,
    selected_airport_elev_ft,
    max_bank_deg=45,
    timestep_sec=0.5,
):
    """
    Power-Off 180 glide model with THREE segments:
      1) Straight downwind at start_heading
      2) Constant-bank turn (higher sink)
      3) Straight final at touchdown_heading

    ***GROUND-REFERENCE MODE***
      - Ground track geometry (downwind length, arc radius/angle, final length)
        is computed as in still air and does NOT change with wind.
      - At each step we move along that desired ground track using TAS.
      - Wind ONLY affects:
          * required heading (crab angle / drift)
          * groundspeed
          * time
          * vertical speed (fpm)
    """

    # Basic guards
    if start_point is None or touchdown_point is None:
        return [], [], None
    if altitude_agl is None or altitude_agl <= 0:
        start_latlon = [start_point.latitude, start_point.longitude]
        hover = [{
            "alt": 0.0,
            "tas": 0.0,
            "time": 0.0,
            "aob": 0.0,
            "vs": 0.0,
            "segment": "ground",
            "track": start_heading,
            "heading": start_heading,
            "drift": 0.0,
        }]
        return [start_latlon], hover, start_latlon

    # --- Aircraft best glide + base glide ratio ---
    if ac.get("engine_count", 1) > 1:
        perf_block = ac["engine_options"][engine_option]["oei_performance"][f"{flap_config}_up"][prop_config]
        best_glide_kias = perf_block["best_glide_speed_kias"]
        base_glide_ratio = ac["single_engine_limits"]["best_glide_ratio"]
    else:
        best_glide_kias = ac["single_engine_limits"]["best_glide"]
        base_glide_ratio = ac["single_engine_limits"]["best_glide_ratio"]

    gear_type = ac.get("gear_type", "fixed")

    # --- Environment: pressure altitude + density ---
    field_elev_ft = float(selected_airport_elev_ft or 0.0)
    alt_msl_ft = field_elev_ft + float(altitude_agl or 0.0)
    pressure_alt_ft = compute_pressure_altitude(alt_msl_ft, float(altimeter_inhg))
    rho = compute_air_density(pressure_alt_ft, float(oat_c))



    # Straight-flight glide ratio (config + density adjusted)
    straight_gr = compute_glide_ratio(base_glide_ratio, flap_config, gear_type, prop_config)
    straight_gr = adjust_glide_ratio_for_density(straight_gr, rho)
    straight_gr = max(3.0, min(straight_gr, 25.0))

    # Use pressure altitude + OAT for TAS (Denver vs sea level distinction)
    ias_for_geometry = float(start_ias_kias or best_glide_kias)
    tas_knots = compute_true_airspeed(ias_for_geometry, pressure_alt_ft, float(oat_c))
    tas_fps = max(1.0, knots_to_fps(tas_knots))

    # --- Arc geometry & required bank angle (STILL AIR) --------------------
    heading_vec = ((touchdown_heading - start_heading + 540.0) % 360.0) - 180.0
    arc_angle_deg_geom = max(abs(heading_vec), 5.0)
    arc_angle_rad_geom = math.radians(arc_angle_deg_geom)

    chord_m = geo_dist(
        (start_point.latitude, start_point.longitude),
        (touchdown_point.latitude, touchdown_point.longitude),
    ).meters

    bank_deg = max_bank_deg  # fallback
    if chord_m > 1.0 and math.sin(arc_angle_rad_geom / 2.0) > 1e-6:
        R_geom_m = chord_m / (2.0 * math.sin(arc_angle_rad_geom / 2.0))
        R_geom_ft = R_geom_m / 0.3048
        bank_from_geom = compute_required_bank(tas_knots, R_geom_ft)
        bank_deg = min(max_bank_deg, max(5.0, bank_from_geom))
    bank_rad = math.radians(bank_deg)

    # Turn radius using computed bank (air-mass geometry)
    R_turn_ft = compute_turn_radius(tas_knots, bank_deg)  # ft

    # Turn angle needed between downwind and final (ground track)
    if pattern_dir == "left":
        delta = (start_heading - touchdown_heading + 360.0) % 360.0
        if delta > 180.0:
            delta -= 360.0
        arc_angle_deg = abs(delta)
        turn_sign = -1.0
    else:
        delta = (touchdown_heading - start_heading + 360.0) % 360.0
        if delta > 180.0:
            delta -= 360.0
        arc_angle_deg = abs(delta)
        turn_sign = 1.0

    arc_angle_deg = min(180.0, arc_angle_deg if arc_angle_deg > 1.0 else 180.0)
    arc_angle_rad = math.radians(arc_angle_deg)

    # Turn glide ratio (worse due to n>1)
    n = compute_load_factor(bank_deg)
    turn_gr = straight_gr / n
    turn_gr = max(2.0, min(turn_gr, straight_gr))

    # Arc length & altitude loss (STILL AIR)
    arc_length_ft = R_turn_ft * arc_angle_rad
    alt_loss_turn_ft = arc_length_ft / turn_gr

    h0 = altitude_agl

    # Make sure these are always defined
    L_dw_ft = 0.0
    L_fn_ft = 0.0

    if alt_loss_turn_ft >= h0:
        # Not enough altitude to finish the planned turn
        alt_for_turn = h0
        frac = max(0.0, min(1.0, alt_for_turn / alt_loss_turn_ft))
        arc_angle_rad *= frac
        arc_angle_deg *= frac
        arc_length_ft *= frac
        L_dw_ft = 0.0
        L_fn_ft = 0.0
    else:
        h_rem = h0 - alt_loss_turn_ft

        # Even split: 50/50 altitude for downwind and final
        h_dw = h_rem * 0.5
        h_fn = h_rem - h_dw

        L_dw_ft = h_dw * straight_gr
        L_fn_ft = h_fn * straight_gr

    downwind_leg_ft = L_dw_ft
    final_leg_ft = L_fn_ft

    # --- Wind model: FROM direction in nav terms (used ONLY for hover data) -
    wind_speed_knots = float(wind_speed or 0.0)

    def drift_corrected(wind_from_deg, track_hdg_deg):
        """
        wind_from_deg: standard pilot-style *FROM* direction (e.g. 180 = wind from south)
        track_hdg_deg: desired ground track (course) we want to fly

        Returns (groundspeed_knots, heading_deg, drift_deg) to hold that track.
        """
        if wind_speed_knots <= 0.1:
            return tas_knots, track_hdg_deg, 0.0

        # Convert FROM → TO so the vector math is correct
        wind_to_deg = (wind_from_deg + 180.0) % 360.0

        # Angle between wind-TO direction and desired track
        alpha_deg = (wind_to_deg - track_hdg_deg + 360.0) % 360.0
        alpha = math.radians(alpha_deg)

        cross = wind_speed_knots * math.sin(alpha)
        head = wind_speed_knots * math.cos(alpha)

        # Clamp to avoid asin domain issues
        cross_clamped = max(min(cross, tas_knots * 0.99), -tas_knots * 0.99)
        drift_rad = math.asin(cross_clamped / tas_knots)
        drift_deg = math.degrees(drift_rad)

        # Heading you actually point the nose to hold that ground track
        heading_deg = (track_hdg_deg + drift_deg + 360.0) % 360.0

        along_air = tas_knots * math.cos(drift_rad)
        gs_knots = along_air + head
        gs_knots = max(5.0, gs_knots)

        return gs_knots, heading_deg, drift_deg

    # --- Main integration loop (geometry from TAS, not GS) -----------------
    path = []
    hover = []

    lat = start_point.latitude
    lon = start_point.longitude
    alt_ft = h0
    time_s = 0.0

    dist_dw_ft = 0.0
    dist_arc_ft = 0.0
    dist_fn_ft = 0.0
    arc_accum_deg = 0.0

    if L_dw_ft > 1.0:
        segment = "downwind"
    elif arc_length_ft > 1.0:
        segment = "arc"
    else:
        segment = "final"

    downwind_leg_ft = L_dw_ft
    final_leg_ft = L_fn_ft

    impact_marker = None
    max_steps = 4000

    for _ in range(max_steps):
        # Desired ground track and glide ratio for this segment
        if segment == "downwind":
            track_hdg = start_heading
            seg_gr = straight_gr
            aob_geom = 0.0        # no turning
        elif segment == "arc":
            track_hdg = start_heading + turn_sign * arc_accum_deg
            seg_gr = turn_gr
            aob_geom = bank_deg   # geometric bank used for radius
        else:  # final
            track_hdg = touchdown_heading
            seg_gr = straight_gr
            aob_geom = 0.0

        track_hdg = (track_hdg + 360.0) % 360.0

        # Step distance along ground track is based on TAS only (geometry)
        ds_ft = tas_fps * timestep_sec
        if ds_ft <= 0.1:
            break

        # How long does that take in the given wind? (GS-based time)
        gs_knots, heading_deg, drift_deg = drift_corrected(wind_dir, track_hdg)
        gs_fps = knots_to_fps(gs_knots)
        dt = ds_ft / gs_fps if gs_fps > 1e-3 else timestep_sec

        # Altitude change based on ground distance and glide ratio
        dh_ft = ds_ft / seg_gr
        alt_ft = max(0.0, alt_ft - dh_ft)

        # Move along the desired ground track
        ds_nm = ds_ft / 6076.12
        new_point = point_from(GeoPoint(lat, lon), track_hdg, ds_nm)
        lat, lon = new_point.latitude, new_point.longitude

        time_s += dt

        # Segment bookkeeping (also distance in feet is TAS-based)
        if segment == "downwind":
            dist_dw_ft += ds_ft
            if dist_dw_ft >= L_dw_ft and arc_length_ft > 1.0:
                segment = "arc"
        elif segment == "arc":
            dist_arc_ft += ds_ft
            dpsi_rad = ds_ft / R_turn_ft
            arc_accum_deg += math.degrees(dpsi_rad)
            if arc_accum_deg >= arc_angle_deg or dist_arc_ft >= arc_length_ft:
                segment = "final"
        else:
            dist_fn_ft += ds_ft

        # Vertical speed (fpm)
        vs_fpm = -(dh_ft / dt) * 60.0 if dt > 1e-3 else 0.0

        seg_for_display = segment
        
        # --- Effective AOB for display (depends on groundspeed in the arc) ---
        if seg_for_display == "arc":
            gs_fps_for_bank = knots_to_fps(gs_knots)
            bank_eff_rad = math.atan((gs_fps_for_bank ** 2) / (g * max(R_turn_ft, 1.0)))
            aob_display = math.degrees(bank_eff_rad)
        else:
            aob_display = 0.0

        path.append([lat, lon])
        hover.append({
            "alt": alt_ft,
            "tas": tas_knots,
            "time": time_s,
            "aob": aob_display,     # <- wind-dependent AOB now
            "vs": vs_fpm,
            "segment": segment,
            "track": track_hdg,
            "heading": heading_deg,
            "drift": drift_deg,
            "gs": gs_knots,
        })

        # Touchdown capture on final
        dist_to_td_ft = geo_dist(
            (lat, lon),
            (touchdown_point.latitude, touchdown_point.longitude)
        ).feet

        # Snap to touchdown any time we're on the final segment and
        # we pass within a small radius of the touchdown point.
        # We don't require alt_ft <= 50 here, because the energy
        # split is approximate and was causing consistent overshoots.
        if segment == "final" and dist_to_td_ft <= max(150.0, ds_ft * 1.5):
            lat = touchdown_point.latitude
            lon = touchdown_point.longitude
            alt_ft = 0.0
            time_s += dt
            vs_fpm = -(dh_ft / dt) * 60.0 if dt > 1e-3 else 0.0

            path.append([lat, lon])
            hover.append({
                "alt": alt_ft,
                "tas": tas_knots,
                "time": time_s,
                "aob": 0.0,
                "vs": vs_fpm,
                "segment": "final",
                "track": touchdown_heading,
                "heading": heading_deg,
                "drift": drift_deg,
                "gs": gs_knots,
            })
            impact_marker = None
            break

        if alt_ft <= 0.0:
            dist_to_td_ft = geo_dist(
                (lat, lon),
                (touchdown_point.latitude, touchdown_point.longitude)
            ).feet

            snap_radius_ft = max(300.0, ds_ft * 2.0)

            if segment == "final" and dist_to_td_ft <= snap_radius_ft:
                lat = touchdown_point.latitude
                lon = touchdown_point.longitude
                alt_ft = 0.0
                time_s += dt

                path.append([lat, lon])
                hover.append({
                    "alt": alt_ft,
                    "tas": tas_knots,
                    "time": time_s,
                    "aob": 0.0,
                    "vs": vs_fpm,
                    "segment": "final",
                    "track": touchdown_heading,
                    "heading": heading_deg,
                    "drift": drift_deg,
                    "gs": gs_knots,
                })
                impact_marker = None
            else:
                impact_marker = [lat, lon]

            break

        # Overshoot final badly → treat as miss/impact
        if segment == "final" and dist_fn_ft > max(L_fn_ft * 1.5, 3 * straight_gr * h0):
            impact_marker = [lat, lon]
            break


    return path, hover, impact_marker
    
def render_hover_polyline(path, hover_data, color="blue", weight=3):
    import dash_leaflet as dl
    from dash import html
    return dl.Polyline(
        positions=path,
        color=color,
        weight=weight,
        children=[
            dl.Tooltip(
                html.Div([
                    html.Div(f"{pt['alt']:.0f} ft AGL"),
                    html.Div(f"{pt['tas']:.0f} kt"),
                    html.Div(f"{pt['time']:.1f} sec"),
                    html.Div(f"{pt['aob']:.1f}° AOB"),
                    html.Div(f"{pt['vs']:.0f} fpm")
                ])
            ) for pt in hover_data[::5]
        ]
    )

def point_from(p, bearing_deg, dist_nm):
    return distance(nautical=dist_nm).destination(p, bearing_deg)


def calculate_initial_compass_bearing(pointA, pointB):
    lat1 = math.radians(pointA.latitude)
    lat2 = math.radians(pointB.latitude)
    diff_long = math.radians(pointB.longitude - pointA.longitude)
    x = math.sin(diff_long) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_long)
    )
    initial_bearing = math.atan2(x, y)
    return (math.degrees(initial_bearing) + 360) % 360

def simulate_tight_overhead_orbit(
    start_lat,
    start_lon,
    alt_ft,
    tas_knots,
    straight_gr,
    bank_deg,            # <<< use explicit bank
    turn_sign,
    entry_track_hdg,
    wind_dir,
    wind_speed_knots,
    timestep_sec=0.5,
    required_alt_loss_ft=None,
):
    """
    One constant–radius 360° orbit around a center placed on the runway side of downwind,
    using the provided bank angle.
    """
    from geopy import Point as GeoPoint
    from geopy.distance import geodesic as geo_dist

    g = 32.174
    tas_fps = tas_knots * 1.68781

    # Clamp bank to something sane
    bank_deg = float(bank_deg)
    bank_deg = max(10.0, min(60.0, bank_deg))
    bank_rad = math.radians(bank_deg)

    # Radius from bank and TAS (air-mass geometry)
    R_ft = (tas_fps ** 2) / (g * math.tan(bank_rad))

    # Glide ratio in the orbit (load-factor degraded)
    n_orbit = compute_load_factor(bank_deg)
    turn_gr_orbit = straight_gr / max(n_orbit, 1.0)
    turn_gr_orbit = max(2.0, min(turn_gr_orbit, straight_gr))

    # Place center on the runway side of downwind so path stays pattern-like
    start_pt = GeoPoint(start_lat, start_lon)
    center_bearing = (entry_track_hdg + turn_sign * 90.0) % 360.0
    center = geo_dist(feet=R_ft).destination(start_pt, center_bearing)

    # Radial bearing from center to start point
    start_angle = calculate_initial_compass_bearing(center, start_pt)

    ds_ft = tas_fps * timestep_sec
    if ds_ft <= 0.1:
        return [[start_lat, start_lon]], [], alt_ft, 0.0

    # How much arc is needed to bleed the requested altitude
    # alt_loss = (R_ft * theta_rad) / turn_gr_orbit  =>  theta_rad = alt_loss * turn_gr_orbit / R_ft
    if required_alt_loss_ft is None:
        # fall back to full circle if nothing specified
        arc_angle_rad = 2.0 * math.pi
    else:
        arc_angle_rad = required_alt_loss_ft * turn_gr_orbit / max(R_ft, 1.0)
        arc_angle_rad = max(0.0, min(arc_angle_rad, 2.0 * math.pi))

    arc_length_ft = R_ft * arc_angle_rad
    n_steps = max(1, int(arc_length_ft / ds_ft))

    # Avoid division by zero
    if n_steps <= 0 or arc_angle_rad <= 0.0:
        return [[start_lat, start_lon]], [], alt_ft, 0.0

    dtheta_rad = arc_angle_rad / n_steps
    dtheta_deg = math.degrees(dtheta_rad) * turn_sign

    wind_speed = float(wind_speed_knots or 0.0)

    def drift_corrected(track_hdg_deg):
        if wind_speed <= 0.1:
            return tas_knots, track_hdg_deg, 0.0
        wind_from_deg = wind_dir
        wind_to_deg   = (wind_from_deg + 180.0) % 360.0

        alpha_deg = (wind_to_deg - track_hdg_deg + 360.0) % 360.0
        alpha     = math.radians(alpha_deg)

        cross = wind_speed * math.sin(alpha)
        head  = wind_speed * math.cos(alpha)

        cross_clamped = max(min(cross, tas_knots * 0.99), -tas_knots * 0.99)
        drift_rad     = math.asin(cross_clamped / tas_knots)
        drift_deg     = math.degrees(drift_rad)

        heading_deg = (track_hdg_deg + drift_deg + 360.0) % 360.0

        along_air = tas_knots * math.cos(drift_rad)
        gs_knots  = along_air + head
        gs_knots  = max(5.0, gs_knots)
        return gs_knots, heading_deg, drift_deg

    path = [[start_lat, start_lon]]
    hover = []
    time_s = 0.0
    h = alt_ft
    angle = start_angle

    for _ in range(n_steps):
        angle = (angle + dtheta_deg) % 360.0
        track_hdg = (angle + 90.0 * turn_sign) % 360.0

        ds_step_ft = R_ft * abs(dtheta_rad)
        gs_knots, heading_deg, drift_deg = drift_corrected(track_hdg)
        gs_fps = knots_to_fps(gs_knots)
        dt = ds_step_ft / gs_fps if gs_fps > 1e-3 else timestep_sec

        dh_ft = ds_step_ft / turn_gr_orbit
        h = max(0.0, h - dh_ft)

        ds_nm = ds_step_ft / 6076.12
        cur_pt = GeoPoint(path[-1][0], path[-1][1])
        new_pt = distance(nautical=ds_nm).destination(cur_pt, track_hdg)

        path.append([new_pt.latitude, new_pt.longitude])
        time_s += dt

        vs_fpm = -(dh_ft / dt) * 60.0 if dt > 1e-3 else 0.0

        # Effective AOB for display using GS and orbit radius
        bank_eff_rad = math.atan((gs_fps ** 2) / (g * max(R_ft, 1.0)))
        aob_display = math.degrees(bank_eff_rad)


        hover.append({
            "alt": h,
            "tas": tas_knots,
            "time": time_s,
            "aob": aob_display,
            "vs": vs_fpm,
            "segment": "engineout",          # <- match main sim
            "track": track_hdg,
            "heading": heading_deg,
            "drift": drift_deg,
            "gs": gs_knots,
            "note": "tight_orbit",           # optional label if you want to see it in hover
        })

        if h <= 0.0:
            break

    return path, hover, h, time_s

def simulate_engineout_glide(
    start_point,
    start_heading,              # kept for interface; not heavily used
    touchdown_point,
    touchdown_heading,
    ac,
    engine_option,
    weight_lbs,
    flap_config,
    prop_config,
    oat_c,
    altimeter_inhg,
    wind_dir,
    wind_speed,
    start_ias_kias,
    altitude_agl,
    touchdown_elev_ft,
    selected_airport_elev_ft,    # unused here but kept for signature
    pattern_dir="left",
    max_bank_deg=45,
    timestep_sec=0.5,
):
    if start_point is None or touchdown_point is None:
        return [], [], None
    if altitude_agl is None or altitude_agl <= 0:
        sp = [start_point.latitude, start_point.longitude]
        hover = [{
            "alt": 0.0, "tas": 0.0, "time": 0.0, "aob": 0.0, "vs": 0.0,
            "segment": "ground", "track": 0.0, "heading": 0.0, "drift": 0.0, "gs": 0.0,
        }]
        return [sp], hover, sp

    # -------- Performance first (defines best_glide_kias) --------
    if ac.get("engine_count", 1) > 1:
        perf_block       = ac["engine_options"][engine_option]["oei_performance"][f"{flap_config}_up"][prop_config]
        best_glide_kias  = perf_block["best_glide_speed_kias"]
        base_glide_ratio = ac["single_engine_limits"]["best_glide_ratio"]
    else:
        best_glide_kias  = ac["single_engine_limits"]["best_glide"]
        base_glide_ratio = ac["single_engine_limits"]["best_glide_ratio"]

    gear_type = ac.get("gear_type", "fixed")

    # -------- Environment (now safe) --------
    field_elev_ft   = float(touchdown_elev_ft or 0.0)
    alt_msl_ft      = field_elev_ft + float(altitude_agl or 0.0)
    pressure_alt_ft = compute_pressure_altitude(alt_msl_ft, float(altimeter_inhg))
    rho             = compute_air_density(pressure_alt_ft, float(oat_c))

    # -------- Glide + TAS --------
    straight_gr = compute_glide_ratio(base_glide_ratio, flap_config, gear_type, prop_config)
    straight_gr = adjust_glide_ratio_for_density(straight_gr, rho)
    straight_gr = max(3.0, min(straight_gr, 25.0))

    tas_knots = compute_true_airspeed(best_glide_kias, pressure_alt_ft, float(oat_c))
    tas_fps   = max(1.0, knots_to_fps(tas_knots))

    if ac.get("engine_count", 1) > 1:
        perf_block       = ac["engine_options"][engine_option]["oei_performance"][f"{flap_config}_up"][prop_config]
        best_glide_kias  = perf_block["best_glide_speed_kias"]
        base_glide_ratio = ac["single_engine_limits"]["best_glide_ratio"]
    else:
        best_glide_kias  = ac["single_engine_limits"]["best_glide"]
        base_glide_ratio = ac["single_engine_limits"]["best_glide_ratio"]

    gear_type = ac.get("gear_type", "fixed")

    straight_gr = compute_glide_ratio(base_glide_ratio, flap_config, gear_type, prop_config)
    straight_gr = adjust_glide_ratio_for_density(straight_gr, rho)
    straight_gr = max(3.0, min(straight_gr, 25.0))

    tas_knots = compute_true_airspeed(best_glide_kias, pressure_alt_ft, oat_c)
    tas_fps   = max(1.0, knots_to_fps(tas_knots))

    LOWKEY_MIN_DIST_FT = FINAL_MIN_DIST_NM * 6076.12
    LOWKEY_MAX_DIST_FT = FINAL_MAX_DIST_NM * 6076.12
    ORBIT_ONSET_RATIO  = 2.0

    LOWKEY_ALT_MIN_FT = 300.0
    LOWKEY_ALT_MAX_FT = 1500.0

    last_track_hdg = None

    # Only treat geometry as "pattern" when inside this horizontal range
    NEAR_PATTERN_MAX_DIST_FT = LOWKEY_MAX_DIST_FT * 2.0  # about 3 nm in your settings

    # Minimum altitude where a PO-180 style pattern makes sense
    MIN_PO180_ALT_FT = 700.0

    turn_sign = -1.0 if pattern_dir == "left" else +1.0

    # Abeam radial for pattern side
    if pattern_dir == "left":
        abeam_ref_bearing = (touchdown_heading - 90.0) % 360.0
    else:
        abeam_ref_bearing = (touchdown_heading + 90.0) % 360.0

    downwind_track = (touchdown_heading + 180.0) % 360.0

    wind_speed_knots = float(wind_speed or 0.0)

    def drift_corrected(track_hdg_deg):
        if wind_speed_knots <= 0.1:
            return tas_knots, track_hdg_deg, 0.0
        wind_from_deg = wind_dir
        wind_to_deg   = (wind_from_deg + 180.0) % 360.0
        alpha_deg = (wind_to_deg - track_hdg_deg + 360.0) % 360.0
        alpha     = math.radians(alpha_deg)
        cross = wind_speed_knots * math.sin(alpha)
        head  = wind_speed_knots * math.cos(alpha)
        cross_clamped = max(min(cross, tas_knots * 0.99), -tas_knots * 0.99)
        drift_rad     = math.asin(cross_clamped / tas_knots)
        drift_deg     = math.degrees(drift_rad)
        heading_deg   = (track_hdg_deg + drift_deg + 360.0) % 360.0
        along_air = tas_knots * math.cos(drift_rad)
        gs_knots  = along_air + head
        gs_knots  = max(5.0, gs_knots)
        return gs_knots, heading_deg, drift_deg

    def shortest_angle_diff(a, b):
        return (a - b + 180.0) % 360.0 - 180.0

    def blend_heading(base_hdg, target_hdg, weight):
        diff = shortest_angle_diff(target_hdg, base_hdg)
        return (base_hdg + weight * diff + 360.0) % 360.0

    # -------- Integration state --------
    path  = []
    hover = []

    lat         = start_point.latitude
    lon         = start_point.longitude
    alt_ft      = float(altitude_agl or 0.0)
    initial_alt = max(alt_ft, 1.0)
    time_s      = 0.0
    max_steps   = 8000
    impact_marker = None
    did_tight_orbit = False
    ABEAM_LOCKOUT = False

    path.append([lat, lon])
    hover.append({
        "alt": alt_ft, "tas": tas_knots, "time": time_s, "aob": 0.0, "vs": 0.0,
        "segment": "engineout", "track": 0.0, "heading": 0.0, "drift": 0.0,
        "gs": 0.0, "note": "start",
    })

    while max_steps > 0:
        max_steps -= 1

        dist_to_td_ft = geo_dist(
            (lat, lon),
            (touchdown_point.latitude, touchdown_point.longitude)
        ).feet

        track_to_td = calculate_initial_compass_bearing(
            GeoPoint(lat, lon), touchdown_point
        )
        radial_bearing = calculate_initial_compass_bearing(
            touchdown_point, GeoPoint(lat, lon)
        )

        # Range and altitude gating for pattern work
        near_pattern = dist_to_td_ft <= NEAR_PATTERN_MAX_DIST_FT
        enough_alt_for_po180 = alt_ft >= MIN_PO180_ALT_FT

        # --------------------------------------------------------
        # 1) Energy-based desired radius R_des(h)
        # --------------------------------------------------------
        if alt_ft > FINAL_CROSSING_HEIGHT_FT:
            R_energy = (alt_ft - FINAL_CROSSING_HEIGHT_FT) * straight_gr
        else:
            R_energy = LOWKEY_MIN_DIST_FT

        R_des = max(LOWKEY_MIN_DIST_FT, min(LOWKEY_MAX_DIST_FT, R_energy))

        dist_err_ft = dist_to_td_ft - R_des
        max_err_ft  = max(R_des, 1.0)
        err_norm    = max(-1.0, min(1.0, dist_err_ft / max_err_ft))

        in_lowkey_band = LOWKEY_ALT_MIN_FT <= alt_ft <= LOWKEY_ALT_MAX_FT

        # --- Tight orbit capability at max bank ---
        R_bank_ft = (tas_fps ** 2) / (g * math.tan(math.radians(max_bank_deg)))
        R_orbit_ft = max(LOWKEY_MIN_DIST_FT, min(R_des, R_bank_ft))

        n_orbit       = compute_load_factor(max_bank_deg)
        turn_gr_orbit = straight_gr / max(n_orbit, 1.0)
        turn_gr_orbit = max(2.0, min(turn_gr_orbit, straight_gr))

        orbit_circ_ft    = 2.0 * math.pi * R_orbit_ft
        alt_for_orbit_ft = orbit_circ_ft / max(turn_gr_orbit, 1.0)

        alt_margin_ft = 100.0
        have_energy_for_orbit = alt_ft >= (alt_for_orbit_ft + alt_margin_ft)

        # --------------------------------------------------------
        # 2) Direct vs orbit blend, with explicit TIGHT-ORBIT mode
        # --------------------------------------------------------
        dist_ratio = dist_to_td_ft / max(R_des, 1.0)
        f_orbit = 1.0 - dist_ratio / ORBIT_ONSET_RATIO
        f_orbit = max(0.0, min(1.0, f_orbit))

        H_direct  = track_to_td
        H_tangent = (radial_bearing + 90.0 * turn_sign) % 360.0

        tight_orbit_active = (
            have_energy_for_orbit
            and not in_lowkey_band
            and (0.5 * R_des <= dist_to_td_ft <= 1.5 * R_des)
        )

        if tight_orbit_active:
            target_radius_ft = R_orbit_ft
            dist_err_tight   = dist_to_td_ft - target_radius_ft
            max_err_tight    = max(target_radius_ft, 1.0)
            err_norm_tight   = max(-1.0, min(1.0, dist_err_tight / max_err_tight))

            base_hdg = H_tangent

            inward_hdg  = (radial_bearing + 180.0) % 360.0
            outward_hdg = radial_bearing
            target_radial_hdg = inward_hdg if dist_err_tight > 0.0 else outward_hdg

            BASE_RADIAL_WEIGHT_TIGHT = 0.7
            MAX_RADIAL_WEIGHT_TIGHT  = 0.95

            radial_weight_tight = BASE_RADIAL_WEIGHT_TIGHT * abs(err_norm_tight)
            radial_weight_tight = max(0.0, min(MAX_RADIAL_WEIGHT_TIGHT, radial_weight_tight))

            track_hdg = blend_heading(base_hdg, target_radial_hdg, radial_weight_tight)

        else:
            base_hdg = blend_heading(H_direct, H_tangent, f_orbit)

            inward_hdg  = (radial_bearing + 180.0) % 360.0
            outward_hdg = radial_bearing
            target_radial_hdg = inward_hdg if dist_err_ft > 0.0 else outward_hdg

            BASE_RADIAL_WEIGHT = 0.3
            EXTRA_NEAR_GROUND  = 0.4 * (1.0 - max(0.0, min(1.0, alt_ft / initial_alt)))
            MAX_RADIAL_WEIGHT  = 0.9

            radial_weight = (BASE_RADIAL_WEIGHT + EXTRA_NEAR_GROUND) * abs(err_norm) * f_orbit
            radial_weight = max(0.0, min(MAX_RADIAL_WEIGHT, radial_weight))

            track_hdg = blend_heading(base_hdg, target_radial_hdg, radial_weight)

        track_hdg = (track_hdg + 360.0) % 360.0
        last_track_hdg = track_hdg

        # ----- 3) Bank angle + glide ratio this step -----
        bank_min = 0.0
        bank_max = float(max_bank_deg or 30.0)
        bank_max = max(10.0, bank_max)

        R_for_bank = max(LOWKEY_MIN_DIST_FT, min(LOWKEY_MAX_DIST_FT, R_des))
        bank_rad_nominal = math.atan((tas_fps ** 2) / (g * R_for_bank))
        bank_deg_nominal = math.degrees(bank_rad_nominal)

        bank_deg = bank_deg_nominal * f_orbit
        bank_deg = max(bank_min, min(bank_max, bank_deg))

        n       = compute_load_factor(bank_deg if bank_deg > 0 else 0.0)
        turn_gr = straight_gr / max(n, 1.0)
        turn_gr = max(2.0, min(turn_gr, straight_gr)) if bank_deg > 0.5 else straight_gr

        # ----- 4) Move one step -----
        gs_knots, heading_deg, drift_deg = drift_corrected(track_hdg)
        gs_fps = knots_to_fps(gs_knots)

        ds_ft = tas_fps * timestep_sec
        if ds_ft <= 0.1:
            break
        dt = ds_ft / gs_fps if gs_fps > 1e-3 else timestep_sec

        dh_ft  = ds_ft / turn_gr
        alt_ft = max(0.0, alt_ft - dh_ft)

        ds_nm  = ds_ft / 6076.12
        new_pt = point_from(GeoPoint(lat, lon), track_hdg, ds_nm)
        lat, lon = new_pt.latitude, new_pt.longitude

        time_s += dt
        vs_fpm = -(dh_ft / dt) * 60.0 if dt > 1e-3 else 0.0

        # Effective AOB display, using GS and the design radius
        if bank_deg > 0.5:
            bank_eff_rad = math.atan((gs_fps ** 2) / (g * max(R_for_bank, 1.0)))
            aob_display = math.degrees(bank_eff_rad)
        else:
            aob_display = 0.0

        path.append([lat, lon])
        hover.append({
            "alt": alt_ft,
            "tas": tas_knots,
            "time": time_s,
            "aob": aob_display,
            "vs": vs_fpm,
            "segment": "engineout",
            "track": track_hdg,
            "heading": heading_deg,
            "drift": drift_deg,
            "gs": gs_knots,
        })

        # Ground impact
        if alt_ft <= 0.0:
            impact_marker = [lat, lon]
            return path, hover, impact_marker

        # ----- 5) Abeam logic: tight orbit vs PO-180 handoff -----
        dist_to_td_ft = geo_dist(
            (lat, lon),
            (touchdown_point.latitude, touchdown_point.longitude)
        ).feet

        # Energy for max-bank orbit (recomputed at new position/altitude)
        R_bank_ft = (tas_fps ** 2) / (g * math.tan(math.radians(max_bank_deg)))
        R_orbit_ft = max(LOWKEY_MIN_DIST_FT, min(R_des, R_bank_ft))

        n_orbit       = compute_load_factor(max_bank_deg)
        turn_gr_orbit = straight_gr / max(n_orbit, 1.0)
        turn_gr_orbit = max(2.0, min(turn_gr_orbit, straight_gr))

        orbit_circ_ft    = 2.0 * math.pi * R_orbit_ft
        alt_for_orbit_ft = orbit_circ_ft / max(turn_gr_orbit, 1.0)
        alt_margin_ft    = 100.0

        have_energy_for_orbit = alt_ft >= (alt_for_orbit_ft + alt_margin_ft)
        in_lowkey_band        = LOWKEY_ALT_MIN_FT <= alt_ft <= LOWKEY_ALT_MAX_FT

        # PO-180 handoff condition (now that all inputs are defined)
        should_handoff_to_po180 = (
            near_pattern
            and enough_alt_for_po180
            and (
                in_lowkey_band
                or not have_energy_for_orbit
                or (did_tight_orbit and alt_ft <= LOWKEY_ALT_MAX_FT + 500.0)
            )
        )

        ABEAM_TOL_DEG = 15.0
        abeam_err = abs(shortest_angle_diff(radial_bearing, abeam_ref_bearing))
        on_abeam_side = abeam_err <= ABEAM_TOL_DEG
        far_enough_out = dist_to_td_ft >= LOWKEY_MIN_DIST_FT * 1.2

        if ABEAM_LOCKOUT and abeam_err > (ABEAM_TOL_DEG + 20.0):
            ABEAM_LOCKOUT = False

        if near_pattern and on_abeam_side and far_enough_out:
            # 1) Tight overhead orbit if still high and not yet burned one
            if have_energy_for_orbit and not in_lowkey_band and not did_tight_orbit and not ABEAM_LOCKOUT:
                lowkey_mid = 0.5 * (LOWKEY_ALT_MIN_FT + LOWKEY_ALT_MAX_FT)
                alt_target = min(
                    max(lowkey_mid, LOWKEY_ALT_MIN_FT + 100.0),
                    alt_ft - 100.0,
                )
                needed_loss = max(50.0, alt_ft - alt_target)

                did_tight_orbit = True
                ABEAM_LOCKOUT = True

                best_bank = None
                best_err  = float("inf")

                for aob in np.linspace(10.0, max_bank_deg, 80):
                    bank_rad_candidate = math.radians(aob)
                    n_cand = compute_load_factor(aob)
                    turn_gr_cand = straight_gr / max(n_cand, 1.0)
                    turn_gr_cand = max(2.0, min(turn_gr_cand, straight_gr))

                    R_ft_cand = (tas_fps ** 2) / (g * math.tan(bank_rad_candidate))
                    circ_ft   = 2.0 * math.pi * R_ft_cand
                    loss_cand = circ_ft / max(turn_gr_cand, 1.0)

                    err = abs(loss_cand - needed_loss)
                    if err < best_err:
                        best_err = err
                        best_bank = aob

                if best_bank is None:
                    best_bank = min(max_bank_deg, 30.0)

                orbit_path, orbit_hover, alt_ft_out, dt_orbit = simulate_tight_overhead_orbit(
                    start_lat=lat,
                    start_lon=lon,
                    alt_ft=alt_ft,
                    tas_knots=tas_knots,
                    straight_gr=straight_gr,
                    bank_deg=best_bank,
                    turn_sign=turn_sign,
                    entry_track_hdg=track_hdg,
                    wind_dir=wind_dir,
                    wind_speed_knots=wind_speed_knots,
                    timestep_sec=timestep_sec,
                    required_alt_loss_ft=needed_loss,
                )

                if len(orbit_path) > 1:
                    for p in orbit_path[1:]:
                        path.append(p)
                for h in orbit_hover:
                    h["time"] += time_s
                    hover.append(h)
                # Advance the engine-out sim state to the end of the orbit
                if orbit_path:
                    lat, lon = orbit_path[-1]      # new position after orbit
                alt_ft = alt_ft_out               # new altitude after orbit
                time_s += dt_orbit                # new time after orbit

                if alt_ft <= 0.0:
                    impact_marker = path[-1]
                    return path, hover, impact_marker

                continue

            # 2) Hand off to PO-180 when conditions say so
            if should_handoff_to_po180:
                hover[-1]["note"] = "PO180_handoff"
                po180_path, po180_hover, impact_po = simulate_glide_path_to_target(
                    start_point=GeoPoint(lat, lon),
                    start_heading=downwind_track,
                    touchdown_point=touchdown_point,
                    touchdown_heading=touchdown_heading,
                    ac=ac,
                    engine_option=engine_option,
                    weight_lbs=weight_lbs,
                    flap_config=flap_config,
                    prop_config=prop_config,
                    oat_c=oat_c,
                    altimeter_inhg=altimeter_inhg,
                    wind_dir=wind_dir,
                    wind_speed=wind_speed,
                    start_ias_kias=best_glide_kias,
                    altitude_agl=alt_ft,
                    pattern_dir=pattern_dir,
                    selected_airport_elev_ft=selected_airport_elev_ft,
                    max_bank_deg=max_bank_deg,
                    timestep_sec=timestep_sec,
                )

                if po180_path and po180_hover:
                    path.extend(po180_path[1:])
                    hover.extend(po180_hover[1:])
                    return path, hover, impact_po

    impact_marker = [lat, lon]
    return path, hover, impact_marker


####------ STEEP TURN UTILITY------#####

import math
import numpy as np
from geopy import Point as GeoPoint
from geopy.distance import geodesic as geo_dist

def simulate_steep_turn(
    entry_point,
    entry_heading_deg,
    altitude_ft,
    bank_angle_deg,
    turn_sequence,
    tas_knots,
    wind_dir_deg,
    wind_speed_kt,
    timestep_sec=0.5
):
    g = 32.174  # ft/s^2
    tas_fps = tas_knots * 1.68781
    wind_rad = math.radians(wind_dir_deg)
    wind_fps = wind_speed_kt * 1.68781

    start = GeoPoint(entry_point["lat"], entry_point["lon"])
    bank_rad = math.radians(bank_angle_deg)
    radius_ft = (tas_fps ** 2) / (g * math.tan(bank_rad))
    turn_rate_dps = (g * math.tan(bank_rad)) / tas_fps * (180 / math.pi)
    step_deg = 1
    step_time = step_deg / turn_rate_dps

    def compute_arc(center, start_angle, turn_dir, arc_deg, start_time):
        points, hover_data = [], []
        for i in range(int(arc_deg) + 1):
            angle = (start_angle + turn_dir * i) % 360
            pt = geo_dist(feet=radius_ft).destination(center, angle)

            t = start_time + i * step_time
            wind_x = wind_fps * math.cos(wind_rad) * t
            wind_y = wind_fps * math.sin(wind_rad) * t
            pt_lat = pt.latitude + (wind_y / 6076.12 / 60)
            pt_lon = pt.longitude + (wind_x / 6076.12 / 60)

            if i < 10:
                aob = bank_angle_deg * (i / 10)
            elif i > arc_deg - 10:
                aob = bank_angle_deg * ((arc_deg - i) / 10)
            else:
                aob = bank_angle_deg

            points.append([pt_lat, pt_lon])
            hover_data.append({
                "alt": altitude_ft,
                "tas": tas_knots,
                "time": t,
                "aob": round(aob, 1),
                "vs": 0
            })
        return points, hover_data

    sequence = []
    if turn_sequence == "left":
        sequence = [("left", 360)]
    elif turn_sequence == "right":
        sequence = [("right", 360)]
    elif turn_sequence == "left-right":
        sequence = [("left", 360), ("pause", 5), ("right", 360)]
    elif turn_sequence == "right-left":
        sequence = [("right", 360), ("pause", 5), ("left", 360)]

    all_points = []
    all_hover = []
    current = start
    heading = entry_heading_deg
    time = 0

    for action in sequence:
        if action[0] == "pause":
            time += action[1]
            all_points.append([current.latitude, current.longitude])
            all_hover.append({
                "alt": altitude_ft,
                "tas": tas_knots,
                "time": time,
                "aob": 0,
                "vs": 0
            })
            continue

        direction, arc = action
        turn_dir = -1 if direction == "left" else 1
        center_bearing = (heading + turn_dir * 90) % 360
        center = geo_dist(feet=radius_ft).destination(current, center_bearing)
        start_angle = calculate_initial_compass_bearing(center, current)

        arc_pts, arc_data = compute_arc(center, start_angle, turn_dir, arc, time)
        time = arc_data[-1]["time"]
        all_points.extend(arc_pts)
        all_hover.extend(arc_data)

        current = GeoPoint(arc_pts[-1][0], arc_pts[-1][1])
        heading = (heading + turn_dir * arc) % 360

    return all_points, all_hover

def calculate_initial_compass_bearing(pointA, pointB):
    lat1, lat2 = map(math.radians, [pointA.latitude, pointB.latitude])
    diff_lon = math.radians(pointB.longitude - pointA.longitude)
    x = math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon)
    )
    return (math.degrees(math.atan2(x, y)) + 360) % 360

import dash_leaflet as dl
from dash import html

def render_hover_polyline(path, hover_data, color="blue", weight=3):
    return dl.Polyline(
        positions=path,
        color=color,
        weight=weight,
        children=[
            dl.Tooltip(
                html.Div([
                    html.Div(f"{pt['alt']} ft AGL"),
                    html.Div(f"{pt['tas']} kt"),
                    html.Div(f"{pt['time']:.1f} sec"),
                    html.Div(f"{pt['aob']}° AOB"),
                    html.Div(f"{pt['vs']} fpm")
                ])
            ) for pt in hover_data[::5]
        ]
    )

