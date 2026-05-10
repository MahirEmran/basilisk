from datetime import datetime, timezone
import numpy as np


def approx_sun_hat_from_epoch(epoch_iso_utc):
    """Approximate inertial Earth->Sun unit vector from epoch (UTC ISO-8601)."""
    dt = datetime.fromisoformat(epoch_iso_utc.replace("Z", "+00:00")).astimezone(timezone.utc)
    # Low-order solar model using days since J2000.
    j2000 = datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
    n_days = (dt - j2000).total_seconds() / 86400.0
    mean_long = np.radians((280.460 + 0.9856474 * n_days) % 360.0)
    mean_anom = np.radians((357.528 + 0.9856003 * n_days) % 360.0)
    lam = mean_long + np.radians(1.915) * np.sin(mean_anom) + np.radians(0.020) * np.sin(2.0 * mean_anom)
    eps = np.radians(23.439 - 0.0000004 * n_days)
    sun_hat = np.array([
        np.cos(lam),
        np.cos(eps) * np.sin(lam),
        np.sin(eps) * np.sin(lam),
    ])
    return sun_hat / np.linalg.norm(sun_hat)


def compute_compromise_x(earth_hat, sun_hat, rho_rad):
    """Build the EXPERIMENT-mode +X target on the Earth-limb cone away from the Sun."""
    u1 = earth_hat

    # Project anti-sun into the plane normal to nadir.
    anti_sun = -sun_hat
    u2 = anti_sun - np.dot(anti_sun, u1) * u1
    n2 = np.linalg.norm(u2)

    if n2 < 1e-6:
        u2 = np.array([1, 0, 0]) if abs(u1[0]) < 0.9 else np.array([0, 1, 0])
        u2 = u2 - np.dot(u2, u1) * u1
        n2 = np.linalg.norm(u2)
    u2 /= n2

    # Keep +X on the Earth-limb cone while biasing away from the Sun.
    x_b = np.cos(rho_rad) * u1 + np.sin(rho_rad) * u2
    return x_b / np.linalg.norm(x_b)


def build_yz_frame_about_x(x_b, earth_hat):
    """Create a stable right-handed frame around x_B with y/z used for roll sweeping."""
    # Build a robust right-handed triad around +X.
    z_temp = np.cross(x_b, earth_hat)
    if np.linalg.norm(z_temp) < 1e-6:
        z_temp = np.cross(x_b, np.array([0, 1, 0]))
        if np.linalg.norm(z_temp) < 1e-6:
            z_temp = np.cross(x_b, np.array([0, 0, 1]))
    z_temp /= np.linalg.norm(z_temp)
    y_temp = np.cross(z_temp, x_b)
    y_temp /= np.linalg.norm(y_temp)
    return y_temp, z_temp


def solve_roll_for_lost_clearance(x_b, earth_hat, sun_hat, prev_roll_deg=None, extra_keepout_hats=None):
    """Choose roll that maximizes minimum LOST (+Z) clearance from Earth, Sun, and optional bodies."""
    y_temp, z_temp = build_yz_frame_about_x(x_b, earth_hat)
    if extra_keepout_hats is None:
        extra_keepout_hats = []

    roll_candidates = []
    max_score = -1.0

    # Maximize minimum LOST clearance over keep-out bodies.
    for roll_deg in range(0, 360, 2):
        roll = np.radians(roll_deg)
        y_test = np.cos(roll) * y_temp + np.sin(roll) * z_temp
        z_test = np.cross(x_b, y_test)
        z_test /= np.linalg.norm(z_test)
        lost_hat = z_test

        keepout_hats = [earth_hat, sun_hat, *extra_keepout_hats]
        keepout_angles = [
            np.degrees(np.arccos(np.clip(np.dot(lost_hat, keepout_hat), -1.0, 1.0)))
            for keepout_hat in keepout_hats
        ]
        score = min(keepout_angles)

        roll_candidates.append((roll_deg, score, y_test, z_test))
        if score > max_score:
            max_score = score

    keep_margin_deg = 0.25
    near_opt = [c for c in roll_candidates if c[1] >= max_score - keep_margin_deg]

    if prev_roll_deg is None:
        best_roll_deg, _, best_y, best_z = max(near_opt, key=lambda c: c[1])
    else:
        def wrap_delta_deg(a, b):
            return abs(((a - b + 180.0) % 360.0) - 180.0)

        best_roll_deg, _, best_y, best_z = min(
            near_opt,
            key=lambda c: wrap_delta_deg(c[0], prev_roll_deg)
        )

    return best_roll_deg, best_y, best_z, max_score


def build_frame_for_plus_x_target(plus_x_target_hat, z_hint_hat):
    """Build a frame where +X points to target (used for Earth/ground-pointing downlink)."""
    plus_x_mag = np.linalg.norm(plus_x_target_hat)
    if plus_x_mag < 1.0e-12:
        return None, None, None

    plus_x_hat = plus_x_target_hat / plus_x_mag
    x_B = plus_x_hat

    z_proj = z_hint_hat - np.dot(z_hint_hat, x_B) * x_B
    z_proj_mag = np.linalg.norm(z_proj)
    if z_proj_mag < 1.0e-12:
        fallback = np.array([0.0, 0.0, 1.0])
        if abs(x_B[2]) >= 0.9:
            fallback = np.array([0.0, 1.0, 0.0])
        z_proj = fallback - np.dot(fallback, x_B) * x_B
        z_proj_mag = np.linalg.norm(z_proj)
        if z_proj_mag < 1.0e-12:
            return None, None, None

    z_B = z_proj / z_proj_mag
    y_B = np.cross(z_B, x_B)
    y_mag = np.linalg.norm(y_B)
    if y_mag < 1.0e-12:
        return None, None, None
    y_B = y_B / y_mag

    return x_B, y_B, z_B
