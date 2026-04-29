import numpy as np
from Basilisk.utilities import macros, RigidBodyKinematics as rbk
from Basilisk.architecture import sysModel, messaging

from guidance_math import approx_sun_hat_from_epoch, compute_compromise_x, solve_roll_for_lost_clearance, build_frame_for_plus_x_target


class ActiveGuidance(sysModel.SysModel):
    def __init__(
        self,
        mode,
        epoch_iso_utc,
        lost_excl_half_deg,
        status_period_sec,
        pos_found_b,
        pos_comms_b,
        gnss_fix_period_sec,
        gnss_fix_duration_sec,
        gnss_zenith_half_angle_deg,
        gnss_dead_reckoning_sec,
        downlink_window_sec,
        ground_stations,
    ):
        super().__init__()
        self.mode = mode
        self.attRefOutMsg = messaging.AttRefMsg()
        self.scStateInMsg = messaging.SCStatesMsgReader()
        self.sunStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.moonStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.prev_roll_deg = None
        self.state = None
        self.pos_found_b = np.array(pos_found_b, dtype=float)
        self.pos_comms_b = np.array(pos_comms_b, dtype=float)

        # Bootstrap value used until the SPICE message is available.
        self.default_sun_hat = approx_sun_hat_from_epoch(epoch_iso_utc)

        # Earth apparent half-angle from a 400 km altitude shell.
        # This is used for the limb geometry and eclipse-style checks.
        self.rho_rad = np.arcsin(6371.0 / (6371.0 + 400.0))
        self.earth_half_angle_deg = np.degrees(self.rho_rad)

        # Hysteresis bands prevent CHARGING/EXPERIMENT chatter near boundaries.
        self.charge_enter_clear_deg = lost_excl_half_deg + 2.0
        self.charge_exit_clear_deg = max(lost_excl_half_deg - 1.0, 0.0)
        self.charge_enter_sun_vis_deg = self.earth_half_angle_deg + 2.0
        self.charge_exit_sun_vis_deg = max(self.earth_half_angle_deg - 1.0, 0.0)

        self.status_period_nanos = macros.sec2nano(status_period_sec)
        self.last_status_print_nanos = None

        self.gnss_fix_period_nanos = macros.sec2nano(gnss_fix_period_sec)
        self.gnss_fix_duration_nanos = macros.sec2nano(gnss_fix_duration_sec)
        self.gnss_zenith_half_angle_deg = float(gnss_zenith_half_angle_deg)
        self.gnss_dead_reckoning_nanos = macros.sec2nano(gnss_dead_reckoning_sec)
        self.downlink_window_nanos = macros.sec2nano(downlink_window_sec)

        self.gnss_fix_schedule_initialized = False
        self.next_gnss_fix_nanos = 0
        self.gnss_fix_end_nanos = 0
        self.last_gnss_good_nanos = 0
        self.has_gnss_good_timestamp = False

        self.downlink_window_active = False
        self.downlink_window_end_nanos = 0
        self.downlink_window_station = None

        self.had_visible_station_last_step = False
        self.last_visible_station_label = None

        self.active_ground_station = "NONE"
        self.ground_stations = self._parse_ground_stations(ground_stations)

    @staticmethod
    def _parse_ground_stations(ground_stations):
        parsed = []
        if not ground_stations:
            return parsed

        for spec in ground_stations:
            if spec is None:
                continue

            token = str(spec).strip()
            if not token:
                continue

            parts = token.split(":")
            if len(parts) != 3:
                print(f"[ADCS] Ignoring malformed ground station '{token}' (expected label:lat_deg:lon_deg)")
                continue

            label = parts[0].strip()
            lat_raw = parts[1].strip()
            lon_raw = parts[2].strip()
            if not label or not lat_raw or not lon_raw:
                print(f"[ADCS] Ignoring malformed ground station '{token}' (empty fields)")
                continue

            try:
                lat_deg = float(lat_raw)
                lon_deg = float(lon_raw)
            except ValueError:
                print(f"[ADCS] Ignoring malformed ground station '{token}' (non-numeric lat/lon)")
                continue

            parsed.append(
                {
                    "label": label,
                    "lat_rad": np.deg2rad(lat_deg),
                    "lon_rad": np.deg2rad(lon_deg),
                }
            )

        return parsed

    def _select_visible_ground_station(self, observer_pos_N, current_sim_nanos):
        if not self.ground_stations:
            return None, None

        t_sec = current_sim_nanos * 1.0e-9
        earth_theta_rad = 7.2921159e-5 * t_sec  # [rad]
        cos_theta = np.cos(earth_theta_rad)
        sin_theta = np.sin(earth_theta_rad)

        best_label = None
        best_los_hat = None
        best_elevation_proxy = -2.0

        for station in self.ground_stations:
            lat_rad = station["lat_rad"]
            lon_rad = station["lon_rad"]

            station_ecef = np.array(
                [
                    6371000.0 * np.cos(lat_rad) * np.cos(lon_rad),
                    6371000.0 * np.cos(lat_rad) * np.sin(lon_rad),
                    6371000.0 * np.sin(lat_rad),
                ]
            )

            station_pos_N = np.array(
                [
                    cos_theta * station_ecef[0] - sin_theta * station_ecef[1],
                    sin_theta * station_ecef[0] + cos_theta * station_ecef[1],
                    station_ecef[2],
                ]
            )

            station_zenith_hat = station_pos_N / np.linalg.norm(station_pos_N)
            station_to_sc = observer_pos_N - station_pos_N
            station_to_sc_mag = np.linalg.norm(station_to_sc)
            if station_to_sc_mag < 1.0:
                continue

            station_to_sc_hat = station_to_sc / station_to_sc_mag
            elevation_proxy = float(np.dot(station_zenith_hat, station_to_sc_hat))
            if elevation_proxy <= 0.0:
                continue

            sc_to_station = station_pos_N - observer_pos_N
            sc_to_station_mag = np.linalg.norm(sc_to_station)
            if sc_to_station_mag < 1.0:
                continue

            sc_to_station_hat = sc_to_station / sc_to_station_mag
            if elevation_proxy > best_elevation_proxy:
                best_elevation_proxy = elevation_proxy
                best_label = station["label"]
                best_los_hat = sc_to_station_hat

        return best_label, best_los_hat

    @staticmethod
    def _build_frame_for_minus_z_target(minus_z_target_hat, x_hint_hat):
        minus_z_mag = np.linalg.norm(minus_z_target_hat)
        if minus_z_mag < 1.0e-12:
            return None, None, None

        minus_z_hat = minus_z_target_hat / minus_z_mag
        z_B = -minus_z_hat

        x_proj = x_hint_hat - np.dot(x_hint_hat, z_B) * z_B
        x_proj_mag = np.linalg.norm(x_proj)
        if x_proj_mag < 1.0e-12:
            fallback = np.array([1.0, 0.0, 0.0])
            if abs(z_B[0]) >= 0.9:
                fallback = np.array([0.0, 1.0, 0.0])
            x_proj = fallback - np.dot(fallback, z_B) * z_B
            x_proj_mag = np.linalg.norm(x_proj)
            if x_proj_mag < 1.0e-12:
                return None, None, None

        x_B = x_proj / x_proj_mag
        y_B = np.cross(z_B, x_B)
        y_mag = np.linalg.norm(y_B)
        if y_mag < 1.0e-12:
            return None, None, None
        y_B = y_B / y_mag

        return x_B, y_B, z_B

    @staticmethod
    def _antenna_zenith_angle_deg(z_B, earth_hat_sc):
        """Calculate zenith angle for GNSS antenna on -Z face."""
        antenna_hat = -z_B
        zenith_hat = -earth_hat_sc
        cosine = float(np.clip(np.dot(antenna_hat, zenith_hat), -1.0, 1.0))
        return float(np.degrees(np.arccos(cosine)))

    def Reset(self, CurrentSimNanos):
        self.prev_roll_deg = None
        self.state = "UNINITIALIZED"
        self.last_status_print_nanos = CurrentSimNanos
        self.active_ground_station = "NONE"

        self.gnss_fix_schedule_initialized = False
        self.next_gnss_fix_nanos = 0
        self.gnss_fix_end_nanos = 0
        self.last_gnss_good_nanos = CurrentSimNanos
        self.has_gnss_good_timestamp = False

        self.downlink_window_active = False
        self.downlink_window_end_nanos = 0
        self.downlink_window_station = None

        self.had_visible_station_last_step = False
        self.last_visible_station_label = None

    def UpdateState(self, CurrentSimNanos):
        scState = self.scStateInMsg()
        r_N = np.array(scState.r_BN_N)
        r_mag = np.linalg.norm(r_N)

        # Early startup can deliver effectively-empty state values; hold identity ref then.
        if r_mag < 1.0:
            refMsg = messaging.AttRefMsgPayload()
            refMsg.sigma_RN = [0.0, 0.0, 0.0]
            refMsg.omega_RN_N = [0.0, 0.0, 0.0]
            refMsg.domega_RN_N = [0.0, 0.0, 0.0]
            self.attRefOutMsg.write(refMsg, CurrentSimNanos)
            return

        # Use SPICE Sun ephemeris once available.
        # SPICE gives absolute positions, so subtract the spacecraft position.
        sun_hat_sc = self.default_sun_hat
        sun_abs_N = None
        moon_hat_sc = None
        if self.sunStateInMsg.isLinked() and self.sunStateInMsg.isWritten():
            sun_state = self.sunStateInMsg()
            sun_abs_N = np.array(sun_state.PositionVector)
            sun_rel_N = sun_abs_N - r_N
            sun_rel_mag = np.linalg.norm(sun_rel_N)
            if sun_rel_mag > 1.0:
                sun_hat_sc = sun_rel_N / sun_rel_mag

        if self.moonStateInMsg.isLinked() and self.moonStateInMsg.isWritten():
            moon_state = self.moonStateInMsg()
            moon_rel_N = np.array(moon_state.PositionVector) - r_N
            moon_rel_mag = np.linalg.norm(moon_rel_N)
            if moon_rel_mag > 1.0:
                # Spacecraft->Moon LOS in inertial coordinates.
                moon_hat_sc = moon_rel_N / moon_rel_mag

        earth_hat_sc = -r_N / r_mag  # Nadir direction in inertial coordinates.

        # FOUND is physically offset from COM, so use camera location (not COM) for constraints.
        found_pos_N = r_N.copy()
        comms_pos_N = r_N.copy()
        if hasattr(scState, "sigma_BN"):
            try:
                sigma_BN_now = np.array(scState.sigma_BN)
                if sigma_BN_now.size == 3:
                    c_bn = rbk.MRP2C(sigma_BN_now)
                    c_nb = c_bn.T
                    found_pos_N = r_N + c_nb.dot(self.pos_found_b)
                    comms_pos_N = r_N + c_nb.dot(self.pos_comms_b)
            except Exception:
                found_pos_N = r_N.copy()
                comms_pos_N = r_N.copy()

        found_r_mag = np.linalg.norm(found_pos_N)
        # Using FOUND position (not COM) slightly changes Earth/Sun direction vectors,
        # which matters when constraints are close to cone boundaries.
        earth_hat_found = earth_hat_sc if found_r_mag < 1.0 else (-found_pos_N / found_r_mag)

        sun_hat_found = sun_hat_sc
        if sun_abs_N is not None:
            sun_rel_found_N = sun_abs_N - found_pos_N
            sun_rel_found_mag = np.linalg.norm(sun_rel_found_N)
            if sun_rel_found_mag > 1.0:
                sun_hat_found = sun_rel_found_N / sun_rel_found_mag

        # Roll-only wants -X at Sun so +X (FOUND) stays anti-sun.
        roll_only_x = -sun_hat_found
        roll_only_roll, roll_only_y, roll_only_z, roll_only_score = solve_roll_for_lost_clearance(
            roll_only_x, earth_hat_sc, sun_hat_sc, self.prev_roll_deg
        )

        # Angle between Sun line-of-sight and Earth center line-of-sight.
        # If this is smaller than Earth apparent half-angle, Sun is geometrically eclipsed.
        ang_sun_earth_deg = np.degrees(np.arccos(np.clip(np.dot(sun_hat_found, earth_hat_found), -1.0, 1.0)))

        x_B = roll_only_x
        best_roll_deg, best_y, best_z = roll_only_roll, roll_only_y, roll_only_z
        selected_state = "CHARGING"

        experiment_context = False
        if self.mode == "ROLL_ONLY":
            selected_state = "CHARGING"
        elif self.mode in ("EXPERIMENT", "COMPROMISE"):
            experiment_context = True
        else:
            # HYBRID state machine.
            # Enter thresholds are stricter than exit thresholds (intentional hysteresis).
            if self.state == "CHARGING":
                can_charge = roll_only_score >= self.charge_exit_clear_deg
                sun_visible = ang_sun_earth_deg >= self.charge_exit_sun_vis_deg
            else:
                can_charge = roll_only_score >= self.charge_enter_clear_deg
                sun_visible = ang_sun_earth_deg >= self.charge_enter_sun_vis_deg

            can_charge = can_charge and sun_visible

            if can_charge:
                selected_state = "CHARGING"
                x_B = roll_only_x
                best_roll_deg, best_y, best_z = roll_only_roll, roll_only_y, roll_only_z
            else:
                experiment_context = True

        self.active_ground_station = "NONE"

        if experiment_context:
            if not self.has_gnss_good_timestamp:
                self.last_gnss_good_nanos = CurrentSimNanos
                self.has_gnss_good_timestamp = True

            dead_reckoning_exceeded = (
                CurrentSimNanos - self.last_gnss_good_nanos
            ) >= self.gnss_dead_reckoning_nanos

            if not self.gnss_fix_schedule_initialized:
                self.next_gnss_fix_nanos = CurrentSimNanos + self.gnss_fix_period_nanos
                self.gnss_fix_schedule_initialized = True

            if self.gnss_fix_end_nanos > 0 and CurrentSimNanos >= self.gnss_fix_end_nanos:
                self.gnss_fix_end_nanos = 0

            if self.gnss_fix_end_nanos == 0 and (
                CurrentSimNanos >= self.next_gnss_fix_nanos or dead_reckoning_exceeded
            ):
                self.gnss_fix_end_nanos = CurrentSimNanos + self.gnss_fix_duration_nanos
                self.next_gnss_fix_nanos = CurrentSimNanos + self.gnss_fix_period_nanos

            if self.gnss_fix_end_nanos > CurrentSimNanos:
                minus_z_target_hat = -earth_hat_sc
                gnss_x, gnss_y, gnss_z = self._build_frame_for_minus_z_target(minus_z_target_hat, sun_hat_sc)
                if gnss_x is not None:
                    x_B, best_y, best_z = gnss_x, gnss_y, gnss_z
                    best_roll_deg = 0.0
                    selected_state = "GNSS_FIX"
            else:
                x_B = compute_compromise_x(earth_hat_found, sun_hat_found, self.rho_rad)
                extra_keepout_hats = [moon_hat_sc] if moon_hat_sc is not None else None
                best_roll_deg, best_y, best_z, _ = solve_roll_for_lost_clearance(
                    x_B,
                    earth_hat_sc,
                    sun_hat_sc,
                    self.prev_roll_deg,
                    extra_keepout_hats=extra_keepout_hats,
                )
                selected_state = "EXPERIMENT"

                visible_label, visible_los_hat = self._select_visible_ground_station(comms_pos_N, CurrentSimNanos)
                has_visible_station = visible_label is not None

                if self.downlink_window_active:
                    window_expired = CurrentSimNanos >= self.downlink_window_end_nanos
                    station_still_visible = has_visible_station and visible_label == self.downlink_window_station
                    if window_expired or not station_still_visible:
                        self.downlink_window_active = False
                        self.downlink_window_station = None

                visibility_rising_edge = has_visible_station and (
                    (not self.had_visible_station_last_step) or visible_label != self.last_visible_station_label
                )

                if (not self.downlink_window_active) and visibility_rising_edge:
                    self.downlink_window_active = True
                    self.downlink_window_station = visible_label
                    self.downlink_window_end_nanos = CurrentSimNanos + self.downlink_window_nanos

                if self.downlink_window_active and has_visible_station and visible_label == self.downlink_window_station:
                    dl_x, dl_y, dl_z = build_frame_for_plus_x_target(visible_los_hat, sun_hat_sc)
                    if dl_x is not None:
                        x_B, best_y, best_z = dl_x, dl_y, dl_z
                        best_roll_deg = 0.0
                        selected_state = "DOWNLINK"
                        self.active_ground_station = self.downlink_window_station

                self.had_visible_station_last_step = has_visible_station
                self.last_visible_station_label = visible_label
        else:
            # GNSS fixing is only needed in experiment context.
            self.gnss_fix_schedule_initialized = False
            self.next_gnss_fix_nanos = 0
            self.gnss_fix_end_nanos = 0
            self.has_gnss_good_timestamp = False

            self.downlink_window_active = False
            self.downlink_window_station = None
            self.had_visible_station_last_step = False
            self.last_visible_station_label = None

        antenna_zenith_angle_deg = self._antenna_zenith_angle_deg(best_z, earth_hat_sc)
        if experiment_context and antenna_zenith_angle_deg <= self.gnss_zenith_half_angle_deg:
            self.last_gnss_good_nanos = CurrentSimNanos
            self.has_gnss_good_timestamp = True

        if selected_state != self.state:
            t_sec = CurrentSimNanos * 1.0e-9
            panel_cmd = "OPEN" if selected_state == "CHARGING" else "STOW"
            print(
                f"[ADCS] t={t_sec:8.1f}s -> {selected_state} "
                f"(roll-only LOST clearance={roll_only_score:5.1f} deg, panel cmd={panel_cmd}, "
                f"station={self.active_ground_station}, zenith-angle={antenna_zenith_angle_deg:5.1f} deg)"
            )
            self.state = selected_state

        if self.mode == "HYBRID":
            should_print_status = (
                self.last_status_print_nanos is None
                or (CurrentSimNanos - self.last_status_print_nanos) >= self.status_period_nanos
            )
            if should_print_status:
                t_sec = CurrentSimNanos * 1.0e-9
                panel_cmd = "OPEN" if selected_state == "CHARGING" else "STOW"
                print(
                    f"[HYBRID] t={t_sec:8.1f}s active={selected_state} "
                    f"panel={panel_cmd} station={self.active_ground_station} "
                    f"roll-only-clearance={roll_only_score:5.1f} deg "
                    f"sun-earth-angle={ang_sun_earth_deg:5.1f} deg "
                    f"zenith-angle={antenna_zenith_angle_deg:5.1f} deg"
                )
                self.last_status_print_nanos = CurrentSimNanos

        self.prev_roll_deg = best_roll_deg

        # Assemble DCM rows [x_B, y_B, z_B] and convert to MRP.
        # Here we define R relative to N by its body axes expressed in inertial coordinates.
        dcm_RN = np.array([x_B, best_y, best_z])
        sigma_RN = rbk.C2MRP(dcm_RN)

        # Rare numerical edge case guard.
        if np.any(np.isnan(sigma_RN)):
            sigma_RN = np.array([0.0, 0.0, 0.0])

        refMsg = messaging.AttRefMsgPayload()
        refMsg.sigma_RN = sigma_RN.tolist()
        refMsg.omega_RN_N = [0.0, 0.0, 0.0]
        refMsg.domega_RN_N = [0.0, 0.0, 0.0]
        self.attRefOutMsg.write(refMsg, CurrentSimNanos)
