import numpy as np
from Basilisk.utilities import macros, vizSupport


def _parse_station_specs(station_specs):
    """Parse station specs in LABEL:LAT_DEG:LON_DEG format."""
    stations = []
    if not station_specs:
        return stations

    for token in station_specs:
        if token is None:
            continue

        spec = str(token).strip()
        if not spec:
            continue

        parts = spec.split(":")
        if len(parts) != 3:
            print(f"[VIZ] Ignoring malformed ground station '{spec}' (expected LABEL:LAT_DEG:LON_DEG)")
            continue

        label = parts[0].strip()
        lat_raw = parts[1].strip()
        lon_raw = parts[2].strip()
        if not label or not lat_raw or not lon_raw:
            print(f"[VIZ] Ignoring malformed ground station '{spec}' (empty fields)")
            continue

        try:
            lat_deg = float(lat_raw)
            lon_deg = float(lon_raw)
        except ValueError:
            print(f"[VIZ] Ignoring malformed ground station '{spec}' (non-numeric lat/lon)")
            continue

        stations.append((label, lat_deg, lon_deg))

    return stations


def enable_vizard(sc_sim, sim_task_name, sc_object, save_path):
    """Create the Vizard interface and apply the baseline display settings."""
    # enableUnityVisualization wires Basilisk data logging + Unity-side visualization together.
    # saveFile is the .bin that Vizard reads/replays.
    viz = vizSupport.enableUnityVisualization(
        sc_sim,
        sim_task_name,
        sc_object,
        saveFile=save_path,
        oscOrbitColorList=[[80, 180, 255, 255]],
        trueOrbitColorList=[[255, 255, 255, 180]],
    )
    assert viz is not None, "Vizard setup failed"

    # These are display toggles only; they do not change dynamics or FSW behavior.
    # Most are 0/1 flags in the Vizard settings schema.
    viz.settings.orbitLinesOn = 1
    viz.settings.trueTrajectoryLinesOn = 1
    viz.settings.spacecraftCSon = 1
    viz.settings.showSpacecraftLabels = 1
    viz.settings.showCelestialBodyLabels = 1
    viz.settings.showCameraLabels = 1
    viz.settings.forceStartAtSpacecraftLocalView = 1
    viz.settings.spacecraftSizeMultiplier = 1.0
    viz.settings.viewCameraBoresightHUD = 1
    viz.settings.viewCameraConeHUD = 1
    return viz


def add_vizard_scene_overlays(
    viz,
    spacecraft_tag,
    vec_lost_b,
    vec_found_b,
    pos_lost_b,
    pos_found_b,
    lost_fov_deg,
    found_fov_deg,
    lost_half_deg,
    found_half_deg,
    lost_excl_half_deg,
    found_excl_half_deg,
    ground_station_specs=None,
    earth_body_name="earth",
):
    """Add guidance cones, reference lines, and camera visuals to the Vizard scene."""
    # createPointLine draws a line from spacecraft to target body for visual intuition.
    # This is purely diagnostic and does not feed back into guidance.
    vizSupport.createPointLine(viz, toBodyName="earth", lineColor=[0, 180, 255, 200])
    vizSupport.createPointLine(viz, toBodyName="sun", lineColor=[255, 220, 0, 200])

    red = [220, 30, 30, 230]
    orange = [255, 140, 0, 180]

    for body in ["earth", "sun", "moon"]:
        # Basilisk createConeInOut key semantics:
        # - normalVector_B: cone axis in the spacecraft body frame B
        # - position_B: cone apex location in body frame B
        # - incidenceAngle: half-angle in radians
        # - isKeepIn=False: object should stay OUTSIDE the cone
        # Orange shows a warning margin around LOST keep-out.
        vizSupport.createConeInOut(
            viz,
            fromBodyName=spacecraft_tag,
            toBodyName=body,
            coneColor=orange,
            normalVector_B=vec_lost_b,
            position_B=pos_lost_b,
            incidenceAngle=lost_excl_half_deg * macros.D2R,
            isKeepIn=False,
            coneHeight=100.0,
            coneName=f"LOST_{body}_EXCL",
        )
        # Red is the strict LOST keep-out region (same axis/apex, tighter angle).
        vizSupport.createConeInOut(
            viz,
            fromBodyName=spacecraft_tag,
            toBodyName=body,
            coneColor=red,
            normalVector_B=vec_lost_b,
            position_B=pos_lost_b,
            incidenceAngle=lost_half_deg * macros.D2R,
            isKeepIn=False,
            coneHeight=100.0,
            coneName=f"LOST_{body}_FOV",
        )

    # FOUND Earth cone uses isKeepIn=True: Earth should remain INSIDE this cone.
    vizSupport.createConeInOut(
        viz,
        fromBodyName=spacecraft_tag,
        toBodyName="earth",
        coneColor=red,
        normalVector_B=vec_found_b,
        position_B=pos_found_b,
        incidenceAngle=found_half_deg * macros.D2R,
        isKeepIn=True,
        coneHeight=100.0,
        coneName="FOUND_Earth_FOV",
    )

    # FOUND Sun cones use isKeepIn=False: Sun should remain OUTSIDE these cones.
    vizSupport.createConeInOut(
        viz,
        fromBodyName=spacecraft_tag,
        toBodyName="sun",
        coneColor=orange,
        normalVector_B=vec_found_b,
        position_B=pos_found_b,
        incidenceAngle=found_excl_half_deg * macros.D2R,
        isKeepIn=False,
        coneHeight=100.0,
        coneName="FOUND_Sun_EXCL",
    )
    vizSupport.createConeInOut(
        viz,
        fromBodyName=spacecraft_tag,
        toBodyName="sun",
        coneColor=red,
        normalVector_B=vec_found_b,
        position_B=pos_found_b,
        incidenceAngle=found_half_deg * macros.D2R,
        isKeepIn=False,
        coneHeight=100.0,
        coneName="FOUND_Sun_FOV",
    )

    # createStandardCamera expects fieldOfView in radians.
    # setMode=1 registers a body-fixed camera that follows spacecraft attitude.
    vizSupport.createStandardCamera(
        viz,
        setMode=1,
        bodyTarget=spacecraft_tag,
        fieldOfView=lost_fov_deg * macros.D2R,
        pointingVector_B=vec_lost_b,
        position_B=pos_lost_b,
        displayName="LOST_cam",
    )
    vizSupport.createStandardCamera(
        viz,
        setMode=1,
        bodyTarget=spacecraft_tag,
        fieldOfView=found_fov_deg * macros.D2R,
        pointingVector_B=vec_found_b,
        position_B=pos_found_b,
        displayName="FOUND_cam",
    )

    stations = _parse_station_specs(ground_station_specs)
    if stations:
        for label, lat_deg, lon_deg in stations:
            vizSupport.addLocation(
                viz,
                stationName=label,
                parentBodyName=earth_body_name,
                lla_GP=[lat_deg * macros.D2R, lon_deg * macros.D2R, 0.0],
                fieldOfView=np.radians(35.0),
                color=[80, 180, 255, 220],
                range=900.0 * 1000.0,
                markerScale=0.9,
                label=label,
            )

        viz.settings.showLocationCommLines = 0
        viz.settings.showLocationCones = 1
        viz.settings.showLocationLabels = 1


def add_spacecraft_status_overlay(viz, spacecraft_tag):
    """Add a dynamic spacecraft status marker for power/temperature telemetry text."""
    station_name = f"{spacecraft_tag}_status"
    vizSupport.addLocation(
        viz,
        stationName=station_name,
        parentBodyName=spacecraft_tag,
        r_GP_P=[0.0, 0.0, 0.25],
        fieldOfView=np.radians(1.0),
        range=1.0,
        color="cyan",
        markerScale=1.2,
        label="POWER: init | MODE: init | TMP: init | SOC: init",
    )
    viz.settings.showLocationLabels = 1
    return station_name


def update_spacecraft_status_overlay(
    viz,
    station_name,
    is_charging,
    guidance_state,
    net_power_w,
    temp_c,
    soc_pct,
):
    """Update the dynamic status marker text and color for the current power/thermal state."""
    status_text = "CHARGING" if is_charging else "ACTIVE"
    mode_text = str(guidance_state).replace("_", " ")
    label = (
        f"POWER {status_text} | "
        f"MODE {mode_text} | "
        f"PWR {net_power_w:+5.1f} W | "
        f"TMP {temp_c:5.1f} C | "
        f"SOC {soc_pct:5.1f}%"
    )
    color = "cyan" if is_charging else "orange"

    vizSupport.changeLocation(
        viz,
        stationName=station_name,
        color=color,
        label=label,
    )
