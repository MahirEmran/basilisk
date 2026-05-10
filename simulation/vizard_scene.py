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


def enable_vizard(
    sc_sim,
    sim_task_name,
    sc_object,
    save_path,
    generic_storage_list=None,
    live_stream=False,
    broadcast_stream=False,
):
    """Create the Vizard interface and apply the baseline display settings."""
    viz = vizSupport.enableUnityVisualization(
        sc_sim,
        sim_task_name,
        sc_object,
        saveFile=save_path,
        genericStorageList=generic_storage_list,
        oscOrbitColorList=[[80, 180, 255, 255]],
        trueOrbitColorList=[[255, 255, 255, 180]],
        liveStream=bool(live_stream),
        broadcastStream=bool(broadcast_stream),
    )
    assert viz is not None, "Vizard setup failed"

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
    if generic_storage_list is not None:
        vizSupport.setInstrumentGuiSetting(viz, showGenericStoragePanel=True)
    return viz


def create_battery_storage_panel(battery_state_reader):
    """Create a Vizard battery storage panel bound to a battery state message reader."""
    battery_panel = vizSupport.vizInterface.GenericStorage()
    battery_panel.label = "Battery Charge"
    battery_panel.units = "W*s"
    battery_panel.color = vizSupport.vizInterface.IntVector(
        vizSupport.toRGBA255("red") + vizSupport.toRGBA255("green")
    )
    battery_panel.thresholds = vizSupport.vizInterface.IntVector([20])  # [%]
    battery_panel.batteryStateInMsg = battery_state_reader
    return battery_panel


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
    vizSupport.createPointLine(viz, toBodyName="earth", lineColor=[0, 180, 255, 200])
    vizSupport.createPointLine(viz, toBodyName="sun", lineColor=[255, 220, 0, 200])

    red = [220, 30, 30, 230]
    orange = [255, 140, 0, 180]

    # LOST keep-out cones for Earth/Sun/Moon.
    for body in ["earth", "sun", "moon"]:
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

    # FOUND Earth keep-in cone.
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

    # FOUND Sun keep-out cones.
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
        label="STATE: init | EPS: init | TMP: init | SOC: init",
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
    mode_text = "UNKNOWN" if guidance_state is None else str(guidance_state).replace("_", " ")
    eps_text = "CHARGING" if is_charging else "DISCHARGING"
    label = (
        f"STATE {mode_text} | "
        f"EPS {eps_text} | "
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
