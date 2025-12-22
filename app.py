import requests
import math
import os
import json
import dash
from dash import dcc, html, Input, Output, State, MATCH, ALL, ctx
import dash_bootstrap_components as dbc
import dash_leaflet as dl
from geopy.point import Point as GeoPoint
from geopy.distance import distance as geo_distance
from dash.exceptions import PreventUpdate
from utility import (
    compute_density_altitude,
    compute_pressure_altitude,
    compute_true_airspeed,
    compute_glide_ratio,
    compute_descent_angle_deg,
    simulate_glide_path_to_target,
    point_from,
    calculate_initial_compass_bearing,
    knots_to_fps,
    fps_to_knots,
    compute_turn_radius,
    compute_required_bank,
    compute_Ps,
    compute_lift_limit_speed,
    compute_load_factor,
    compute_stall_speed,
    wind_components,
    simulate_steep_turn,
    render_hover_polyline,
    simulate_engineout_glide,
    simulate_impossible_turn,
)

# === Load aircraft data ===
def load_aircraft_data(folder="aircraft_data"):
    data = {}
    for filename in os.listdir(folder):
        if filename.endswith(".json"):
            with open(os.path.join(folder, filename)) as f:
                name = filename.replace(".json", "")
                data[name] = json.load(f)
    return data

aircraft_data = load_aircraft_data()
available_aircraft = sorted(aircraft_data.keys())

# === Load airport data ===
def load_airport_data():
    base = os.path.dirname(__file__)
    path = os.path.join(base, "airports", "airports.json")
    with open(path, "r") as f:
        return json.load(f)
airport_data = load_airport_data()

# === Dash App ===
app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.BOOTSTRAP],
    suppress_callback_exceptions=True,
    prevent_initial_callbacks=True
)
server = app.server

app.title = "Maneuver Overlay Tool | AeroEdge"

def legal_banner_block():
    return html.Div(
        children=[
            html.Div(
                "⚠️ This tool is for educational and training discussion only. It is not FAA-approved and may not reflect actual aircraft capabilities. "
                "Always verify against the aircraft POH or AFM and applicable regulations. ⚠️",
                className="disclaimer-banner",
            ),
            html.Div(
                children=[
                    html.A("Full Legal Disclaimer", href="#", id="open-disclaimer", className="legal-link"),
                    html.Span(" | ", className="legal-separator"),
                    html.A("Terms of Use & Privacy Policy", href="#", id="open-terms-policy", className="legal-link"),
                ],
                className="legal-links-row",
            ),

            dbc.Modal(
                [
                    dbc.ModalHeader("AeroEdge Disclaimer", close_button=True),
                    dbc.ModalBody(
                        [
                            html.P("This tool supplements, not replaces, FAA published documentation."),
                            html.P("It is intended for educational and reference use only and has not been approved or endorsed by the Federal Aviation Administration (FAA)."),
                            html.P("Do not use this tool for flight planning, aircraft operation, or regulatory compliance decisions."),
                            html.P("Outputs may be incomplete, inaccurate, outdated, or derived from public or user-provided inputs. No warranties are made regarding accuracy, completeness, or fitness for purpose."),
                            html.P("If any information conflicts with the aircraft FAA-approved AFM or POH, the official documentation shall govern."),
                            html.P("AeroEdge disclaims liability for errors, omissions, injuries, damages, or losses resulting from use of this application."),
                        ]
                    ),
                    dbc.ModalFooter(dbc.Button("Close", id="close-disclaimer", className="green-button")),
                ],
                id="disclaimer-modal",
                is_open=False,
                centered=True,
                size="lg",
                dialogClassName="aeroedge-modal",
                backdrop="static",
                scrollable=True,
            ),

            dbc.Modal(
                [
                    dbc.ModalHeader("Terms of Use & Privacy Policy", close_button=True),
                    dbc.ModalBody(
                        [
                            html.H6("Terms of Use", className="mb-2 mt-2"),
                            html.P("Use is for educational and informational purposes only. This tool is not FAA-certified."),
                            html.P("Verify all performance and procedural information using the POH or AFM and applicable regulations. Use is at your own risk."),
                            html.H6("Privacy Policy", className="mb-2 mt-4"),
                            html.P("No user accounts are required. The app does not intentionally collect personally identifiable information for functionality."),
                            html.P("Hosting providers may log basic operational metadata such as IP address, timestamps, and user agent for security and reliability."),
                        ]
                    ),
                    dbc.ModalFooter(dbc.Button("Close", id="close-terms-policy", className="green-button")),
                ],
                id="terms-policy-modal",
                is_open=False,
                centered=True,
                size="lg",
                dialogClassName="aeroedge-modal",
                backdrop="static",
                scrollable=True,
            ),
        ]
    )

app.layout = html.Div(className="full-height-container", children=[
    # Header
    html.Div(className="banner-header", children=[
        html.Div(className="banner-inner", children=[
            html.A(
                html.Img(src="/assets/logo.png", className="banner-logo"),
                href="https://www.flyaeroedge.com",
                style={"textDecoration": "none"}
            )
            
        ])
    ]),
    legal_banner_block(),
    # Main 2-column layout
    html.Div(className="main-row", children=[
        # === Sidebar ===
        html.Div(className="resizable-sidebar", children=[
            html.Div("Maneuver Overlay Tool", style={
                "fontWeight": "600",
                "fontSize": "20px",
                "marginBottom": "10px",
                "color": "#1b1e23"
            }),

            # --- Aircraft & Weight Section ---
            html.Label("Aircraft", className="input-label"),
            dcc.Dropdown(
                id="aircraft-select",
                className="dropdown",
                options=[{"label": name, "value": name} for name in available_aircraft],
                value=None,
                placeholder="Select Aircraft"
            ),

            html.Label("Engine Option", className="input-label"),
            dcc.Dropdown(id="engine-select", className="dropdown"),

            html.Label("Occupants", className="input-label"),
            dcc.Input(id="occupants", type="number", value=1, min=1, max=4, className="input-small"),

            html.Label("Occupant Weight (lbs)", className="input-label"),
            dcc.Input(id="occupant-weight", type="number", value=180, min=100, max=300, className="input-small"),

            html.Label("Fuel Load (gal)", className="input-label"),
            dcc.Slider(
                id="fuel-load",
                min=0,
                max=50,
                step=1,
                value=0,
                marks={0: "0", 12: "¼", 25: "½", 37: "¾", 50: "Full"},
                tooltip={"always_visible": True}
            ),

            html.Label("Total Weight (lbs)", className="input-label"),
            dcc.Input(
                id="total-weight-display",
                type="text",
                value="",
                readOnly=True,
                className="input-small",
                
            ),

            html.Label("CG Position", className="input-label"),
            dcc.Slider(
                id="cg-slider",
                min=0.0,
                max=1.0,
                step=0.01,
                value=0.5,
                marks={0.0: "FWD", 0.5: "MID", 1.0: "AFT"},
                tooltip={"always_visible": True}
            ),

            html.Label("Power Setting", className="input-label"),
            dcc.Slider(
                id="power-setting",
                min=0.05, max=1.0, step=0.05, value=0.5,
                marks={0.05: "IDLE", 0.2: "20%", 0.4: "40%", 0.6: "60%", 0.8: "80%", 0.99: "100%"},
                tooltip={"always_visible": True}
            ),

            html.Hr(),

            html.Div([
                html.Div("Environmentals", style={
                    "fontWeight": "600",
                    "fontSize": "16px",
                    "marginBottom": "10px",
                    "color": "#1b1e23"
                }),

                html.Label("Search Airport", className="input-label"),
                dcc.Input(
                    id="airport-search-input",
                    type="text",
                    placeholder="ICAO or name...",
                    debounce=True,
                    className="input-large"
                ),
                html.Div(id="airport-search-results", className="search-results-box"),
                html.Label("Airport Elevation (AGL ft)", className="input-label"),
                html.Div(id="env-airport-agl", className="weight-box", style={"marginBottom": "10px"}),

                html.Label("Outside Air Temp (°F)", className="input-label"),
                dcc.Input(id="env-oat", type="number", value=52, className="input-small"),

                html.Label("Altimeter Setting (inHg)", className="input-label"),
                dcc.Input(id="env-altimeter", type="number", value=29.92, className="input-small"),

                html.Label("Wind Direction (°)", className="input-label"),
                dcc.Input(id="env-wind-dir", type="number", value=360, className="input-small"),

                html.Label("Wind Speed (kt)", className="input-label"),
                dcc.Input(id="env-wind-speed", type="number", value=0, className="input-small"),
            ]),

            html.Hr(),

            # --- Maneuver Dropdown ---
            html.Label("Maneuver", className="input-label"),
            dcc.Dropdown(
                id="maneuver-select",
                className="dropdown",
                placeholder="Select Maneuver",
                options=[
                    {"label": "Impossible Turn", "value": "impossible_turn"},
                    {"label": "Power-Off 180", "value": "poweroff180"},
                    {"label": "Engine-Out Glide Simulation", "value": "engineout"},
                    {"label": "Steep Turns", "value": "steep_turn"},
                    {"label": "Chandelle", "value": "chandelle"},
                    {"label": "Lazy Eight", "value": "lazy8"},
                    {"label": "Steep Spiral", "value": "steep_spiral"},
                    {"label": "S-Turns", "value": "sturns"},
                    {"label": "Turns Around a Point", "value": "turns_point"},
                    {"label": "Rectangular Course", "value": "rect_course"},
                    {"label": "Eights on Pylons", "value": "pylons"},
                ]
            ),

            # --- Conditionally Shown Based on Maneuver ---
            html.Div(id="maneuver-params-container", children=[], style={"marginTop": "20px"}),

            html.Hr(),

            html.Button("Reset All", id="reset-all", className="green-button"),
            html.Button("Reset Click Points", id="reset-clicks", className="green-button")
        ]),

        # === Map Column ===
        html.Div(id="engineout-click-status", style={"display": "none"}),
        html.Div(className="graph-column", style={"display": "flex", "flexDirection": "column"}, children=[
            html.Div(
                style={
                    "flexGrow": 1,
                    "height": "calc(100vh - 180px)",
                    "position": "relative"
                },
                children=[
                    dl.Map(
                        id="map",
                        center=[33.0635, -80.2795],
                        zoom=13.5,
                        style={"width": "100%", "height": "100%"},
                        children=[
                            dl.TileLayer(),
                            dl.LayerGroup(id="layer")
                        ]
                    )
                ]
            ),

            html.Div(id="click_debug", style={
                "padding": "10px 12px",
                "fontStyle": "italic",
                "color": "#555",
                "backgroundColor": "#fff",
                "borderTop": "1px solid #ccc"
            }),

            # ===== Maneuver-scoped point stores (no shared state between maneuvers) =====

            dcc.Store(id="runtime-total-weight-lb"),

            # Power-Off 180 (touchdown only; start is auto-generated but keep for future flexibility)
            dcc.Store(id={"type": "point-store", "m_id": "poweroff180", "role": "touchdown"}),
            dcc.Store(id={"type": "point-store", "m_id": "poweroff180", "role": "start"}),

            # Engine-Out Glide
            dcc.Store(id={"type": "point-store", "m_id": "engineout", "role": "touchdown"}),
            dcc.Store(id={"type": "point-store", "m_id": "engineout", "role": "start"}),

            # Steep Turns
            dcc.Store(id={"type": "point-store", "m_id": "steep_turn", "role": "start"}),

            # Chandelle
            dcc.Store(id={"type": "point-store", "m_id": "chandelle", "role": "start"}),

            # Lazy Eight
            dcc.Store(id={"type": "point-store", "m_id": "lazy8", "role": "start"}),

            # Steep Spiral (center point)
            dcc.Store(id={"type": "point-store", "m_id": "steep_spiral", "role": "start"}),

            # S-Turns
            dcc.Store(id={"type": "point-store", "m_id": "sturns", "role": "start"}),

            # Turns Around a Point (center point)
            dcc.Store(id={"type": "point-store", "m_id": "turns_point", "role": "start"}),

            # Rectangular Course (start point)
            dcc.Store(id={"type": "point-store", "m_id": "rect_course", "role": "start"}),

            # Eights on Pylons (two pylons)
            dcc.Store(id={"type": "point-store", "m_id": "pylons", "role": "pylon_a"}),
            dcc.Store(id={"type": "point-store", "m_id": "pylons", "role": "pylon_b"}),

            # Impossible Turn (start only)
            dcc.Store(id={"type": "point-store", "m_id": "impossible_turn", "role": "start"}),
            dcc.Store(id="active-click-target"),
            dcc.Store(id="selected-airport-id"),

            html.Div("© 2025 Nicholas Len, AEROEDGE. All rights reserved.",
                     className="footer", style={"paddingBottom": "10px"})
        ])
    ])
])

# === Maneuver UI Layouts ===
def impossible_turn_layout():
    return [

        html.Div(
            [
                html.Div(
                    "Assumptions: engine fails at the selected point while tracking selected runway heading upwind. "
                    "Model applies reaction delay, transitions toward best glide, and attempts to intercept opposite  "
                    "runway heading.",
                    style={"fontStyle": "italic", "color": "#555"}
                ),

            ],
            style={"marginBottom": "10px"}
        ),

        html.Label("Turn Direction", className="input-label"),
        dcc.RadioItems(
            id="impossibleturn-direction",
            options=[
                {"label": "Left", "value": "left"},
                {"label": "Right", "value": "right"},
            ],
            value="left",
            inline=True,
            className="radio-inline-group"
        ),

        html.Label("Runway Heading (°)", className="input-label"),
        dcc.Input(
            id="impossibleturn-touchdown-heading",
            type="number",
            value=240,
            min=0,
            max=360,
            step=1,
            className="input-small"
        ),

        html.Label("Engine Failure Altitude (ft AGL)", className="input-label"),
        dcc.Input(
            id="impossibleturn-altitude",
            type="number",
            value=1000,
            min=0,
            step=10,
            className="input-small"
        ),

        html.Label("Entry IAS at Failure (KIAS)", className="input-label"),
        dcc.Input(
            id="impossibleturn-entry-ias",
            type="number",
            value=75,
            min=0,
            step=1,
            className="input-small"
        ),

        html.Label("Reaction Delay (sec)", className="input-label"),
        dcc.Input(
            id="impossibleturn-reaction-sec",
            type="number",
            value=3.0,
            min=0.0,
            step=0.5,
            className="input-small"
        ),

                # Flaps
        html.Label("Flap configuration"),
        dcc.Dropdown(
            id="impossibleturn-flap-config",
            options=[
                {"label": "Clean", "value": "clean"},
                {"label": "Takeoff", "value": "takeoff"},
                {"label": "Landing", "value": "landing"},
            ],
            value="clean",
            clearable=False,
            searchable=False,
        ),

        # Prop
        html.Label("Prop condition"),
        dcc.Dropdown(
            id="impossibleturn-prop-config",
            options=[
                {"label": "Idle", "value": "idle"},
                {"label": "Windmilling", "value": "windmilling"},
                {"label": "Prop Stopped", "value": "stationary"},
                {"label": "Feathered", "value": "feathered"}
            ],
            value="windmilling",
            clearable=False,
            searchable=False,
        ),

        html.Hr(),

        html.Button(
            "Set Engine Failure Point",
            id={"type": "click-button", "m_id": "impossible_turn", "role": "start"},
            className="green-button"
        ),

        html.Br(),
        html.Br(),

        html.Button(
            "Draw Impossible Turn",
            id="impossibleturn-draw-btn",
            className="blue-button"
        ),

        html.Div(
            id={"type": "click-status", "m_id": "impossible_turn"},
            style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"},
        ),

        html.Div(
            id="impossibleturn-result",
            className="weight-box",
            style={"marginTop": "10px"}
        ),
    ]
def poweroff180_layout(default_elev=None):
    return [
        html.Label("Pattern Direction", className="input-label"),
        dcc.RadioItems(
            id="poweroff180-pattern",
            options=[
                {"label": "Left", "value": "left"},
                {"label": "Right", "value": "right"}
            ],
            value="left",
            inline=True,
            className="radio-inline-group"
        ),

        html.Label("Flap Setting", className="input-label"),
        dcc.Dropdown(
            id="poweroff180-flap-setting",
            options=[
                {"label": "Clean", "value": "clean"},
                {"label": "Takeoff", "value": "takeoff"},
                {"label": "Landing", "value": "landing"}
            ],
            value="clean",
            className="dropdown"
        ),

        html.Label("Prop Condition", className="input-label"),
        dcc.Dropdown(
            id="poweroff180-prop-condition",
            options=[
                {"label": "Idle", "value": "idle"},
                {"label": "Windmilling", "value": "windmilling"},
                {"label": "Prop Stopped", "value": "stationary"},
                {"label": "Feathered", "value": "feathered"}
            ],
            value="idle",
            className="dropdown"
        ),

        html.Label("Touchdown Heading (°)", className="input-label"),
        dcc.Input(
            id="poweroff180-touchdown-heading",
            type="number",
            value=60,
            className="input-small"
        ),

        html.Label("Start Distance From Touchdown (NM)", className="input-label"),
        dcc.Slider(
            id="poweroff180-start-distance-nm",
            min=0.3,
            max=2.5,
            step=0.1,
            value=0.8,
            marks={
                0.5: "0.5",
                1.0: "1.0",
                1.5: "1.5",
                2.0: "2.0",
                2.5: "2.5",
            },
            tooltip={"always_visible": True}
        ),

        html.Label("Start Altitude (ft AGL)", className="input-label"),
        dcc.Input(
            id="poweroff180-altitude",
            type="number",
            value=1000,
            className="input-small"
        ),

        html.Hr(),

        html.Button(
            "Set Touchdown Point",
            id={"type": "click-button", "m_id": "poweroff180", "role": "touchdown"},
            className="green-button"
        ),
        # NOTE: removed "Set Start Point" – start point is now auto-generated
        html.Div(
            id={"type": "click-status", "m_id": "poweroff180"},
            style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"}
        ),
        html.Button("Draw Power-Off 180", id="poweroff180-draw-btn", className="green-button"),
    ]

def engineout_layout():
    return [

        # ---- Configuration Inputs ----
        html.Label("Flap Setting", className="input-label"),
        dcc.Dropdown(
            id="engineout-flap-setting",
            options=[
                {"label": "Clean", "value": "clean"},
                {"label": "Takeoff", "value": "takeoff"},
                {"label": "Landing", "value": "landing"},
            ],
            value="clean",
            className="dropdown"
        ),

        html.Label("Prop Condition", className="input-label"),
        dcc.Dropdown(
            id="engineout-prop-condition",
            options=[
                {"label": "Idle", "value": "idle"},
                {"label": "Windmilling", "value": "windmilling"},
                {"label": "Prop Stopped", "value": "stationary"},
                {"label": "Feathered", "value": "feathered"},
            ],
            value="idle",
            className="dropdown"
        ),

        html.Label("Touchdown Heading (°)", className="input-label"),
        dcc.Input(
            id="engineout-touchdown-heading",
            type="number",
            value=60,
            className="input-small"
        ),

        # ✅ NEW: Pattern direction toggle
        html.Label("Pattern Direction", className="input-label"),
        dcc.RadioItems(
            id="engineout-pattern-dir",
            options=[
                {"label": "Left Pattern", "value": "left"},
                {"label": "Right Pattern", "value": "right"},
            ],
            value="left",
            labelStyle={"display": "inline-block", "margin-right": "12px"},
            className="radio-items"
        ),

        html.Label("Touchdown Elevation (ft)", className="input-label"),
        dcc.Input(
            id="engineout-manual-elev",
            type="number",
            placeholder="map click",
            className="input-small"
        ),

        html.Label("Start Heading (°)", className="input-label"),
        dcc.Input(
            id="engineout-start-heading",
            type="number",
            value=240,
            className="input-small"
        ),

        html.Label("Start Altitude (ft AGL)", className="input-label"),
        dcc.Input(
            id="engineout-altitude",
            type="number",
            value=1000,
            className="input-small"
        ),

        html.Hr(),

        # ---- Map Interaction Buttons ----
        html.Button(
            "Set Touchdown Point",
            id={"type": "click-button", "m_id": "engineout", "role": "touchdown"},
            className="green-button"
        ),
        html.Button(
            "Set Start Point",
            id={"type": "click-button", "m_id": "engineout", "role": "start"},
            className="green-button"
        ),

        html.Br(), html.Br(),

        # ---- ★ NEW: DRAW ENGINE-OUT PATH BUTTON ----
        html.Button(
            "Draw Engine-Out Glide Path",
            id="engineout-draw-btn",
            className="blue-button",
            style={"marginTop": "10px"},
        ),

        html.Div(
            id={"type": "click-status", "m_id": "engineout"},
            style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"},
        ),
    ]
def steep_turn_layout():
    return [
        html.Label("Bank Angle (°)", className="input-label"),
        dcc.Input(
            id="steepturn-bank-angle",
            type="number",
            value=45,
            min=30,
            max=60,
            className="input-small"
        ),

        html.Label("Turn Sequence", className="input-label"),
        dcc.RadioItems(
            id="steepturn-sequence",
            options=[
                {"label": "Left → Right", "value": "left-right"},
                {"label": "Right → Left", "value": "right-left"},
                {"label": "Left Only", "value": "left"},
                {"label": "Right Only", "value": "right"}
            ],
            value="left-right",
            inline=True,
            className="radio-inline-group"
        ),

        html.Label("Entry Heading (°)", className="input-label"),
        dcc.Input(
            id="steepturn-entry-heading",
            type="number",
            value=0,
            className="input-small"
        ),

        html.Label("Entry Altitude (ft AGL)", className="input-label"),
        dcc.Input(
            id="steepturn-altitude",
            type="number",
            placeholder="Optional",
            className="input-small"
        ),

        html.Label("Entry Speed (KIAS)", className="input-label"),
        dcc.Input(
            id="steepturn-ias",
            type="number",
            placeholder="e.g. Va",
            className="input-small"
        ),

        html.Hr(),

        html.Div([
            html.Button("Set Entry Point", id={"type": "click-button", "m_id": "steep_turn", "role": "start"}, className="green-button"),
            html.Button("Draw Steep Turn", id="steepturn-draw-btn", className="blue-button", style={"marginLeft": "10px"})
        ], style={"display": "flex", "alignItems": "center"}),

        html.Div(id={"type": "click-status", "m_id": "steep_turn"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def chandelle_layout():
    return [
        html.Label("Entry Heading (°)", className="input-label"),
        dcc.Input(
            id="chandelle-entry-heading",
            type="number",
            value=0,
            className="input-small"
        ),

        html.Label("Bank Angle (°)", className="input-label"),
        dcc.Input(
            id="chandelle-bank-angle",
            type="number",
            value=30,
            min=15,
            max=45,
            className="input-small"
        ),

        html.Label("Turn Direction", className="input-label"),
        dcc.RadioItems(
            id="chandelle-direction",
            options=[
                {"label": "Left", "value": "left"},
                {"label": "Right", "value": "right"}
            ],
            value="right",
            inline=True,
            className="radio-inline-group"
        ),

        html.Label("Entry Altitude (ft)", className="input-label"),
        dcc.Input(
            id="chandelle-altitude",
            type="number",
            value=3000,
            className="input-small"
        ),

        html.Hr(),

        html.Button("Set Entry Point", id={"type": "click-button", "m_id": "chandelle", "role": "start"}, className="green-button"),
        html.Div(id={"type": "click-status", "m_id": "chandelle"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def lazy8_layout():
    return [
        html.Label("Entry Heading (°)", className="input-label"),
        dcc.Input(
            id="lazy8-entry-heading",
            type="number",
            value=0,
            className="input-small"
        ),

        html.Label("Entry Altitude (ft)", className="input-label"),
        dcc.Input(
            id="lazy8-entry-altitude",
            type="number",
            value=3000,
            className="input-small"
        ),

        html.Label("Maneuver Symmetry / Direction", className="input-label"),
        dcc.RadioItems(
            id="lazy8-direction-sequence",
            options=[
                {"label": "Left → Right", "value": "left-right"},
                {"label": "Right → Left", "value": "right-left"},
                {"label": "Left Only", "value": "left"},
                {"label": "Right Only", "value": "right"}
            ],
            value="left-right",
            inline=True,
            className="radio-inline-group"
        ),

        html.Hr(),

        html.Button("Set Entry Point", id={"type": "click-button", "m_id": "lazy8", "role": "start"}, className="green-button"),
        html.Div(id={"type": "click-status", "m_id": "lazy8"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def steep_spiral_layout():
    return [
        html.Label("Number of Turns", className="input-label"),
        dcc.Input(
            id="steepspiral-turns",
            type="number",
            value=3,
            min=1,
            max=10,
            step=1,
            className="input-small"
        ),

        html.Label("Entry Heading (°)", className="input-label"),
        dcc.Input(
            id="steepspiral-entry-heading",
            type="number",
            value=0,
            className="input-small"
        ),

        html.Label("Entry Altitude (ft)", className="input-label"),
        dcc.Input(
            id="steepspiral-altitude",
            type="number",
            value=3000,
            className="input-small"
        ),

        html.Label("Target Airspeed (KIAS)", className="input-label"),
        dcc.Input(
            id="steepspiral-airspeed",
            type="number",
            value=80,
            className="input-small"
        ),

        html.Label("Bank Angle (°)", className="input-label"),
        dcc.Input(
            id="steepspiral-bank-angle",
            type="number",
            value=45,
            min=20,
            max=60,
            className="input-small"
        ),

        html.Hr(),

        html.Button("Set Spiral Center", id={"type": "click-button", "m_id": "steepspiral", "role": "start"}, className="green-button"),
        html.Div(id={"type": "click-status", "m_id": "steepspiral"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def sturns_layout():
    return [
        html.Label("Entry Heading (°)", className="input-label"),
        dcc.Input(
            id="sturns-entry-heading",
            type="number",
            value=0,
            className="input-small"
        ),

        html.Label("Entry Altitude (ft)", className="input-label"),
        dcc.Input(
            id="sturns-altitude",
            type="number",
            value=1000,
            className="input-small"
        ),

        html.Label("Turn Sequence", className="input-label"),
        dcc.RadioItems(
            id="sturns-sequence",
            options=[
                {"label": "Left → Right", "value": "left-right"},
                {"label": "Right → Left", "value": "right-left"}
            ],
            value="left-right",
            inline=True,
            className="radio-inline-group"
        ),

        html.Label("Number of Arc Pairs", className="input-label"),
        dcc.Input(
            id="sturns-arcs",
            type="number",
            value=2,
            min=1,
            max=5,
            step=1,
            className="input-small"
        ),

        html.Hr(),

        html.Button("Set Entry Point", id={"type": "click-button", "m_id": "sturns", "role": "start"}, className="green-button"),
        html.Div(id={"type": "click-status", "m_id": "sturns"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def turns_point_layout():
    return [
        html.Label("Ground Speed (kt)", className="input-label"),
        dcc.Input(id="global-groundspeed", type="number", value=90, className="input-small"),

        html.Label("Calculated Pivotal Altitude (ft)", className="input-label"),
        html.Div(id="global-pivotal-altitude", className="weight-box", style={"marginBottom": "10px"}),

        html.Label("Turn Direction", className="input-label"),
        dcc.RadioItems(
            id="turns-point-direction",
            options=[
                {"label": "Left", "value": "left"},
                {"label": "Right", "value": "right"}
            ],
            value="left",
            inline=True,
            className="radio-inline-group"
        ),

        html.Label("Entry Heading (°)", className="input-label"),
        dcc.Input(
            id="turns-point-entry-heading",
            type="number",
            value=0,
            className="input-small"
        ),

        html.Hr(),

        html.Button("Set Center Point", id={"type": "click-button", "m_id": "turns_point", "role": "start"}, className="green-button"),
        html.Div(id={"type": "click-status", "m_id": "turns_point"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def rect_course_layout():
    return [
        html.Label("Entry Altitude (ft)", className="input-label"),
        dcc.Input(
            id="rectcourse-altitude",
            type="number",
            value=1000,
            className="input-small"
        ),

        html.Hr(),

        html.Button("Draw Rectangular Course", id="rectcourse-draw-btn", className="green-button"),
        html.Button("Set Start Point", id={"type": "click-button", "m_id": "rectcourse", "role": "start"}, className="green-button", style={"marginTop": "10px"}),
        html.Div(id="rectcourse-heading-selector", children=[], style={"marginTop": "12px"}),
        html.Div(id={"type": "click-status", "m_id": "rectcourse"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"})
    ]

def pylons_layout():
    return [
        html.Label("Ground Speed (kt)", className="input-label"),
        dcc.Input(id="global-groundspeed", type="number", value=90, className="input-small"),

        html.Label("Calculated Pivotal Altitude (ft)", className="input-label"),
        html.Div(id="global-pivotal-altitude", className="weight-box", style={"marginBottom": "10px"}),

        html.Label("Entry Altitude (ft)", className="input-label"),
        dcc.Input(
            id="pylons-altitude",
            type="number",
            value=1200,
            className="input-small"
        ),

        html.Div("Click to select pylon locations on the map:", style={
            "fontWeight": "bold",
            "marginTop": "12px"
        }),

        html.Button("Set Pylon A", id={"type": "click-button", "m_id": "pylons", "role": "pylon_a"}, className="green-button"),
        html.Button("Set Pylon B", id={"type": "click-button", "m_id": "pylons", "role": "pylon_b"}, className="green-button", style={"marginLeft": "10px"}),
        html.Div(id={"type": "click-status", "m_id": "pylons"}, style={"marginTop": "10px", "fontStyle": "italic", "color": "#555"}),
        html.Label("Start Heading (Select One)", className="input-label", style={"marginTop": "12px"}),
        html.Div(id="pylons-heading-options", children=[])
    ]

# === Utility Callbacks ===


from dash import callback, Input, Output

@callback(
    Output("total-weight-display", "value"),
    Output("runtime-total-weight-lb", "data"),
    Input("aircraft-select", "value"),
    Input("occupants", "value"),
    Input("occupant-weight", "value"),
    Input("fuel-load", "value"),
)
def update_total_weight_display(ac_name, occupants, occupant_wt, fuel_gal):
    if not ac_name or ac_name not in aircraft_data:
        return "", None

    ac = aircraft_data[ac_name]
    empty_wt = float(ac.get("empty_weight", 0.0))
    fuel_per_gal = float(ac.get("fuel_weight_per_gal", 6.0))

    occ = float(occupants or 0)
    occ_wt = float(occupant_wt or 0)
    fuel = float(fuel_gal or 0)

    total = empty_wt + (occ * occ_wt) + (fuel * fuel_per_gal)
    total_round = int(round(total))

    return f"{total_round}", total


@app.callback(
    Output("map", "center"),
    Output("env-airport-agl", "children"),
    Output("selected-airport-id", "data"),
    Output("airport-search-input", "value"),
    Input({"type": "airport-result", "index": ALL}, "n_clicks"),
    prevent_initial_call=True
)
def handle_airport_result_click(n_clicks_list):
    if not ctx.triggered_id or not isinstance(ctx.triggered_id, dict):
        raise PreventUpdate

    airport_id = ctx.triggered_id.get("index")
    ap = next((a for a in airport_data if a.get("id") == airport_id), None)
    if not ap:
        raise PreventUpdate

    lat, lon = ap["lat"], ap["lon"]
    elev = ap.get("elevation_ft", "---")

    # Fill the input with the selected airport ID
    return [lat, lon], f"{elev} ft", airport_id, airport_id

@app.callback(
    Output("airport-search-results", "children"),
    Input("airport-search-input", "value")
)
def search_airport_database(query):
    if not query or len(query.strip()) < 2:
        return []

    q = query.strip().lower()

    # If the input is already an exact airport ID, hide the dropdown.
    exact = next((ap for ap in airport_data if ap.get("id", "").lower() == q), None)
    if exact is not None:
        return []

    matches = []
    for ap in airport_data:
        ap_id = ap.get("id", "").lower()
        ap_name = ap.get("name", "").lower()
        if q in ap_id or q in ap_name:
            matches.append(ap)
        if len(matches) >= 10:
            break

    return [
        html.Div(
            f"{ap['name']} ({ap['id']})",
            className="airport-result",
            id={"type": "airport-result", "index": ap["id"]},
            n_clicks=0
        )
        for ap in matches
    ]



# === Maneuver Dispatcher ===
@app.callback(
    Output("maneuver-params-container", "children"),
    Input("maneuver-select", "value"),
    State("selected-airport-id", "data")
)
def render_maneuver_layout(maneuver, airport_id):
    elev_ft = None
    if airport_id:
        ap = next((a for a in airport_data if a["id"] == airport_id), None)
        elev_ft = ap.get("elevation_ft", None) if ap else None

    if maneuver == "impossible_turn":
        return impossible_turn_layout()
    elif maneuver == "poweroff180":
        return poweroff180_layout(default_elev=elev_ft)
    elif maneuver == "engineout":
        return engineout_layout()
    elif maneuver == "steep_turn":
        return steep_turn_layout()
    elif maneuver == "chandelle":
        return chandelle_layout()
    elif maneuver == "lazy8":
        return lazy8_layout()
    elif maneuver == "steep_spiral":
        return steep_spiral_layout()
    elif maneuver == "sturns":
        return sturns_layout()
    elif maneuver == "turns_point":
        return turns_point_layout()
    elif maneuver == "rect_course":
        return rect_course_layout()
    elif maneuver == "pylons":
        return pylons_layout()
    return []

# === Aircraft Fields Update ===
@app.callback(
    Output("engine-select", "options"),
    Output("engine-select", "value"),
    Output("occupants", "value"),
    Output("occupant-weight", "value"),
    Output("fuel-load", "max"),
    Output("fuel-load", "value"),
    Output("fuel-load", "marks"),
    Output("cg-slider", "min"),
    Output("cg-slider", "max"),
    Output("cg-slider", "value"),
    Output("cg-slider", "marks"),
    Input("aircraft-select", "value")
)
def update_aircraft_fields(selected_aircraft):
    if not selected_aircraft or selected_aircraft not in aircraft_data:
        return (
            [], None, 1, 180,
            50, 50,
            {0: "0", 12: "¼", 25: "½", 37: "¾", 50: "Full"},
            0.0, 1.0, 0.5,
            {0.0: "FWD", 0.5: "MID", 1.0: "AFT"}
        )

    ac = aircraft_data[selected_aircraft]
    engine_options = [{"label": k, "value": k} for k in ac.get("engine_options", {}).keys()]
    default_engine = engine_options[0]["value"] if engine_options else None
    seats = ac.get("seats", 2)
    default_occupants = min(seats, 2)
    default_weight = 180
    fuel_cap = ac.get("fuel_capacity_gal", 50)
    fuel_marks = {
        0: "0",
        int(0.25 * fuel_cap): "¼",
        int(0.5 * fuel_cap): "½",
        int(0.75 * fuel_cap): "¾",
        int(fuel_cap): "Full"
    }
    cg_range = ac.get("cg_range", [0.0, 1.0])
    cg_min = cg_range[0]
    cg_max = cg_range[1]
    cg_default = round((cg_min + cg_max) / 2, 2)
    cg_marks = {
        round(cg_min, 2): "FWD",
        round((cg_min + cg_max) / 2, 2): "MID",
        round(cg_max, 2): "AFT"
    }
    return (engine_options, default_engine, default_occupants, default_weight, fuel_cap,
            fuel_cap, fuel_marks, cg_min, cg_max, cg_default, cg_marks)

# === Click Target Registry ===
click_target_registry = {
    "poweroff180": {
        "buttons": {
            "poweroff180-touchdown-btn": "touchdown",
            "poweroff180-start-btn": "start"
        },
        "status_id": "poweroff180-click-status"
    },
    "engineout": {
        "buttons": {
            "engineout-touchdown-btn": "touchdown",
            "engineout-start-btn": "start"
        },
        "status_id": "engineout-click-status"
    },
    "steep_turn": {
        "buttons": {
            "steepturn-start-btn": "start"
        },
        "status_id": "steep_turn-click-status"
    },
    # Add other maneuvers as needed
}

from dash import no_update
from dash.exceptions import PreventUpdate

@app.callback(
    Output("active-click-target", "data", allow_duplicate=True),
    Input({"type": "click-button", "m_id": ALL, "role": ALL}, "n_clicks"),
    State("maneuver-select", "value"),
    prevent_initial_call=True
)
def set_active_click_target(n_clicks_list, maneuver):
    if not ctx.triggered_id:
        raise PreventUpdate

    trig = ctx.triggered_id  # dict: {type, m_id, role}
    if not isinstance(trig, dict):
        raise PreventUpdate

    # Only accept clicks for the currently selected maneuver
    if trig.get("m_id") != maneuver:
        raise PreventUpdate

    return {"m_id": trig.get("m_id"), "role": trig.get("role")}

@app.callback(
    Output({"type": "click-status", "m_id": MATCH}, "children", allow_duplicate=True),
    Input({"type": "click-button", "m_id": MATCH, "role": ALL}, "n_clicks"),
    prevent_initial_call=True
)
def show_click_prompt(n_clicks_list):
    ctx = dash.callback_context
    if not ctx.triggered:
        raise PreventUpdate

    triggered_id = ctx.triggered_id
    role = triggered_id.get("role")
    return f"🖱 Click on the map to set the {role.replace('_', ' ')} point."

from dash import no_update
from dash.exceptions import PreventUpdate

from dash import no_update
from dash.exceptions import PreventUpdate

from dash import no_update
from dash.exceptions import PreventUpdate

@app.callback(
    Output({"type": "point-store", "m_id": ALL, "role": ALL}, "data", allow_duplicate=True),
    Output("active-click-target", "data", allow_duplicate=True),
    Output("layer", "children", allow_duplicate=True),
    Input("map", "clickData"),
    State("active-click-target", "data"),
    State({"type": "point-store", "m_id": ALL, "role": ALL}, "id"),
    State({"type": "point-store", "m_id": ALL, "role": ALL}, "data"),
    State("layer", "children"),
    prevent_initial_call=True
)
def write_point_to_scoped_store(click, target, store_ids, store_data, layer_children):
    if not click or "latlng" not in click or not isinstance(target, dict):
        raise PreventUpdate

    m_id = target.get("m_id")
    role = target.get("role")
    if not m_id or not role:
        raise PreventUpdate

    lat = click["latlng"]["lat"]
    lon = click["latlng"]["lng"]
    elev = get_elevation(lat, lon)

    new_pt = {"lat": lat, "lon": lon, "elevation_ft": elev}

    # ----- write to the correct scoped store -----
    updated = list(store_data)  # same order as store_ids
    wrote = False
    for i, sid in enumerate(store_ids):
        if isinstance(sid, dict) and sid.get("m_id") == m_id and sid.get("role") == role:
            updated[i] = new_pt
            wrote = True
            break

    if not wrote:
        raise PreventUpdate

    # ----- immediate marker on the map -----
    layer_children = layer_children or []

    marker_id = {"type": "pt-marker", "m_id": m_id, "role": role}

    kept = []
    for child in layer_children:
        try:
            # Drop any existing marker for this exact (m_id, role)
            if getattr(child, "id", None) != marker_id:
                kept.append(child)
        except Exception:
            kept.append(child)

    # Color convention
    color = "green"
    if role == "touchdown":
        color = "red"
    elif role in ("impact", "failure", "engine_failure"):
        color = "black"

    marker = dl.CircleMarker(
        id=marker_id,
        center=[lat, lon],
        radius=7,
        color=color,
        fill=True,
        fillOpacity=1.0,
        children=dl.Tooltip(f"{m_id} {role}: {lat:.5f}, {lon:.5f}")
    )

    new_layer = kept + [marker]

    # Clear target after one successful click so extra clicks don’t overwrite
    return updated, None, new_layer

@app.callback(
    Output({"type": "click-status", "m_id": MATCH}, "children", allow_duplicate=True),
    Input({"type": "point-store", "m_id": MATCH, "role": ALL}, "data"),
    State({"type": "point-store", "m_id": MATCH, "role": ALL}, "id"),
    prevent_initial_call=True
)
def summarize_points(points, ids):
    parts = []
    for pid, pdata in zip(ids, points):
        role = pid.get("role", "point").replace("_", " ")
        if isinstance(pdata, dict) and pdata.get("lat") is not None and pdata.get("lon") is not None:
            lat = pdata["lat"]
            lon = pdata["lon"]
            elev = pdata.get("elevation_ft")
            if role == "touchdown" and elev is not None:
                parts.append(f"✅ {role} set ({lat:.4f}, {lon:.4f}) elev {int(round(elev))} ft")
            else:
                parts.append(f"✅ {role} set ({lat:.4f}, {lon:.4f})")
        else:
            parts.append(f"⬜ {role} not set")
    return " | ".join(parts)

# === Elevation ===

from dash import no_update
from dash.exceptions import PreventUpdate

@app.callback(
    Output("engineout-manual-elev", "value"),
    Input({"type": "point-store", "m_id": "engineout", "role": "touchdown"}, "data"),
    State("maneuver-select", "value"),
    prevent_initial_call=True
)
def autofill_engineout_touchdown_elev(td_data, maneuver):
    if maneuver != "engineout":
        raise PreventUpdate
    if not isinstance(td_data, dict):
        raise PreventUpdate

    elev = td_data.get("elevation_ft")
    if elev is None:
        raise PreventUpdate

    return int(round(elev))

def get_elevation(lat, lon):
    if lat is None or lon is None:
        return None

    try:
        url = "https://api.open-meteo.com/v1/elevation"
        r = requests.get(url, params={"latitude": lat, "longitude": lon}, timeout=5)
        r.raise_for_status()
        data = r.json()

        elev_m = data.get("elevation")
        if isinstance(elev_m, list) and elev_m:
            elev_m = elev_m[0]

        if elev_m is None:
            return None

        return int(round(float(elev_m) * 3.28084))
    except Exception as e:
        print(f"❌ Open-Meteo elevation lookup failed: {e}")
        return None

@app.callback(
    Output("click_debug", "children"),
    Input("map", "clickData"),
    prevent_initial_call=True
)
def display_click_location(click):
    if not click or "latlng" not in click:
        raise dash.exceptions.PreventUpdate

    lat = click["latlng"]["lat"]
    lon = click["latlng"]["lng"]
    return f"🗺️ Location clicked: {lat:.5f}, {lon:.5f}"


# === Impossible Turn Rendering Callback ===
@app.callback(
    Output("layer", "children", allow_duplicate=True),
    Output("map", "bounds", allow_duplicate=True),
    Output({"type": "click-status", "m_id": "impossible_turn"}, "children", allow_duplicate=True),
    Output("impossibleturn-result", "children", allow_duplicate=True),
    Input("impossibleturn-draw-btn", "n_clicks"),
    State({"type": "point-store", "m_id": "impossible_turn", "role": "start"}, "data"),
    State("aircraft-select", "value"),
    State("engine-select", "value"),
    State("occupants", "value"),
    State("occupant-weight", "value"),
    State("fuel-load", "value"),
    State("cg-slider", "value"),
    State("env-wind-dir", "value"),
    State("env-wind-speed", "value"),
    State("env-oat", "value"),
    State("env-altimeter", "value"),
    State("impossibleturn-direction", "value"),
    State("impossibleturn-touchdown-heading", "value"),
    State("impossibleturn-altitude", "value"),
    State("impossibleturn-reaction-sec", "value"),
    State("impossibleturn-entry-ias", "value"),
    State("impossibleturn-flap-config", "value"),
    State("impossibleturn-prop-config", "value"),
    State("selected-airport-id", "data"),
    State("runtime-total-weight-lb", "data"),
    prevent_initial_call=True,
)
def draw_impossible_turn(
    n_clicks,
    failure_data,
    ac_name,
    engine_key,
    occupants,
    occupant_wt,
    fuel_gal,
    cg_pos,
    wind_dir,
    wind_speed,
    oat_f,
    altimeter,
    turn_dir,
    runway_heading,
    failure_alt_agl,
    reaction_sec,
    entry_ias,
    flap_config,
    prop_config,
    selected_airport_id,
    runtime_weight,
):
    if not n_clicks:
        raise PreventUpdate

    if not failure_data:
        return [], None, "⚠️ Set engine failure point first.", ""

    if not ac_name or not engine_key:
        return [], None, "⚠️ Select aircraft and engine first.", ""

    try:
        states = dash.callback_context.states

        def safe_float(state_key):
            v = states.get(state_key)
            return float(v) if v not in [None, "", "null"] else None

        runway_heading  = safe_float("impossibleturn-touchdown-heading.value")
        failure_alt_agl = safe_float("impossibleturn-altitude.value")
        reaction_sec    = safe_float("impossibleturn-reaction-sec.value")
        entry_ias       = safe_float("impossibleturn-entry-ias.value")
        wind_dir        = safe_float("env-wind-dir.value")
        wind_speed      = safe_float("env-wind-speed.value")
        oat_f           = safe_float("env-oat.value")
        altimeter       = safe_float("env-altimeter.value")

        # Weight: use runtime store only
        total_wt = safe_float("runtime-total-weight-lb.data")
        if total_wt is None:
            # fallback to State value if callback_context didn't capture it
            total_wt = float(runtime_weight) if runtime_weight not in [None, "", "null"] else None

        required = [
            runway_heading, failure_alt_agl, reaction_sec, entry_ias,
            wind_dir, wind_speed, oat_f, altimeter,
            total_wt,
        ]
        if any(x is None for x in required):
            return [], None, "⚠️ Missing or invalid inputs.", ""

        failure_pt = GeoPoint(failure_data["lat"], failure_data["lon"])

        # Aircraft dict copy + stash runtime weight (no JSON changes)
        ac = dict(aircraft_data[ac_name])
        ac["total_weight_lb"] = float(total_wt)

        # Airport elevation reference
        selected_airport = next((a for a in airport_data if a.get("id") == selected_airport_id), None)
        airport_elev_ft = float(selected_airport.get("elevation_ft", 0.0)) if selected_airport else 0.0

        # OAT F -> C
        oat_c = (float(oat_f) - 32.0) * 5.0 / 9.0

        path, hover, meta = simulate_impossible_turn(
            start_point=failure_pt,
            runway_heading_deg=float(runway_heading),
            turn_dir=str(turn_dir),
            reaction_sec=float(reaction_sec),
            start_ias_kias=float(entry_ias),
            altitude_agl=float(failure_alt_agl),

            ac=ac,
            engine_option=engine_key,
            weight_lbs=float(total_wt),
            oat_c=float(oat_c),
            altimeter_inhg=float(altimeter),
            wind_dir=float(wind_dir),
            wind_speed=float(wind_speed),
            timestep_sec=0.5,
            flap_config=flap_config,
            prop_config=prop_config,
            touchdown_elev_ft=float(airport_elev_ft),
            find_min_alt=True,
        )

        if not path:
            return [], None, "⚠️ No path generated. Check inputs.", ""

        # Meta
        meta = meta or {}
        made_it      = bool(meta.get("success", False))
        impact       = meta.get("impact_marker", None)
        reason       = meta.get("reason", "unknown")
        min_required = meta.get("min_feasible_alt_agl", None)
        bank_deg     = meta.get("bank_deg", None)
        # Distance (NM): failure point -> impact (if any) else -> end of path
        dist_nm = None
        try:
            if impact and isinstance(impact, (list, tuple)) and len(impact) == 2:
                end_pt = GeoPoint(float(impact[0]), float(impact[1]))
                dist_nm = geo_distance(failure_pt, end_pt).nm
                dist_label = "Failure distance to impact"
            else:
                end_lat, end_lon = path[-1][0], path[-1][1]
                end_pt = GeoPoint(float(end_lat), float(end_lon))
                dist_nm = geo_distance(failure_pt, end_pt).nm
                dist_label = "Failure distance to touchdown"
        except Exception:
            dist_nm = None
            dist_label = "Distance"

        dist_txt = f"{dist_label}: {dist_nm:.2f} NM" if isinstance(dist_nm, (int, float)) else f"{dist_label}: n/a"
        
        # Markers
        start_marker = dl.CircleMarker(
            center=[failure_pt.latitude, failure_pt.longitude],
            radius=7,
            color="green",
            fill=True,
            fillOpacity=1.0,
            children=dl.Tooltip("Engine failure point"),
        )

        elements = [start_marker]

        # ---------- Core visuals: full glide track + hover markers ----------
        arc_line = dl.Polyline(positions=path, color="red", weight=3)

        hover_markers = []
        if hover and isinstance(hover, list):
            for i, pt in enumerate(hover):
                # thin markers (match PO180 pattern). Change 5 -> 1 if you truly want every point.
                if i % 5 != 0 or i >= len(path):
                    continue

                # Safely pull fields
                alt   = pt.get("alt")
                tas   = pt.get("tas")
                gs    = pt.get("gs")
                t_sec = pt.get("time")
                aob   = pt.get("aob")
                vs    = pt.get("vs")
                track = pt.get("track")
                hdg   = pt.get("heading")
                drift = pt.get("drift")

                # Build tooltip: ALL rounded to 0 decimals except time if you want tenths
                tooltip_children = []

                if alt is not None:
                    tooltip_children.append(html.Div(f"{float(alt):.0f} ft AGL"))
                if tas is not None:
                    tooltip_children.append(html.Div(f"TAS: {float(tas):.0f} kt"))
                if gs is not None:
                    tooltip_children.append(html.Div(f"GS: {float(gs):.0f} kt"))

                if t_sec is not None:
                    tooltip_children.append(html.Div(f"Time: {float(t_sec):.0f} sec"))
                if aob is not None:
                    tooltip_children.append(html.Div(f"AOB: {float(aob):.0f}°"))
                if vs is not None:
                    tooltip_children.append(html.Div(f"VS: {float(vs):.0f} fpm"))

                if track is not None:
                    tooltip_children.append(html.Div(f"Track: {float(track):.0f}°"))
                if hdg is not None:
                    tooltip_children.append(html.Div(f"Hdg: {float(hdg):.0f}°"))
                if drift is not None:
                    tooltip_children.append(html.Div(f"Drift: {float(drift):+.0f}°"))

                hover_markers.append(
                    dl.CircleMarker(
                        center=path[i],
                        radius=3,
                        color="red",
                        fill=True,
                        fillOpacity=0.8,
                        children=dl.Tooltip(tooltip_children),
                    )
                )

        # Add to elements
        elements.append(arc_line)
        elements.extend(hover_markers)

        # Impact marker
        if impact and isinstance(impact, (list, tuple)) and len(impact) == 2:
            elements.append(
                dl.CircleMarker(
                    center=[impact[0], impact[1]],
                    radius=7,
                    color="black",
                    fill=True,
                    fillOpacity=1.0,
                    children=dl.Tooltip("Impact point"),
                )
            )

        # Bounds
        lats = [p[0] for p in path] + [failure_pt.latitude]
        lons = [p[1] for p in path] + [failure_pt.longitude]
        if impact and isinstance(impact, (list, tuple)) and len(impact) == 2:
            lats.append(impact[0])
            lons.append(impact[1])
        bounds = [[min(lats), min(lons)], [max(lats), max(lons)]]

        # Status
        if made_it:
            status = "✅ Impossible turn: succesful"
        else:
            status = f"⚠️ Impossible turn: unsuccessful ({reason})."

        # Result text
        bank_txt = f"{float(bank_deg):.0f}°" if isinstance(bank_deg, (int, float)) else "n/a"

        if isinstance(min_required, (int, float)):
            result = (
                f"Recommended constant bank: {bank_txt}. "
                f"Minimum failure altitude (AGL): {float(min_required):.0f} ft. "
                f"{dist_txt}."
            )
        else:
            result = (
                f"Recommended constant bank: {bank_txt}. "
                f"Minimum failure altitude (AGL): not found in search range. "
                f"{dist_txt}."
            )

        return elements, bounds, status, result

    except Exception as e:
        print(f"❌ EXCEPTION in draw_impossible_turn(): {e}")
        return [], None, f"⚠️ Error: {str(e)}", ""
    
# === Power-Off 180 Rendering Callback ===
@app.callback(
    Output("layer", "children", allow_duplicate=True),
    Output("map", "bounds", allow_duplicate=True),
    Output({"type": "click-status", "m_id": "poweroff180"}, "children", allow_duplicate=True),
    Input("poweroff180-draw-btn", "n_clicks"),
    State({"type": "point-store", "m_id": "poweroff180", "role": "touchdown"}, "data"),
    State("aircraft-select", "value"),
    State("engine-select", "value"),
    State("occupants", "value"),
    State("occupant-weight", "value"),
    State("fuel-load", "value"),
    State("cg-slider", "value"),
    State("env-wind-dir", "value"),
    State("env-wind-speed", "value"),
    State("env-oat", "value"),
    State("env-altimeter", "value"),
    State("poweroff180-altitude", "value"),
    State("poweroff180-pattern", "value"),
    State("poweroff180-flap-setting", "value"),
    State("poweroff180-prop-condition", "value"),
    State("poweroff180-touchdown-heading", "value"),
    State("poweroff180-start-distance-nm", "value"),
    State("selected-airport-id", "data"),
    State("runtime-total-weight-lb", "data"),
    prevent_initial_call=True
)
def draw_poweroff180(
    n_clicks,
    touchdown_data,
    ac_name,
    engine_key,
    occupants,
    occupant_wt,
    fuel_gal,
    cg_pos,
    wind_dir,
    wind_speed,
    oat_f,
    altimeter,
    start_alt_agl,
    pattern_dir,
    flap_setting,
    prop_condition,
    touchdown_heading,
    start_distance_nm,
    selected_airport_id,
    runtime_weight
):
    if not n_clicks or not touchdown_data:
        return [], None, "⚠️ Set a touchdown point first."

    if not ac_name or not engine_key:
        return [], None, "⚠️ Select aircraft and engine first."

    try:
        states = dash.callback_context.states

        def safe_float(key):
            val = states.get(key)
            return float(val) if val not in [None, "", "null"] else None

        start_alt_agl      = safe_float("poweroff180-altitude.value")
        touchdown_heading  = safe_float("poweroff180-touchdown-heading.value")
        wind_dir           = safe_float("env-wind-dir.value")
        wind_speed         = safe_float("env-wind-speed.value")
        oat_f              = safe_float("env-oat.value")
        altimeter          = safe_float("env-altimeter.value")
        start_distance_nm  = safe_float("poweroff180-start-distance-nm.value")

        total_wt = safe_float("runtime-total-weight-lb.data")
        if total_wt is None:
            total_wt = float(runtime_weight) if runtime_weight not in [None, "", "null"] else None

        required = [
            start_alt_agl, touchdown_heading,
            wind_dir, wind_speed, oat_f, altimeter,
            start_distance_nm, total_wt
        ]
        if any(x is None for x in required):
            return [], None, "⚠️ Missing or invalid input values."

        touchdown = GeoPoint(touchdown_data["lat"], touchdown_data["lon"])

        downwind_heading = (touchdown_heading + 180.0) % 360.0

        if pattern_dir == "left":
            offset_bearing = (touchdown_heading - 90.0) % 360.0
        else:
            offset_bearing = (touchdown_heading + 90.0) % 360.0

        start = point_from(touchdown, offset_bearing, start_distance_nm)
        start_heading = downwind_heading

        ac = dict(aircraft_data[ac_name])
        ac["total_weight_lb"] = float(total_wt)

        selected_airport = next((a for a in airport_data if a["id"] == selected_airport_id), None)
        elev_ft = float(selected_airport.get("elevation_ft", 0.0)) if selected_airport else 0.0

        oat_c = (float(oat_f) - 32.0) * 5.0 / 9.0

        start_ias_kias = 80.0

        path, hover_data, impact_marker = simulate_glide_path_to_target(
            start_point=start,
            start_heading=start_heading,
            touchdown_point=touchdown,
            touchdown_heading=touchdown_heading,
            ac=ac,
            engine_option=engine_key,
            weight_lbs=float(total_wt),
            flap_config=flap_setting,
            prop_config=prop_condition,
            oat_c=float(oat_c),
            altimeter_inhg=float(altimeter),
            wind_dir=float(wind_dir),
            wind_speed=float(wind_speed),
            start_ias_kias=float(start_ias_kias),
            altitude_agl=float(start_alt_agl),
            pattern_dir=pattern_dir,
            selected_airport_elev_ft=float(elev_ft),
            max_bank_deg=45,
            timestep_sec=0.5,
        )

        if not path or not hover_data:
            return [], None, "⚠️ No glide path generated. Check inputs."

        FT_PER_NM = 6076.12

        dw_ft = hover_data[0].get("downwind_leg_ft", 0.0) or 0.0
        fn_ft = hover_data[0].get("final_leg_ft", 0.0) or 0.0

        final_out_bearing = (touchdown_heading + 180.0) % 360.0

        dw_end = None
        fn_start = None
        if dw_ft > 0.0:
            dw_nm = dw_ft / FT_PER_NM
            p_dw_end = point_from(start, start_heading, dw_nm)
            dw_end = [p_dw_end.latitude, p_dw_end.longitude]
        if fn_ft > 0.0:
            fn_nm = fn_ft / FT_PER_NM
            p_fn_start = point_from(touchdown, final_out_bearing, fn_nm)
            fn_start = [p_fn_start.latitude, p_fn_start.longitude]

        arc_line = dl.Polyline(positions=path, color="red", weight=3)

        hover_markers = []
        for i, pt in enumerate(hover_data):
            if i % 5 != 0 or i >= len(path):
                continue

            gs    = pt.get("gs")
            track = pt.get("track")
            hdg   = pt.get("heading")
            drift = pt.get("drift")

            tooltip_children = [
                html.Div(f"{pt['alt']:.0f} ft AGL"),
                html.Div(f"TAS: {pt['tas']:.0f} kt"),
            ]

            if gs is not None:
                tooltip_children.append(html.Div(f"GS: {gs:.0f} kt"))

            tooltip_children.extend([
                html.Div(f"Time: {pt['time']:.1f} sec"),
                html.Div(f"AOB: {pt['aob']:.1f}°"),
                html.Div(f"VS: {pt['vs']:.0f} fpm"),
            ])

            if track is not None:
                tooltip_children.append(html.Div(f"Track: {track:.0f}°"))
            if hdg is not None:
                tooltip_children.append(html.Div(f"Hdg: {hdg:.0f}°"))
            if drift is not None:
                tooltip_children.append(html.Div(f"Drift: {drift:+.1f}°"))

            if dw_ft > 0.0 and fn_ft > 0.0:
                dw_nm = dw_ft / FT_PER_NM
                fn_nm = fn_ft / FT_PER_NM
                tooltip_children.append(html.Hr())
                tooltip_children.append(html.Div(f"Downwind: {dw_nm:.2f} NM"))
                tooltip_children.append(html.Div(f"Final: {fn_nm:.2f} NM"))

            hover_markers.append(
                dl.CircleMarker(
                    center=path[i],
                    radius=3,
                    color="red",
                    fill=True,
                    fillOpacity=0.8,
                    children=dl.Tooltip(tooltip_children),
                )
            )

        start_marker = dl.CircleMarker(
            center=[start.latitude, start.longitude],
            radius=7,
            color="green",
            fill=True,
            fillOpacity=1.0,
        )
        touchdown_marker = dl.CircleMarker(
            center=[touchdown.latitude, touchdown.longitude],
            radius=7,
            color="red",
            fill=True,
            fillOpacity=1.0,
        )

        downwind_line = None
        final_line = None

        if dw_end is not None:
            downwind_line = dl.Polyline(
                positions=[[start.latitude, start.longitude], dw_end],
                color="orange",
                weight=2,
            )

        if fn_start is not None:
            final_line = dl.Polyline(
                positions=[fn_start, [touchdown.latitude, touchdown.longitude]],
                color="purple",
                weight=2,
            )

        elements = [start_marker, touchdown_marker] + hover_markers + [arc_line]
        if downwind_line:
            elements.append(downwind_line)
        if final_line:
            elements.append(final_line)

        if impact_marker:
            impact_lat, impact_lon = impact_marker
            impact_mark = dl.CircleMarker(
                center=[impact_lat, impact_lon],
                radius=7,
                color="black",
                fill=True,
                fillOpacity=1.0,
                children=dl.Tooltip("☠️Impact point☠️"),
            )
            elements.append(impact_mark)

            msg = (
                "⚠️ Glide path impacted ground before reaching touchdown at "
                f"({impact_lat:.4f}, {impact_lon:.4f}). Path flown to impact point."
            )
        else:
            msg = "✅ Power-Off 180 path flown successfully."

        lats = [pt[0] for pt in path] + [start.latitude, touchdown.latitude]
        lons = [pt[1] for pt in path] + [start.longitude, touchdown.longitude]

        if dw_end is not None:
            lats.append(dw_end[0])
            lons.append(dw_end[1])
        if fn_start is not None:
            lats.append(fn_start[0])
            lons.append(fn_start[1])
        if impact_marker:
            lats.append(impact_marker[0])
            lons.append(impact_marker[1])

        bounds = [[min(lats), min(lons)], [max(lats), max(lons)]]

        return elements, bounds, msg

    except Exception as e:
        print(f"❌ EXCEPTION in draw_poweroff180(): {e}")
        return [], None, f"⚠️ Error generating path: {str(e)}"

# ============== Engine-Out Glide Rendering Callback ======================#

# === Engine-Out Glide Rendering Callback ===
@app.callback(
    Output("layer", "children", allow_duplicate=True),
    Output("map", "bounds", allow_duplicate=True),
    Output({"type": "click-status", "m_id": "engineout"}, "children", allow_duplicate=True),
    Input("engineout-draw-btn", "n_clicks"),
    State({"type": "point-store", "m_id": "engineout", "role": "start"}, "data"),
    State({"type": "point-store", "m_id": "engineout", "role": "touchdown"}, "data"),
    State("aircraft-select", "value"),
    State("engine-select", "value"),
    State("occupants", "value"),
    State("occupant-weight", "value"),
    State("fuel-load", "value"),
    State("cg-slider", "value"),
    State("env-wind-dir", "value"),
    State("env-wind-speed", "value"),
    State("env-oat", "value"),
    State("env-altimeter", "value"),
    State("engineout-start-heading", "value"),
    State("engineout-altitude", "value"),
    State("engineout-flap-setting", "value"),
    State("engineout-prop-condition", "value"),
    State("engineout-touchdown-heading", "value"),
    State("engineout-pattern-dir", "value"),
    State("engineout-manual-elev", "value"),
    State("selected-airport-id", "data"),
    State("runtime-total-weight-lb", "data"),
    prevent_initial_call=True,
)
def draw_engineout(
    n_clicks,
    start_data,
    touchdown_data,
    ac_name,
    engine_key,
    occupants,
    occupant_wt,
    fuel_gal,
    cg_pos,
    wind_dir,
    wind_speed,
    oat_f,
    altimeter,
    start_heading,
    start_alt_agl,
    flap_setting,
    prop_condition,
    touchdown_heading,
    engineout_pattern_dir,
    manual_td_elev,
    selected_airport_id,
    runtime_weight
):
    if not n_clicks:
        raise PreventUpdate

    if not start_data or not touchdown_data:
        return [], None, "⚠️ Set start and touchdown points first."

    if not ac_name or not engine_key:
        return [], None, "⚠️ Select aircraft and engine first."

    try:
        states = dash.callback_context.states

        def safe_float(key):
            val = states.get(key)
            return float(val) if val not in [None, "", "null"] else None

        start_heading      = safe_float("engineout-start-heading.value")
        start_alt_agl      = safe_float("engineout-altitude.value")
        touchdown_heading  = safe_float("engineout-touchdown-heading.value")
        manual_td_elev     = safe_float("engineout-manual-elev.value")
        wind_dir           = safe_float("env-wind-dir.value")
        wind_speed         = safe_float("env-wind-speed.value")
        oat_f              = safe_float("env-oat.value")
        altimeter          = safe_float("env-altimeter.value")

        total_wt = safe_float("runtime-total-weight-lb.data")
        if total_wt is None:
            total_wt = float(runtime_weight) if runtime_weight not in [None, "", "null"] else None

        required = [
            start_heading, start_alt_agl, touchdown_heading,
            wind_dir, wind_speed, oat_f, altimeter,
            total_wt
        ]
        if any(x is None for x in required):
            return [], None, "⚠️ Missing or invalid input values."

        start = GeoPoint(start_data["lat"], start_data["lon"])
        touchdown = GeoPoint(touchdown_data["lat"], touchdown_data["lon"])

        ac = dict(aircraft_data[ac_name])
        ac["total_weight_lb"] = float(total_wt)

        selected_airport = next((a for a in airport_data if a["id"] == selected_airport_id), None)
        airport_elev_ft = float(selected_airport.get("elevation_ft", 0.0)) if selected_airport else 0.0

        td_store_elev = touchdown_data.get("elevation_ft") if isinstance(touchdown_data, dict) else None

        if manual_td_elev is not None:
            touchdown_elev_ft = float(manual_td_elev)
        elif td_store_elev is not None:
            touchdown_elev_ft = float(td_store_elev)
        else:
            touchdown_elev_ft = float(airport_elev_ft)

        oat_c = (float(oat_f) - 32.0) * 5.0 / 9.0

        path, hover_data, impact_marker = simulate_engineout_glide(
            start_point=start,
            start_heading=float(start_heading),
            touchdown_point=touchdown,
            touchdown_heading=float(touchdown_heading),
            ac=ac,
            engine_option=engine_key,
            weight_lbs=float(total_wt),
            flap_config=flap_setting,
            prop_config=prop_condition,
            oat_c=float(oat_c),
            altimeter_inhg=float(altimeter),
            wind_dir=float(wind_dir),
            wind_speed=float(wind_speed),
            start_ias_kias=None,
            altitude_agl=float(start_alt_agl),
            touchdown_elev_ft=float(touchdown_elev_ft),
            selected_airport_elev_ft=float(airport_elev_ft),
            max_bank_deg=45,
            timestep_sec=0.5,
            pattern_dir=engineout_pattern_dir,
        )

        if not path or not hover_data:
            return [], None, "⚠️ No glide path generated. Check inputs."

        # ---------- Core visuals: full glide track + hover markers ----------
        arc_line = dl.Polyline(positions=path, color="red", weight=3)

        hover_markers = []
        for i, pt in enumerate(hover_data):
            # Thin markers
            if i % 5 != 0 or i >= len(path):
                continue

            tooltip_children = [
                html.Div(f"{pt['alt']:.0f} ft AGL"),
                html.Div(f"TAS: {pt['tas']:.0f} kt"),
                html.Div(f"GS: {pt.get('gs', pt['tas']):.0f} kt"),
                html.Div(f"Time: {pt['time']:.1f} sec"),
                html.Div(f"AOB: {pt['aob']:.1f}°"),
                html.Div(f"VS: {pt['vs']:.0f} fpm"),
                html.Div(f"Track: {pt.get('track', 0):.0f}°"),
                html.Div(f"Heading: {pt.get('heading', 0):.0f}°"),
                html.Div(f"Drift: {pt.get('drift', 0):+.1f}°"),
            ]

            hover_markers.append(
                dl.CircleMarker(
                    center=path[i],
                    radius=3,
                    color="red",
                    fill=True,
                    fillOpacity=0.8,
                    children=dl.Tooltip(tooltip_children),
                )
            )

        # Start / touchdown markers
        start_marker = dl.CircleMarker(
            center=[start.latitude, start.longitude],
            radius=7,
            color="green",
            fill=True,
            fillOpacity=1.0,
        )
        touchdown_marker = dl.CircleMarker(
            center=[touchdown.latitude, touchdown.longitude],
            radius=7,
            color="red",
            fill=True,
            fillOpacity=1.0,
        )

        elements = [start_marker, touchdown_marker] + hover_markers + [arc_line]

        # Impact vs success messaging / marker
        if impact_marker:
            impact_lat, impact_lon = impact_marker
            impact_mark = dl.CircleMarker(
                center=[impact_lat, impact_lon],
                radius=7,
                color="black",
                fill=True,
                fillOpacity=1.0,
                children=dl.Tooltip("☠️Impact point☠️"),
            )
            elements.append(impact_mark)
            msg = (
                "⚠️ Glide path impacted ground before reaching touchdown at "
                f"({impact_lat:.4f}, {impact_lon:.4f}). Path flown to impact point."
            )
        else:
            msg = f"✅ Engine-out glide path flown successfully ({engineout_pattern_dir} pattern)."

        # ---------- Bounds ----------
        lats = [pt[0] for pt in path] + [start.latitude, touchdown.latitude]
        lons = [pt[1] for pt in path] + [start.longitude, touchdown.longitude]
        if impact_marker:
            lats.append(impact_marker[0])
            lons.append(impact_marker[1])

        bounds = [[min(lats), min(lons)], [max(lats), max(lons)]]

        return elements, bounds, msg

    except Exception as e:
        print(f"❌ EXCEPTION in draw_engineout(): {e}")
        return [], None, f"⚠️ Error generating path: {str(e)}"


# === Steep Turn Rendering Callback ===
@app.callback(
    Output("layer", "children", allow_duplicate=True),
    Input("steepturn-draw-btn", "n_clicks"),
    State({"type": "point-store", "m_id": "steep_turn", "role": "start"}, "data"),
    State("steepturn-bank-angle", "value"),
    State("steepturn-sequence", "value"),
    State("steepturn-entry-heading", "value"),
    State("steepturn-altitude", "value"),
    State("steepturn-ias", "value"),
    State("total-weight-display", "value"),
    State("env-oat", "value"),
    State("env-altimeter", "value"),
    State("env-wind-dir", "value"),
    State("env-wind-speed", "value"),
    State("aircraft-select", "value"),
    State("engine-select", "value"),
    State("runtime-total-weight-lb", "data"),
    prevent_initial_call=True
)
def draw_steep_turn(
    n_clicks,
    start,
    bank_angle,
    sequence,
    entry_heading,
    entry_alt_ft,
    entry_ias,
    weight_str,
    oat_f,
    altimeter_inhg,
    wind_dir,
    wind_speed,
    aircraft_name,
    engine_name,
    runtime_weight
):
    if not n_clicks or not start or not aircraft_name or not engine_name:
        raise PreventUpdate

    ac = aircraft_data[aircraft_name]

    # Use Va as default entry IAS if user left blank
    if int(ac.get("engine_count", 1)) > 1:
        va = float((ac.get("multi_engine_limits", {}) or {}).get("va", 100))
    else:
        va = float((ac.get("single_engine_limits", {}) or {}).get("va", 100))
    entry_ias = float(entry_ias) if entry_ias not in [None, "", "null"] else float(va)

    # Runtime weight should be authoritative. Fallback to parsing the display box.
    weight_lbs = None
    try:
        if runtime_weight not in [None, "", "null"]:
            weight_lbs = float(runtime_weight)
    except Exception:
        weight_lbs = None

    if weight_lbs is None:
        try:
            # total-weight-display is already just a number string in your UI ("1523"), so parse directly.
            weight_lbs = float(str(weight_str).replace(",", "").strip())
        except Exception:
            weight_lbs = float(ac.get("empty_weight", 1200.0)) + 180.0

    altitude_ft = float(entry_alt_ft) if entry_alt_ft not in [None, "", "null"] else float(ac.get("default_altitude", 1000.0))

    oat_c = None
    try:
        oat_c = (float(oat_f) - 32.0) * 5.0 / 9.0
    except Exception:
        oat_c = (52.0 - 32.0) * 5.0 / 9.0

    # Pass runtime weight through the aircraft dict so any helper in utility.py can use it
    ac_rt = dict(ac)
    ac_rt["total_weight_lb"] = float(weight_lbs)

    path, hover = simulate_steep_turn(
        entry_point={"lat": start["lat"], "lon": start["lon"]},
        entry_heading_deg=float(entry_heading),
        altitude_ft=float(altitude_ft),
        bank_angle_deg=float(bank_angle),
        turn_sequence=sequence,
        tas_knots=float(entry_ias),
        wind_dir_deg=float(wind_dir) if wind_dir not in [None, "", "null"] else 0.0,
        wind_speed_kt=float(wind_speed) if wind_speed not in [None, "", "null"] else 0.0,
        # If you later wire weight into simulate_steep_turn, you already have it here:
        # ac=ac_rt,
        # weight_lbs=float(weight_lbs),
        # oat_c=float(oat_c),
        # altimeter_inhg=float(altimeter_inhg),
    )

    return [render_hover_polyline(path, hover, color="blue", weight=3)]

from dash import no_update

from dash import callback, Input, Output, State, ctx
from dash.exceptions import PreventUpdate

@callback(
    Output({"type": "point-store", "m_id": ALL, "role": ALL}, "data", allow_duplicate=True),
    Output("active-click-target", "data", allow_duplicate=True),
    Output("layer", "children", allow_duplicate=True),
    Output("map", "bounds", allow_duplicate=True),
    Input("reset-all", "n_clicks"),
    Input("reset-clicks", "n_clicks"),
    State({"type": "point-store", "m_id": ALL, "role": ALL}, "id"),
    prevent_initial_call=True
)
def handle_resets(n_reset_all, n_reset_clicks, store_ids):
    trigger = ctx.triggered_id
    if trigger not in ("reset-all", "reset-clicks"):
        raise PreventUpdate

    # Clear every point-store, regardless of maneuver
    cleared_points = [None] * len(store_ids)

    # Clear click target so map clicks do not overwrite anything until re-armed
    cleared_target = None

    # Clear the drawing layer and bounds
    cleared_layer = []
    cleared_bounds = None


    return cleared_points, cleared_target, cleared_layer, cleared_bounds

@app.callback(
    Output("disclaimer-modal", "is_open"),
    Output("terms-policy-modal", "is_open"),
    Input("open-disclaimer", "n_clicks"),
    Input("close-disclaimer", "n_clicks"),
    Input("open-terms-policy", "n_clicks"),
    Input("close-terms-policy", "n_clicks"),
    State("disclaimer-modal", "is_open"),
    State("terms-policy-modal", "is_open"),
    prevent_initial_call=True,
)
def toggle_legal_modals(open_disc, close_disc, open_terms, close_terms, disc_open, terms_open):
    trigger = ctx.triggered_id

    if trigger == "open-disclaimer":
        return True, False
    if trigger == "close-disclaimer":
        return False, terms_open
    if trigger == "open-terms-policy":
        return disc_open, True
    if trigger == "close-terms-policy":
        return disc_open, False

    return no_update, no_update

if __name__ == "__main__":
    app.run(debug=True)
