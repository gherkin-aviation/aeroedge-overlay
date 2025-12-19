import dash
from dash import dcc, html
import json
import os
from dash.exceptions import PreventUpdate

def load_aircraft_data_from_folder():
    folder_path = os.path.join(os.path.dirname(__file__), "aircraft_data")
    aircraft_data = {}
    for filename in os.listdir(folder_path):
        if filename.endswith(".json"):
            filepath = os.path.join(folder_path, filename)
            with open(filepath, "r") as f:
                try:
                    data = json.load(f)
                    name = os.path.splitext(filename)[0].replace("_", " ")
                    aircraft_data[name] = data
                except Exception as e:
                    print(f"[ERROR] Failed to load {filename}: {e}")
    return aircraft_data

# ✅ Load it once for this page
aircraft_data = load_aircraft_data_from_folder()

# --- Full Edit Aircraft Page Layout ---
def edit_aircraft_layout():
    return html.Div([
        dcc.Store(id="stored-flap-configs", data=[]),
        dcc.Store(id="stored-g-limits", data=[]),
        dcc.Store(id="stored-stall-speeds", data=[]),
        dcc.Store(id="stored-single-engine-limits", data=[]),
        dcc.Store(id="stored-engine-options", data=[]),
        dcc.Store(id="stored-other-limits", data={}),
        dcc.Store(id="stored-oei-performance", data=[]),

        html.Div([
            html.Div([
                html.Img(src="/assets/logo.png", className="banner-logo")
            ], className="banner-inner")
        ], className="banner-header"),
        
        html.Div([
            html.Button("⬅️ Back to EM Diagram", id="back-button", n_clicks=0, className="green-button")
        ], style={"marginBottom": "20px"}),
        
        html.Div([
            html.Div([
                html.Label("Search Aircraft", className="input-label"),
                dcc.Dropdown(
                    id="aircraft-search",
                    options=[{"label": name, "value": name} for name in sorted(aircraft_data.keys())],
                    placeholder="Start typing...",
                    searchable=True,
                    className="dropdown"
                )
            ]),
            html.Div([
                html.Button("New Aircraft", id="new-aircraft-button", n_clicks=0, className="green-button", style={"marginRight": "10px"}),
                html.Button("💾 Save Aircraft", id="save-aircraft-button", n_clicks=0, className="green-button"),
            ], style={"display": "flex", "alignItems": "center", "gap": "10px"}),
            html.Div(id="search-result", style={"marginTop": "10px", "color": "green"})
        ], style={"marginBottom": "20px"}),

        html.Div([
            html.Label("Apply Default Performance Values:", className="input-label"),
            html.Div([
                html.Button("Single Engine", id="default-single", n_clicks=0, className="green-button", style={"marginRight": "10px"}),
                html.Button("Multi Engine", id="default-multi", n_clicks=0, className="green-button", style={"marginRight": "10px"}),
                html.Button("Aerobatic", id="default-aerobatic", n_clicks=0, className="green-button", style={"marginRight": "10px"}),
                html.Button("Trainer", id="default-trainer", n_clicks=0, className="green-button", style={"marginRight": "10px"}),
                html.Button("Military Trainer", id="default-mil-trainer", n_clicks=0, className="green-button", style={"marginRight": "10px"}),
                html.Button("Experimental", id="default-experimental", n_clicks=0, className="green-button")
            ], style={"display": "flex", "flexWrap": "wrap", "gap": "10px"})
        ], className="mb-4"),

    html.Div([
        html.Label("Units:", className="input-label"),
        dcc.RadioItems(
            id="units-toggle",
            options=[
                {"label": "KIAS", "value": "KIAS"},
                {"label": "MPH", "value": "MPH"}
            ],
            value="KIAS",
            labelStyle={"display": "inline-block", "marginRight": "15px"},
            className="dash-radio-items"
        )
    ], className="mb-3"),

    html.Div([
        html.Label("Aircraft Name", className="input-label"),
        dcc.Input(id="aircraft-name", type="text", className="dropdown", placeholder="e.g. DA40-180")
    ], className="mb-3"),

    html.Div([
        html.Label("Aircraft Type", className="input-label"),
        dcc.Dropdown(
            id="aircraft-type",
            options=[
                {"label": "Single Engine", "value": "single_engine"},
                {"label": "Multi Engine", "value": "multi_engine"}
            ],
            placeholder="Select type",
            className="dropdown"
        )
    ], className="mb-3"),

    html.Div([
        html.Label("Engine Count", className="input-label"),
        dcc.Input(id="engine-count", type="number", min=1, step=1, className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Wing Area (ft²)", className="input-label"),
        dcc.Input(id="wing-area", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Aspect Ratio", className="input-label"),
        dcc.Input(id="aspect-ratio", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("CD₀ (Parasite Drag Coefficient)", className="input-label"),
        dcc.Input(id="cd0", type="number", step=0.001, className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Oswald Efficiency Factor (e)", className="input-label"),
        dcc.Input(id="oswald-efficiency", type="number", step=0.01, className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Empty Weight (lbs)", className="input-label"),
        dcc.Input(id="empty-weight", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Max Gross Weight (lbs)", className="input-label"),
        dcc.Input(id="max-weight", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Number of Seats", className="input-label"),
        dcc.Input(id="seats", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("🛫 CG Range (inches)", className="input-label"),
        html.Div([
            html.Label("FWD", className="inline-label"),
            dcc.Input(id="cg-fwd", type="number", className="input-small", style={"marginRight": "20px"}),
            html.Label("AFT", className="inline-label"),
            dcc.Input(id="cg-aft", type="number", className="input-small")
        ], style={"display": "flex", "alignItems": "center"})
    ], className="mb-3"),
    html.Div([
        html.Label("Fuel Capacity (gal)", className="input-label"),
        dcc.Input(id="fuel-capacity-gal", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Fuel Weight per Gallon (lbs)", className="input-label"),
        dcc.Input(id="fuel-weight-per-gal", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Vne (Never Exceed Speed)", className="input-label"),
        dcc.Input(id="vne", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Vno (Max Structural Cruising Speed)", className="input-label"),
        dcc.Input(id="vno", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Best Glide Speed", className="input-label"),
        dcc.Input(id="best-glide", type="number", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Best Glide Ratio", className="input-label"),
        dcc.Input(id="best-glide-ratio", type="number", step=0.1, className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("Service Ceiling (ft)", className="input-label"),
        dcc.Input(id="max-altitude", type="number", placeholder="e.g. 18000", className="input-small")
    ], className="mb-3"),

    html.Div([
        html.Label("🛫 Airspeed Arcs", className="input-label"),
        
        html.Div([
            html.Label("White Arc", className="inline-label"),
            dcc.Input(id="arc-white-bottom", type="number", placeholder="Bottom", className="input-small", style={"marginRight": "10px"}),
            dcc.Input(id="arc-white-top", type="number", placeholder="Top", className="input-small")
        ], className="mb-2"),

        html.Div([
            html.Label("Green Arc", className="inline-label"),
            dcc.Input(id="arc-green-bottom", type="number", placeholder="Bottom", className="input-small", style={"marginRight": "10px"}),
            dcc.Input(id="arc-green-top", type="number", placeholder="Top", className="input-small")
        ], className="mb-2"),

        html.Div([
            html.Label("Yellow Arc", className="inline-label"),
            dcc.Input(id="arc-yellow-bottom", type="number", placeholder="Bottom", className="input-small", style={"marginRight": "10px"}),
            dcc.Input(id="arc-yellow-top", type="number", placeholder="Top", className="input-small")
        ], className="mb-2"),

        html.Div([
            html.Label("Red Line", className="inline-label"),
            dcc.Input(id="arc-red", type="number", placeholder="Red", className="input-small")
        ])
    ], className="mb-4"),

    html.Div([
        html.Label("🛫 Prop Thrust Decay", className="input-label"),
        html.Div([
            html.Label("T_static Factor", className="inline-label"),
            dcc.Input(id="prop-static-factor", type="number", step=0.1, className="input-small", style={"marginRight": "15px"}),
            html.Label("V_max", className="inline-label"),
            dcc.Input(id="prop-vmax-kts", type="number", className="input-small")
        ], style={"display": "flex", "alignItems": "center", "flexWrap": "wrap"})
    ], className="mb-4"),

    html.Div([
        html.Label("🛫 Flap Configurations", className="input-label mb-2"),
        html.Div(id="flap-configs-container", children=[
            html.Div([
                html.Label("Clean / Up", className="inline-label", style={"width": "100px"}),
                dcc.Input(id={"type": "clmax-input", "config": "clean"}, type="number", placeholder="CLmax", step=0.01, className="input-small")
            ], className="mb-2"),

            html.Div([
                html.Label("Takeoff / 10-20°", className="inline-label", style={"width": "100px"}),
                dcc.Input(id={"type": "vfe-input", "config": "takeoff"}, type="number", placeholder="Vfe", className="input-small", style={"marginRight": "10px"}),
                dcc.Input(id={"type": "clmax-input", "config": "takeoff"}, type="number", placeholder="CLmax", step=0.01, className="input-small")
            ], className="mb-2"),

            html.Div([
                html.Label("Landing / 30-40°", className="inline-label", style={"width": "100px"}),
                dcc.Input(id={"type": "vfe-input", "config": "landing"}, type="number", placeholder="Vfe", className="input-small", style={"marginRight": "10px"}),
                dcc.Input(id={"type": "clmax-input", "config": "landing"}, type="number", placeholder="CLmax", step=0.01, className="input-small")
            ])
        ])
    ], className="mb-4"),

    html.Div([
        html.H3("🛫 G Limits", className="input-label"),
        html.Div(id="g-limits-container"),
        html.Button("➕ Add G Limit", id="add-g-limit", n_clicks=0, className="green-button mt-2")
    ], className="mb-4"),

    html.Div([
        html.H3("🛫 Stall Speeds", className="input-label"),
        html.Div(id="stall-speeds-container"),
        html.Button("➕ Add Stall Speed", id="add-stall-speed", n_clicks=0, className="green-button mt-2")
    ], className="mb-4"),

    html.Div([
        html.H3("🛫 Single Engine Limits", className="input-label"),
        html.Div(id="single-engine-limits-container"),
        html.Button("➕ Add Single Engine Limit", id="add-single-engine-limit", n_clicks=0, className="green-button mt-2")
    ], className="mb-4"),

    html.Div([
        html.H3("🛫 OEI Performance", className="input-label"),
        html.Div(id="oei-performance-container"),
        html.Button("➕ Add OEI Performance", id="add-oei-performance", n_clicks=0, className="green-button mt-2")
    ], className="mb-4"),

    html.Div([
        html.H3("🛫 Engine Options / HP / Power Curves", className="input-label"),
        html.Div(id="engine-options-container"),
        html.Button("➕ Add Engine Option", id="add-engine-option", n_clicks=0, className="green-button mt-2")
    ], className="mb-4"),


    html.Button("💾 Save Aircraft", id="save-aircraft-button", n_clicks=0, className="green-button mt-4"),
    html.Div(id="save-status", className="mt-2", style={"marginTop": "20px"}),

])
