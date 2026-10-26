"""
campus_data.py  --  The campus map used by the Smart Campus Navigator
=====================================================================
Coordinates are in METRES (x to the right, y downwards).  They are used both
for drawing and for the A* heuristic (straight-line distance).

Every walkway has a cost = straight-line length * surface factor.
The factor models how unpleasant the path is (stairs, crowds, rough footpath).
All factors are >= 1, which guarantees that the Euclidean heuristic is
admissible.  To use your own campus, just edit the two tables below.
"""

from astar_engine import Graph

# kind -> (cost factor, colour used on the map, description)
EDGE_TYPES = {
    "road":     (1.05, "#6b6b6b", "Main road"),
    "covered":  (1.00, "#8a6d3b", "Covered corridor"),
    "footpath": (1.10, "#9a9a9a", "Footpath"),
    "crowded":  (1.50, "#c77d00", "Crowded lane (canteen)"),
    "stairs":   (1.35, "#7a5fa8", "Stairs / ramp"),
}

# key, display name, x, y, kind
NODES = [
    ("MG",  "Main Gate",            60, 320, "gate"),
    ("J1",  "Junction 1",          180, 320, "junction"),
    ("ADM", "Admin Block",         180, 220, "building"),
    ("PK",  "Parking",             180, 440, "parking"),
    ("J2",  "Junction 2",          330, 320, "junction"),
    ("LIB", "Central Library",     330, 200, "building"),
    ("ACA", "Academic Block A",    260,  80, "building"),
    ("ACB", "Academic Block B",    330, 440, "building"),
    ("J3",  "Junction 3",          480, 320, "junction"),
    ("CS",  "CSE & IT Block",      480, 100, "building"),
    ("AUD", "Auditorium",          480, 210, "building"),
    ("CAF", "Cafeteria",           640, 210, "building"),
    ("J4",  "Junction 4",          640, 320, "junction"),
    ("J5",  "Junction 5",          640,  95, "junction"),
    ("MED", "Medical Centre",      640, 430, "medical"),
    ("H1",  "Hostel Block 1",      480, 470, "building"),
    ("H2",  "Hostel Block 2",      330, 560, "building"),
    ("SPT", "Sports Complex",      790, 450, "building"),
    ("AP1", "Assembly Pt 1 (Lawn)",        780,  90, "assembly"),
    ("AP2", "Assembly Pt 2 (Sports Ground)", 790, 320, "assembly"),
    ("AP3", "Assembly Pt 3 (Parking)",      60, 460, "assembly"),
]

# a, b, kind
EDGES = [
    ("MG",  "J1",  "road"),
    ("J1",  "ADM", "footpath"),
    ("J1",  "PK",  "road"),
    ("J1",  "J2",  "road"),
    ("J2",  "LIB", "footpath"),
    ("J2",  "ACB", "road"),
    ("J2",  "J3",  "road"),
    ("LIB", "ADM", "covered"),
    ("LIB", "ACA", "footpath"),
    ("LIB", "AUD", "covered"),
    ("ACA", "CS",  "covered"),
    ("J3",  "AUD", "stairs"),
    ("AUD", "CS",  "covered"),
    ("AUD", "CAF", "crowded"),
    ("J3",  "J4",  "road"),
    ("J3",  "H1",  "road"),
    ("J4",  "CAF", "crowded"),
    ("J4",  "MED", "road"),
    ("J4",  "AP2", "road"),
    ("CS",  "J5",  "footpath"),
    ("J5",  "CAF", "footpath"),
    ("J5",  "AP1", "road"),
    ("ACB", "H1",  "footpath"),
    ("ACB", "H2",  "footpath"),
    ("ACB", "PK",  "road"),
    ("H2",  "H1",  "footpath"),
    ("H1",  "MED", "footpath"),
    ("MED", "SPT", "road"),
    ("SPT", "AP2", "footpath"),
    ("PK",  "AP3", "footpath"),
]

ASSEMBLY_POINTS = ["AP1", "AP2", "AP3"]
MEDICAL_CENTRE = "MED"
WALKING_SPEED_M_PER_MIN = 84.0      # 1.4 m/s


def build_campus() -> Graph:
    g = Graph()
    for key, name, x, y, kind in NODES:
        g.add_node(key, name, x, y, kind)
    for a, b, kind in EDGES:
        g.add_edge(a, b, kind, EDGE_TYPES[kind][0])
    return g
