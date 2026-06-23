"""Unit tests for the ground path planner (pure logic, no network)."""
import math
import os
import tempfile

from planner import route_planner as rp


def test_enu_latlon_roundtrip():
    olat, olon = 43.47, -80.54
    for east, north in [(0, 0), (1500, -2300), (-800, 600)]:
        lat, lon = rp.enu_to_latlon(olat, olon, east, north)
        e2, n2 = rp.latlon_to_enu(olat, olon, lat, lon)
        assert abs(e2 - east) < 1.0 and abs(n2 - north) < 1.0


def test_plan_route_picks_strongest_by_default():
    prior = {"candidates": [[1000, 0, 3.0, 0.8], [2000, 0, 2.0, 0.5], [500, 500, 1.5, 0.4]]}
    route, goal = rp.plan_route(prior, plan_alt=1500)
    assert route, "expected a route"
    last = route[-1]
    assert abs(last["enu_x"] - 1000) < 1 and abs(last["enu_y"]) < 1   # the strongest


def test_plan_route_respects_reachability():
    prior = {"candidates": [[5000, 0, 3.0, 0.9]]}      # 5 km away
    route, _ = rp.plan_route(prior, plan_alt=100)       # reach ~ (100-80)*22 = 440 m
    assert route == []


def test_plan_route_empty_prior():
    route, _ = rp.plan_route({"candidates": []})
    assert route == []


def test_write_qgc_is_soaring_mission():
    route_ll = [{"seq": 1, "enu_x": 0, "enu_y": 0, "w_star": 2.0, "prob": 0.8,
                 "lat": 40.0, "lon": -80.0}]
    with tempfile.TemporaryDirectory() as d:
        p = rp.write_qgc(route_ll, (40.0, -80.0), os.path.join(d, "r.waypoints"),
                         takeoff_alt=120, ceiling_alt=300)
        lines = [l for l in open(p).read().splitlines() if l]
    assert lines[0] == "QGC WPL 110"
    rows = [l.split("\t") for l in lines[1:]]
    assert len(rows) == 4                       # home, takeoff, 1 hotspot, RTL
    assert rows[1][3] == "22"                    # NAV_TAKEOFF
    assert rows[2][3] == "31"                    # NAV_LOITER_TO_ALT (soaring)
    assert rows[3][3] == "20"                    # RETURN_TO_LAUNCH


def test_write_sitl_thermals_relative_to_first():
    route_ll = [{"enu_x": 100, "enu_y": 200, "w_star": 2.0},
                {"enu_x": 100, "enu_y": 5200, "w_star": 2.5}]
    with tempfile.TemporaryDirectory() as d:
        p = rp.write_sitl_thermals(route_ll, os.path.join(d, "t.txt"), radius=300,
                                   ref_enu=(route_ll[0]["enu_x"], route_ll[0]["enu_y"]))
        lines = open(p).read().splitlines()
    # first thermal at the SITL home (0,0); second 5000 m north (x_north col first)
    assert lines[0].split()[:2] == ["0.0", "0.0"]
    assert abs(float(lines[1].split()[0]) - 5000) < 1
