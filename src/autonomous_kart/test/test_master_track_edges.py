"""The surveyed track edges served by master_api's /track_edges.

The KML holds the asphalt boundaries in WGS84; the frontend draws in the /odom
frame, so the loader projects them about lat0/lon0 exactly as gps_node does.
"""
import math

import pytest

rclpy_mod = pytest.importorskip("rclpy")

from autonomous_kart.nodes.master import master_api  # noqa: E402

LAT0, LON0 = 40.4380475, -86.9442826
R_EARTH = 6_371_000

KML = """<?xml version="1.0" encoding="UTF-8"?>
<kml><Document>
<Placemark><name>Inside 2</name><LineString><coordinates>
{lon0},{lat0},0 {lon1},{lat0},0
</coordinates></LineString></Placemark>
<Placemark><name>Outside 2</name><LineString><coordinates>
{lon0},{lat1},0
</coordinates></LineString></Placemark>
</Document></kml>
"""


@pytest.fixture
def kml_path(tmp_path):
    master_api._TRACK_EDGE_CACHE.clear()
    p = tmp_path / "track.kml"
    # One degree-ish offsets big enough to survive float noise, small enough
    # that the spherical approximation is the thing under test.
    p.write_text(KML.format(lat0=LAT0, lon0=LON0, lat1=LAT0 + 0.001,
                            lon1=LON0 + 0.001))
    yield str(p)
    master_api._TRACK_EDGE_CACHE.clear()


def test_loader_splits_inner_and_outer(kml_path):
    edges = master_api._load_track_edges(kml_path, LAT0, LON0)
    assert len(edges["inner"]) == 2
    assert len(edges["outer"]) == 1


def test_loader_projects_about_the_origin(kml_path):
    edges = master_api._load_track_edges(kml_path, LAT0, LON0)
    assert edges["inner"][0] == pytest.approx([0.0, 0.0], abs=1e-9)

    x, y = edges["inner"][1]
    assert x == pytest.approx(
        R_EARTH * math.radians(0.001) * math.cos(math.radians(LAT0)))
    assert y == pytest.approx(0.0, abs=1e-9)

    x, y = edges["outer"][0]
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(R_EARTH * math.radians(0.001))


def test_loader_returns_empty_when_file_missing(tmp_path):
    master_api._TRACK_EDGE_CACHE.clear()
    edges = master_api._load_track_edges(str(tmp_path / "nope.kml"), LAT0, LON0)
    assert edges == {"inner": [], "outer": []}
    master_api._TRACK_EDGE_CACHE.clear()


class _StubNode:
    def __init__(self, path):
        self.track_path = path
        self.lat0 = LAT0
        self.lon0 = LON0


def test_route_serves_both_edges(kml_path, monkeypatch):
    monkeypatch.setattr(master_api, "master_node", _StubNode(kml_path))
    resp = master_api.app.test_client().get("/track_edges")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["path"] == kml_path
    assert len(body["inner"]) == 2
    assert len(body["outer"]) == 1


def test_route_404s_when_track_missing(tmp_path, monkeypatch):
    master_api._TRACK_EDGE_CACHE.clear()
    monkeypatch.setattr(master_api, "master_node",
                        _StubNode(str(tmp_path / "nope.kml")))
    resp = master_api.app.test_client().get("/track_edges")
    assert resp.status_code == 404
    assert resp.get_json()["inner"] == []
    master_api._TRACK_EDGE_CACHE.clear()
