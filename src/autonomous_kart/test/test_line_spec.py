"""
Racing-line shape discovery + speed scaling.

A shape is one CSV in the racing-line directory, named by its file stem, so
dropping a file in is all it takes to make a new shape loadable. The speed
knobs then reshape the vx column: multiply by v_mult first, clamp into
[v_min, v_max] second.
"""
import pytest

from autonomous_kart import line_spec


# Shape naming

@pytest.mark.parametrize("filename,expected", [
    ("line1.csv", "line1"),
    ("line2.csv", "line2"),
    ("center.csv", "center"),
    ("line8_gl.csv", "line8_gl"),
    ("/abs/path/track.csv", "track"),
])
def test_shape_name_is_the_file_stem(filename, expected):
    assert line_spec.shape_name(filename) == expected


# Discovery

def _write_line(d, name, vx, n=4):
    rows = [
        f"{i}.0,{i}.0,0.0,0.0,0.0,{vx},0.0"
        for i in range(n)
    ]
    (d / name).write_text("\n".join(rows) + "\n")


def test_discover_lists_one_shape_per_csv(tmp_path):
    _write_line(tmp_path, "line1.csv", 6.0)
    _write_line(tmp_path, "line2.csv", 6.0)
    _write_line(tmp_path, "center.csv", 6.0)

    shapes = [s["shape"] for s in line_spec.discover_shapes(str(tmp_path))]

    assert shapes == ["center", "line1", "line2"]


def test_discover_reports_each_shapes_file_and_vx_range(tmp_path):
    _write_line(tmp_path, "line1.csv", 10.0)

    (shape,) = line_spec.discover_shapes(str(tmp_path))

    assert shape["file"].endswith("line1.csv")
    assert shape["vx_min"] == pytest.approx(10.0)
    assert shape["vx_max"] == pytest.approx(10.0)
    assert shape["points"] == 4


def test_discover_sees_a_file_dropped_in_after_the_first_scan(tmp_path):
    _write_line(tmp_path, "line1.csv", 6.0)
    assert len(line_spec.discover_shapes(str(tmp_path))) == 1

    _write_line(tmp_path, "oval.csv", 9.0)

    assert [s["shape"] for s in line_spec.discover_shapes(str(tmp_path))] == [
        "line1", "oval",
    ]


def test_discover_on_missing_dir_is_empty(tmp_path):
    assert line_spec.discover_shapes(str(tmp_path / "nope")) == []


def test_discover_skips_a_csv_with_no_vx_column(tmp_path):
    _write_line(tmp_path, "line1.csv", 6.0)
    (tmp_path / "junk.csv").write_text("0.0,1.0,2.0\n")

    assert [s["shape"] for s in line_spec.discover_shapes(str(tmp_path))] == ["line1"]


def test_resolve_shape_returns_its_file(tmp_path):
    _write_line(tmp_path, "line1.csv", 6.0)
    _write_line(tmp_path, "line2.csv", 6.0)

    assert line_spec.resolve_shape(str(tmp_path), "line2").endswith("line2.csv")


def test_resolve_unknown_shape_returns_none(tmp_path):
    _write_line(tmp_path, "line1.csv", 6.0)

    assert line_spec.resolve_shape(str(tmp_path), "line") is None


# Speed scaling

_ROWS = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0),
    (1.0, 1.0, 0.0, 0.0, 0.0, 10.0, 0.0),
]


def test_scale_multiplies_the_vx_column():
    out = line_spec.scale_line_speed(_ROWS, v_min=0.0, v_max=10.0, v_mult=0.6)

    assert [r[5] for r in out] == pytest.approx([6.0, 6.0])


def test_scale_clamps_to_v_max():
    out = line_spec.scale_line_speed(_ROWS, v_min=0.0, v_max=6.0, v_mult=1.0)

    assert [r[5] for r in out] == pytest.approx([6.0, 6.0])


def test_scale_clamps_to_v_min():
    out = line_spec.scale_line_speed(_ROWS, v_min=3.0, v_max=10.0, v_mult=0.1)

    assert [r[5] for r in out] == pytest.approx([3.0, 3.0])


def test_scale_multiplies_before_clamping():
    # Multiply-then-clamp gives 6.0; clamp-then-multiply would give 12.0.
    out = line_spec.scale_line_speed(_ROWS, v_min=0.0, v_max=6.0, v_mult=2.0)

    assert [r[5] for r in out] == pytest.approx([6.0, 6.0])


def test_scale_leaves_every_other_column_alone():
    rows = [(1.0, 2.0, 3.0, 4.0, 5.0, 10.0, 7.0)]

    (out,) = line_spec.scale_line_speed(rows, v_min=0.0, v_max=5.0, v_mult=1.0)

    assert out[:5] == (1.0, 2.0, 3.0, 4.0, 5.0)
    assert out[6] == 7.0


def test_scale_leaves_short_rows_alone():
    rows = [(0.0, 1.0, 2.0)]

    assert line_spec.scale_line_speed(rows, v_min=0.0, v_max=5.0, v_mult=1.0) == rows


# Spec normalization

def test_normalize_fills_defaults_from_the_mpc_target_speed():
    spec, err = line_spec.normalize_spec(
        {"shape": "newline"}, default_v_max=10.0, v_max_limit=12.0
    )

    assert err is None
    assert spec == {"shape": "newline", "v_min": 0.0, "v_max": 10.0, "v_mult": 1.0}


def test_normalize_keeps_an_explicit_v_max():
    spec, err = line_spec.normalize_spec(
        {"shape": "line", "v_max": 7.5}, default_v_max=10.0, v_max_limit=12.0
    )

    assert err is None
    assert spec["v_max"] == pytest.approx(7.5)


def test_normalize_rejects_a_missing_shape():
    _, err = line_spec.normalize_spec({}, default_v_max=10.0, v_max_limit=12.0)

    assert "shape" in err


def test_normalize_rejects_v_min_above_v_max():
    _, err = line_spec.normalize_spec(
        {"shape": "line", "v_min": 8.0, "v_max": 6.0},
        default_v_max=10.0, v_max_limit=12.0,
    )

    assert "v_min" in err


def test_normalize_rejects_v_max_above_the_kart_limit():
    _, err = line_spec.normalize_spec(
        {"shape": "line", "v_max": 20.0}, default_v_max=10.0, v_max_limit=12.0
    )

    assert "12" in err


def test_normalize_rejects_a_non_positive_multiplier():
    _, err = line_spec.normalize_spec(
        {"shape": "line", "v_mult": 0.0}, default_v_max=10.0, v_max_limit=12.0
    )

    assert "v_mult" in err


def test_normalize_rejects_a_non_numeric_value():
    _, err = line_spec.normalize_spec(
        {"shape": "line", "v_max": "fast"}, default_v_max=10.0, v_max_limit=12.0
    )

    assert "v_max" in err


# CSV reading + the combined load the nodes call

def test_read_line_csv_parses_rows_as_floats(tmp_path):
    p = tmp_path / "l.csv"
    p.write_text("0.0,1.0,2.0,3.0,4.0,5.0,6.0\n")

    assert line_spec.read_line_csv(str(p)) == [(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)]


def test_read_line_csv_skips_a_header_row(tmp_path):
    p = tmp_path / "l.csv"
    p.write_text("s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n"
                 "0.0,1.0,2.0,3.0,4.0,5.0,6.0\n")

    assert line_spec.read_line_csv(str(p)) == [(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)]


def test_read_line_csv_on_a_missing_file_is_empty(tmp_path):
    assert line_spec.read_line_csv(str(tmp_path / "nope.csv")) == []


def test_load_shape_returns_the_path_and_scaled_rows(tmp_path):
    _write_line(tmp_path, "line1.csv", 10.0, n=3)

    path, rows = line_spec.load_shape(
        str(tmp_path), {"shape": "line1", "v_min": 0.0, "v_max": 6.0, "v_mult": 1.0}
    )

    assert path.endswith("line1.csv")
    assert [r[5] for r in rows] == pytest.approx([6.0, 6.0, 6.0])


def test_load_shape_on_an_unknown_shape_returns_no_path(tmp_path):
    _write_line(tmp_path, "line1.csv", 10.0)

    assert line_spec.load_shape(
        str(tmp_path), {"shape": "nope", "v_min": 0.0, "v_max": 6.0, "v_mult": 1.0}
    ) == (None, [])


# Single-value form, shared by the row scaler and the /map endpoint

def test_scale_vx_multiplies_then_clamps():
    assert line_spec.scale_vx(10.0, v_min=0.0, v_max=6.0, v_mult=2.0) == pytest.approx(6.0)


def test_scale_vx_honours_the_floor():
    assert line_spec.scale_vx(10.0, v_min=4.0, v_max=9.0, v_mult=0.1) == pytest.approx(4.0)


def test_scale_vx_passes_a_value_inside_the_band_through():
    assert line_spec.scale_vx(10.0, v_min=0.0, v_max=9.0, v_mult=0.5) == pytest.approx(5.0)
