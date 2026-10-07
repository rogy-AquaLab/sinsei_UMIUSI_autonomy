import math
import os
import sys
import types

import pytest
import yaml

from umiusi_autonomy import fsm_params

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "competition.yaml")


def _fake_fsm(**module_values):
    module = types.ModuleType("fake_fsm_module")
    for k, v in module_values.items():
        setattr(module, k, v)

    class Behavior:
        ki_heave = 0.0
        heave_bias_max = 0.25
    Behavior.__module__ = module.__name__
    sys.modules[module.__name__] = module
    return module, Behavior()


@pytest.fixture
def fake():
    module, b = _fake_fsm(SEARCH_YAW=0.5, FACE_TOL=math.radians(45.0), RAM_MAX_STEPS=85,
                          RAM_COMMIT_BBOX=0.26, CONFIRM_MIN_PEAK=0.26)
    yield module, b
    del sys.modules[module.__name__]


def test_module_constant_is_overridden(fake):
    module, b = fake
    fsm_params.apply(b, fsm_params.by_ros_name("fsm.search_yaw"), 0.3)
    assert module.SEARCH_YAW == 0.3


def test_deg_params_go_in_as_radians_and_read_back_as_degrees(fake):
    module, b = fake
    p = fsm_params.by_ros_name("fsm.face_tol_deg")
    assert fsm_params.current_value(b, p) == pytest.approx(45.0)
    fsm_params.apply(b, p, 30.0)
    assert module.FACE_TOL == pytest.approx(math.radians(30.0))


def test_int_params_stay_int(fake):
    module, b = fake
    fsm_params.apply(b, fsm_params.by_ros_name("fsm.ram_max_steps"), 200.0)
    assert module.RAM_MAX_STEPS == 200 and isinstance(module.RAM_MAX_STEPS, int)


def test_ram_commit_bbox_moves_confirm_min_peak_with_it(fake):
    module, b = fake
    fsm_params.apply(b, fsm_params.by_ros_name("fsm.ram_commit_bbox"), 0.30)
    assert (module.RAM_COMMIT_BBOX, module.CONFIRM_MIN_PEAK) == (0.30, 0.30)


def test_yaml_number_types_match_the_table():
    with open(CONFIG) as f:
        fsm = yaml.safe_load(f)["auto_target_generator"]["ros__parameters"]["fsm"]
    for p in fsm_params.FSM_PARAMS:
        want = int if p.kind == "int" else float
        assert type(fsm[p.name]) is want, (p.name, fsm[p.name])


def test_instance_fields_are_set_on_the_behavior(fake):
    _, b = fake
    fsm_params.apply(b, fsm_params.by_ros_name("fsm.ki_heave"), 0.3)
    assert b.ki_heave == 0.3


def test_missing_lists_names_an_old_wheel_lacks(fake):
    absent = fsm_params.missing(fake[1])
    assert "search_yaw" not in absent and "ram_surge" in absent


def test_unknown_names_are_not_fsm_params():
    assert fsm_params.by_ros_name("fsm.no_such_thing") is None
    assert fsm_params.by_ros_name("surge_sign") is None


def test_every_fsm_entry_in_the_yaml_is_a_known_param():
    with open(CONFIG) as f:
        fsm = yaml.safe_load(f)["auto_target_generator"]["ros__parameters"]["fsm"]
    known = {p.name for p in fsm_params.FSM_PARAMS}
    assert set(fsm) <= known, set(fsm) - known
    assert set(fsm) == known   # yaml で全部見えること


def test_measured_dt_tracks_a_slow_timer():
    nominal = 0.02
    assert fsm_params.measured_dt(None, 10.0, nominal) == nominal
    assert fsm_params.measured_dt(10.0, 10.038, nominal) == pytest.approx(0.038)   # 26 Hz
    assert fsm_params.measured_dt(10.0, 12.0, nominal) == pytest.approx(5 * nominal)  # 止まっていた後は頭打ち


def test_table_matches_the_installed_fsm():
    behavior = pytest.importorskip("umiusi_perception.autonomy.behavior")
    b = behavior.BalloonBehavior()
    assert fsm_params.missing(b) == []
