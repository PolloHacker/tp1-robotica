from inspect import signature

import numpy as np

from main import (
    DEFAULT_NAVIGATION_DURATION,
    HokuyoSensorSim,
    choose_navigation_command,
    plot_navigation_history,
    resume_simulation_if_paused,
    run_navigation,
    start_simulation,
)


def test_choose_navigation_command_moves_forward_when_front_is_clear():
    ranges = np.full(8, 1.0)
    ranges[len(ranges) // 2] = 3.0

    assert choose_navigation_command(ranges) == (0.5, 0.0)


def test_choose_navigation_command_turns_right_before_left_when_front_is_blocked():
    ranges = np.full(8, 1.0)
    ranges[2] = 3.0

    assert choose_navigation_command(ranges) == (0.0, -np.deg2rad(30))


def test_plot_navigation_history_writes_one_nonempty_global_png(tmp_path):
    path = plot_navigation_history(
        np.array([[0.0, 0.0], [1.0, 0.5]]),
        np.array([[0.5, 0.2], [1.2, 0.8]]),
        tmp_path / "navigation.png",
    )

    assert path == tmp_path / "navigation.png"
    assert path.stat().st_size > 0


def test_start_simulation_resumes_scene_that_pauses_on_start():
    class FakeSimulation:
        simulation_paused = 8

        def __init__(self):
            self.start_calls = 0

        def startSimulation(self):
            self.start_calls += 1

        def getSimulationState(self):
            return 8 if self.start_calls == 1 else 17

    sim = FakeSimulation()

    start_simulation(sim, settle_time=0)

    assert sim.start_calls == 2


def test_resume_simulation_if_paused_starts_the_simulation_again():
    class FakeSimulation:
        simulation_paused = 8

        def __init__(self):
            self.state = 8
            self.start_calls = 0

        def getSimulationState(self):
            return self.state

        def startSimulation(self):
            self.start_calls += 1
            self.state = 17

    sim = FakeSimulation()

    resumed = resume_simulation_if_paused(sim)

    assert resumed is True
    assert sim.start_calls == 1


def test_hokuyo_ignores_invalid_vision_sensor_frame():
    class FakeSimulation:
        def getObject(self, path):
            return path

        def readVisionSensor(self, sensor):
            return -1

    sensor = HokuyoSensorSim(FakeSimulation(), "/PioneerP3DX/fastHokuyo")

    points = sensor.get_points()

    assert points.shape == (0, 3)


def test_navigation_duration_defaults_to_forty_seconds():
    assert DEFAULT_NAVIGATION_DURATION == 40.0
    assert signature(run_navigation).parameters["duration"].default == 40.0
