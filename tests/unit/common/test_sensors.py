from boat_simulator.common.sensors import WindSensor
import numpy as np


class TestWindSensor:
    def test_windsensor_init(self):
        pass

    def test_windsensor_read(self):
        MOCK_DATA = np.array([1, 2])
        in_data = {"wind": MOCK_DATA}
        w = WindSensor(in_data=in_data)
        assert np.array_equal(w.read("wind"), MOCK_DATA)

    def test_windsensor_update(self):
        MOCK_DATA = np.array([1, 2])
        in_data = {"wind": MOCK_DATA}
        w = WindSensor(in_data=in_data)

        new_data = np.array([3, 4])
        w.update({"wind": new_data})
        assert np.array_equal(new_data, w.read("wind"))
