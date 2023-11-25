from boat_simulator.common.sensors import WindSensor, GPS
import numpy as np
from boat_simulator.common.generators import (
    MVGaussianGenerator,
    ConstantGenerator,
    GaussianGenerator,
)
import pytest


@pytest.fixture
def init_data():
    return np.array([1, 0])


class TestWindSensor:
    def test_wind_sensor_init(self, init_data):
        error_fn = None
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )

        assert ws.wind_noisemaker == error_fn
        assert np.all(ws.wind == init_data)

    def test_wind_sensor_read_no_error(self, init_data):
        error_fn = None
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )
        read_data = ws.read("wind")
        assert (init_data == read_data).all()

    def test_wind_sensor_read_constant_error(self, init_data):
        const_err = 0.1
        error_fn = ConstantGenerator(constant=0.1)
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )

        read_data = ws.read("wind")
        assert ((init_data + const_err) == read_data).all()

    def test_wind_sensor_read_mv_gaussian_error(self, init_data):
        mean = np.array([1, 1])
        cov = np.eye(2)
        error_fn = MVGaussianGenerator(mean=mean, cov=cov)
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )

        NUM_READINGS = 10000
        reading = np.zeros(shape=(NUM_READINGS, mean.size))
        for i in range(NUM_READINGS):
            reading[i, :] = ws.read("wind")

        sample_mean = np.mean(reading, axis=0)
        sample_cov = np.cov(reading, rowvar=False)

        assert np.allclose(sample_cov, cov, atol=0.2)
        assert np.isclose(sample_mean, mean + init_data, 0.1).all()

    def test_wind_sensor_update(self):
        init_data = np.zeros(2)
        error_fn = None
        ws = WindSensor(wind=init_data, wind_noisemaker=error_fn)

        NUM_READINGS = 100
        for i in range(NUM_READINGS):
            wind = ws.read("wind")
            assert (wind == np.array([i, i])).all()
            ws.update(wind=(wind + 1))


class TestGPS:
    def test_gps_init(self):
        lat_lon = np.array([1, 0])
        speed = 100
        heading = 1.09
        error_fn = None

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
            lat_lon_noisemaker=error_fn,
            speed_noisemaker=error_fn,
            heading_noisemaker=error_fn,
        )

        assert (gps.lat_lon == lat_lon).all()
        assert gps.speed == speed
        assert gps.heading == heading
        assert gps.lat_lon_noisemaker is error_fn
        assert gps.speed_noisemaker is error_fn
        assert gps.heading_noisemaker is error_fn

    def test_gps_read_no_error(self):
        lat_lon = np.array([1, 0])
        speed = np.random.randint(0, 100)
        heading = np.random.rand()
        error_fn = None

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
            lat_lon_noisemaker=error_fn,
            speed_noisemaker=error_fn,
            heading_noisemaker=error_fn,
        )

        assert (gps.read("lat_lon") == lat_lon).all()
        assert gps.read("speed") == speed
        assert gps.read("heading") == heading

    def test_gps_read_constant_error(self):
        lat_lon = np.array([1, 0])
        speed = np.random.randint(0, 100)
        heading = np.random.rand()
        constant = 3.01
        error_fn = ConstantGenerator(constant=constant)

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
            lat_lon_noisemaker=error_fn,
            speed_noisemaker=error_fn,
            heading_noisemaker=error_fn,
        )

        assert (gps.read("lat_lon") == lat_lon + constant).all()
        assert gps.read("speed") == speed + constant
        assert gps.read("heading") == heading + constant

    def test_gps_gaussian_error(self):
        lat_lon = np.array([1, 0])
        speed = np.random.randint(0, 100)
        heading = np.random.rand()
        mean = 0
        stdev = 1

        error_fn = GaussianGenerator(mean=mean, stdev=stdev)

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
            lat_lon_noisemaker=error_fn,
            speed_noisemaker=error_fn,
            heading_noisemaker=error_fn,
        )

        NUM_READINGS = 10000
        speed_readings = np.zeros(NUM_READINGS)
        heading_readings = np.zeros(NUM_READINGS)
        lat_lon_readings = np.zeros(shape=(NUM_READINGS, 2))
        for i in range(NUM_READINGS):
            speed_readings[i] = gps.read("speed")
            heading_readings[i] = gps.read("heading")
            lat_lon_readings[i, :] = gps.read("lat_lon")

        for reading, init_data in zip(
            [speed_readings, heading_readings, lat_lon_readings],
            [speed, heading, lat_lon],
        ):
            sample_mean = np.mean(reading, axis=0)
            assert np.isclose(sample_mean, mean + init_data, atol=0.1).all()

    def test_wind_sensor_update(self):
        lat_lon = np.array([0, 0])
        speed = 0
        heading = 0
        error_fn = None

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
            lat_lon_noisemaker=error_fn,
            speed_noisemaker=error_fn,
            heading_noisemaker=error_fn,
        )

        NUM_READINGS = 100
        for i in range(NUM_READINGS):
            speed_reading = gps.read("speed")
            assert speed_reading == i
            gps.update(speed=i + 1)

            heading_reading = gps.read("heading")
            assert heading_reading == i
            gps.update(heading=i + 1)

            lat_lon_reading = gps.read("lat_lon")
            assert (lat_lon_reading == np.array([i, i])).all()
            gps.update(lat_lon=(lat_lon_reading + 1))
