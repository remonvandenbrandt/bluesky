from netCDF4 import date2num, num2date
from scipy import ndimage, interpolate
import numpy as np
import iris
from bluesky.tools.aero import vatmos, kts
import bluesky as bs


class WindIris:
    """
    Create interpolation and statistical routines that apply to the wind forecast.

    Parameters
    ----------
    filename : str
        Path to netCDF weather data file.

    Notes
    -----
    Schematic of the coordinate system:

      +-------------- 90 lat ------------+
      |                |                 |
      |                |                 |
      |                |                 |
      +- 0 ------------+--------- 359.5 -| lon
      |                |                 |
      |                |                 |
      |                |                 |
      +-------------- -90 ---------------+

    """

    def __init__(self):
        self.winddim = 3
        self.cubes = []
        self.lat = []
        self.lon = []
        self.pressure = []
        self.t = []
        self.north_mean = []
        self.east_mean = []
        self.ens = []
        self.north = []
        self.east = []
        self.__ens = []

    def _get_mean(self, lat, lon, pressure, time):
        time = date2num(time, units='hours since 1900-01-01 00:00:0.0', calendar='gregorian')
        return self.__interpolate(self.north_mean, self.east_mean, lat, lon, pressure, time)

    def _get_wind(self, lat, lon, pressure, time, ens=None):
        """
        Retrieve the north and south component of the windfield, interpolated at a given positions.

        Parameters
        ----------
        lat: array_like
            latitude.
        lon: array_like
            Longitude.
        pressure: array_like
            Pressure in Pa.
        time: datetime
            timestamp.
        ens: int, optional
            Ensemble member.
        ignore_date: bool
            Ignore the date, instead only use time.

        Returns
        -------
        north: array_like
             North component of the wind.
        east: array_like
            East component of the wind.
        """

        # TODO: find a faster alternative to date2num
        time = date2num(time, units='hours since 1900-01-01 00:00:0.0', calendar='gregorian')

        if ens:
            self.__load_ensemble(ens)
        return self.__interpolate(self.north, self. east, lat, lon, pressure, time)

    def load_file(self, filename):
        self.cubes = iris.load(filename, ['northward_wind', 'eastward_wind'])
        self.cubes[0].coord('pressure_level').convert_units('pascal')
        self.cubes[1].coord('pressure_level').convert_units('pascal')

        self.lat = self.cubes[0].coord('latitude').points
        self.lon = self.cubes[0].coord('longitude').points
        self.pressure = self.cubes[0].coord('pressure_level').points
        self.t = self.cubes[0].coord('time').points
        if self.cubes[0].coords('ensemble_member'):
            self.ens = self.cubes[0].coord('ensemble_member').points
            self.north_mean = self.cubes[0].collapsed('ensemble_member', iris.analysis.MEAN).data
            self.east_mean = self.cubes[1].collapsed('ensemble_member', iris.analysis.MEAN).data
        else:
            self.ens = []
            self.north = self.cubes[0].data
            self.east = self.cubes[1].data
        self.__ens = []
        self.__load_ensemble(1)

    # -----  mimic windsim class API -------------------
    def get(self, lat, lon, alt=0):
        """ Get wind vector at given position (and optionally altitude) """

        vn, ve = self.getdata(lat, lon, alt)

        wdir = (np.degrees(np.arctan2(ve, vn)) + 180) % 360
        wspd = np.sqrt(vn * vn + ve * ve)

        txt = "WIND AT %.5f, %.5f: %03d/%d" % (lat, lon, np.round(wdir), np.round(wspd / kts))

        return True, txt

    def getdata(self, userlat, userlon, useralt=0.0):
        p = vatmos(useralt)[0]
        time = bs.sim.utc

        return self._get_wind(userlat, userlon, p, time)

    def addpoint(self, lat, lon, winddir, windspd, windalt=None):
        # not used
        pass

    def remove(self, idx):
        # not used
        pass

    def add(self, *arg):
        pass

    def clear(self):
        # not used
        pass

    @property
    def ensembles(self):
        if self.ens.any():
            return self.cubes[0].coord('ensemble_member').points
        else:
            return [1]

    @property
    def time(self):
        """Time instance of forecast in hours since 1900-01-01 00:00:0.0"""
        return num2date(self.cubes[0].coord('time').points, units='hours since 1900-01-01 00:00:0.0',
                        calendar='gregorian')

    def __load_ensemble(self, ens):
        # check if cubes contains ensemble members
        if list(self.ens):
            # if ens member is different from the one currently loaded
            if self.__ens is not ens:
                self.north = self.cubes[0].extract(iris.Constraint(ensemble_member=ens)).data
                self.east = self.cubes[1].extract(iris.Constraint(ensemble_member=ens)).data
            self.__ens = ens

    def __interpolate(self, cube_n, cube_e, lat, lon, pressure, time):
        # wrap longitude around for periodic boundary
        lon = (lon + 360) % 360

        # saturate pressure altitude
        pressure = np.clip(pressure, self.pressure[0], self.pressure[-1])

        # find coordinates, assumes 720/360 grid size TODO change to be more flexible
        lon_i = lon * (720 / 360)
        lat_i = (lat - 90) * (360 / -180)

        f = interpolate.interp1d(self.pressure, range(len(self.pressure)), bounds_error=True, assume_sorted=True)
        pres_i = f(pressure)
        time_i = (time - self.cubes[0].coord('time').points[0]) / 6  # note: this assumes 6 hour intervals

        # TODO check for out of bounds
        coord = np.vstack((time_i, pres_i, lat_i, lon_i))

        north = ndimage.map_coordinates(cube_n, coord, order=1, mode='wrap')
        east = ndimage.map_coordinates(cube_e, coord, order=1, mode='wrap')
        return north, east
