/*
(The MIT License.)

Copyright (c) 2009, Kevin Lacker.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
 * This is a Java port of original C code by Kevin Lacker.
 * https://github.com/lacker/ikalman
 * 
 * Ported by Andrey Novikov, 2013
 */

package com.androzic.utils;

/* To use these functions:

 1. Start with a KalmanFilter created by alloc_filter_velocity2d.
 2. At fixed intervals, call update_velocity2d with the lat/long.
 3. At any time, to get an estimate for the current position,
 bearing, or speed, use the functions:
 get_lat_long
 get_bearing
 get_mph
 */

public class GeoTrackFilter {
	// FIXME Radius should be calculated depending on latitude instead
	// http://en.wikipedia.org/wiki/Earth_radius#Radius_at_a_given_geodetic_latitude
	private static final double EARTH_RADIUS_IN_METERS = 6371009;

	private KalmanFilter f;

	/*
	 * Create a GPS filter that only tracks two dimensions of position and
	 * velocity. The inherent assumption is that changes in velocity are
	 * randomly distributed around 0. Noise is a parameter you can use to alter
	 * the expected noise. 1.0 is the original, and the higher it is, the more a
	 * path will be "smoothed". Free with free_filter after using.
	 */

	GeoTrackFilter(double noise) {
		/*
		 * The state model has four dimensions: x, y, x', y' Each time step we
		 * can only observe position, not velocity, so the observation vector
		 * has only two dimensions.
		 */
		f = new KalmanFilter(4, 2);

		/*
		 * Assuming the axes are rectilinear does not work well at the poles,
		 * but it has the bonus that we don't need to convert between lat/long
		 * and more rectangular coordinates. The slight inaccuracy of our
		 * physics model is not too important.
		 */
		f.state_transition.set_identity_matrix();
		set_seconds_per_timestep(1.0);

		/* We observe (x, y) in each time step */
		f.observation_model.set_matrix(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

		/* Noise in the world. */
		double pos = 0.000001;
		f.process_noise_covariance.set_matrix(pos, 0.0, 0.0, 0.0, 0.0, pos,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

		/* Noise in our observation */
		f.observation_noise_covariance.set_matrix(pos * noise, 0.0, 0.0, pos
				* noise);

		/* The start position is totally unknown, so give a high variance */
		f.state_estimate.set_matrix(0.0, 0.0, 0.0, 0.0);
		f.estimate_covariance.set_identity_matrix();
		double trillion = 1000.0 * 1000.0 * 1000.0 * 1000.0;
		f.estimate_covariance.scale_matrix(trillion);
	}

	/* Set the seconds per timestep in the velocity2d model. */
	/*
	 * The position units are in thousandths of latitude and longitude. The
	 * velocity units are in thousandths of position units per second.
	 * 
	 * So if there is one second per timestep, a velocity of 1 will change the
	 * lat or long by 1 after a million timesteps.
	 * 
	 * Thus a typical position is hundreds of thousands of units. A typical
	 * velocity is maybe ten.
	 */
	void set_seconds_per_timestep(double seconds_per_timestep) {
		/*
		 * unit_scaler accounts for the relation between position and velocity
		 * units
		 */
		double unit_scaler = 0.001;
		f.state_transition.data[0][2] = unit_scaler * seconds_per_timestep;
		f.state_transition.data[1][3] = unit_scaler * seconds_per_timestep;
	}

	/* Update the velocity2d model with new gps data. */
	void update_velocity2d(double lat, double lon,
			double seconds_since_last_timestep) {
		set_seconds_per_timestep(seconds_since_last_timestep);
		f.observation.set_matrix(lat * 1000.0, lon * 1000.0);
		f.update();
	}

	/* Extract a lat long from a velocity2d Kalman filter. */
	double[] get_lat_long() {
		double[] latlon = new double[2];
		latlon[0] = f.state_estimate.data[0][0] / 1000.0;
		latlon[1] = f.state_estimate.data[1][0] / 1000.0;
		return latlon;
	}

	/*
	 * Extract velocity with lat-long-per-second units from a velocity2d Kalman
	 * filter.
	 */
	double[] get_velocity() {
		double[] delta_latlon = new double[2];
		delta_latlon[0] = f.state_estimate.data[2][0] / (1000.0 * 1000.0);
		delta_latlon[1] = f.state_estimate.data[3][0] / (1000.0 * 1000.0);
		return delta_latlon;
	}

	/*
	 * Extract a bearing from a velocity2d Kalman filter. 0 = north, 90 = east,
	 * 180 = south, 270 = west
	 */
	/*
	 * See http://www.movable-type.co.uk/scripts/latlong.html for formulas
	 */
	double get_bearing() {
		double x, y;
		double[] latlon = get_lat_long();
		double[] delta_latlon = get_velocity();

		/* Convert to radians */
		latlon[0] = Math.toRadians(latlon[0]);
		latlon[1] = Math.toRadians(latlon[1]);
		delta_latlon[0] = Math.toRadians(delta_latlon[0]);
		delta_latlon[1] = Math.toRadians(delta_latlon[1]);

		/* Do math */
		double lat1 = latlon[0] - delta_latlon[0];
		y = Math.sin(delta_latlon[1]) * Math.cos(latlon[0]);
		x = Math.cos(lat1) * Math.sin(latlon[0]) - Math.sin(lat1)
				* Math.cos(latlon[0]) * Math.cos(delta_latlon[1]);
		double bearing = Math.atan2(y, x);

		/* Convert to degrees */
		bearing = Math.toDegrees(bearing);
		return bearing;
	}

	/* Extract speed in meters per second from a velocity2d Kalman filter. */
	double get_speed(double altitude) {
		double[] latlon = get_lat_long();
		double[] delta_latlon = get_velocity();
		/*
		 * First, let's calculate a unit-independent measurement - the radii of
		 * the earth traveled in each second. (Presumably this will be a very
		 * small number.)
		 */

		/* Convert to radians */
		latlon[0] = Math.toRadians(latlon[0]);
		latlon[1] = Math.toRadians(latlon[1]);
		delta_latlon[0] = Math.toRadians(delta_latlon[0]);
		delta_latlon[1] = Math.toRadians(delta_latlon[1]);

		/* Haversine formula */
		double lat1 = latlon[0] - delta_latlon[0];
		double sin_half_dlat = Math.sin(delta_latlon[0] / 2.0);
		double sin_half_dlon = Math.sin(delta_latlon[1] / 2.0);
		double a = sin_half_dlat * sin_half_dlat + Math.cos(lat1) * Math.cos(latlon[0]) * sin_half_dlon * sin_half_dlon;
		double radians_per_second = 2 * Math.atan2(1000.0 * Math.sqrt(a), 1000.0 * Math.sqrt(1.0 - a));

		/* Convert units */
		double meters_per_second = radians_per_second * (EARTH_RADIUS_IN_METERS + altitude);
		return meters_per_second;
	}
	
    public static void main(String[] args)
    {
    }
}
