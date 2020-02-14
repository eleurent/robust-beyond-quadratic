import copy

import numpy as np

from highway_env import utils
from highway_env.interval import polytope, LPV, interval_absolute_to_local, \
    interval_local_to_absolute
from highway_env.vehicle.behavior import LinearVehicle
from highway_env.vehicle.control import MDPVehicle


class IntervalVehicle(LinearVehicle):
    """
        Estimator for the interval-membership of a LinearVehicle under parameter uncertainty.

        The model trajectory is stored in a model_vehicle, and the lower and upper bounds of the states are stored
        in a min_vehicle and max_vehicle. Note that these vehicles do not follow a proper Vehicle dynamics, and
        are only used for storage of the bounds.
    """
    def __init__(self,
                 road,
                 position,
                 heading=0,
                 velocity=0,
                 target_lane_index=None,
                 target_velocity=None,
                 route=None,
                 enable_lane_change=True,
                 timer=None,
                 theta_a_i=None,
                 theta_b_i=None,
                 data=None):
        """
        :param theta_a_i: The interval of possible acceleration parameters
        :param theta_b_i: The interval of possible steering parameters
        """
        super().__init__(road,
                         position,
                         heading,
                         velocity,
                         target_lane_index,
                         target_velocity,
                         route,
                         enable_lane_change,
                         timer)
        self.theta_a_i = theta_a_i if theta_a_i is not None else LinearVehicle.ACCELERATION_RANGE
        self.theta_b_i = theta_b_i if theta_b_i is not None else LinearVehicle.STEERING_RANGE
        self.data = data
        self.interval = VehicleInterval(self)
        self.trajectory = []
        self.interval_trajectory = []
        self.longitudinal_lpv, self.lateral_lpv = None, None
        self.previous_target_lane_index = self.target_lane_index

    @classmethod
    def create_from(cls, vehicle):
        v = cls(vehicle.road,
                vehicle.position,
                heading=vehicle.heading,
                velocity=vehicle.velocity,
                target_lane_index=getattr(vehicle, 'target_lane_index', None),
                target_velocity=getattr(vehicle, 'target_velocity', None),
                route=getattr(vehicle, 'route', None),
                timer=getattr(vehicle, 'timer', None),
                theta_a_i=getattr(vehicle, 'theta_a_i', None),
                theta_b_i=getattr(vehicle, 'theta_b_i', None),
                data=getattr(vehicle, "data", None))
        return v

    def step(self, dt):
        self.store_trajectories()
        if self.crashed:
            self.interval = VehicleInterval(self)
        else:
            self.predictor_step(dt)
        super(IntervalVehicle, self).step(dt)

    def predictor_step(self, dt):
        """
            Step the interval predictor dynamics
        :param dt: timestep [s]
        """
        # Create longitudinal and lateral LPVs
        self.predictor_init()

        # Detect lane change and update intervals of local coordinates with the new frame
        if self.target_lane_index != self.previous_target_lane_index:
            position_i = self.interval.position
            target_lane = self.road.network.get_lane(self.target_lane_index)
            previous_target_lane = self.road.network.get_lane(self.previous_target_lane_index)
            longi_i, lat_i = interval_absolute_to_local(position_i, target_lane)
            psi_i = self.interval.heading + \
                    target_lane.heading_at(longi_i.mean()) - previous_target_lane.heading_at(longi_i.mean())
            x_i_local_unrotated = np.transpose([lat_i, psi_i])
            new_x_i_t = self.lateral_lpv.change_coordinates(x_i_local_unrotated, back=False, interval=True)
            delta = new_x_i_t.mean(axis=0) - self.lateral_lpv.x_i_t.mean(axis=0)
            self.lateral_lpv.x_i_t += delta
            x_i_local_unrotated = self.longitudinal_lpv.change_coordinates(self.longitudinal_lpv.x_i_t,
                                                                         back=True,
                                                                         interval=True)
            x_i_local_unrotated[:, 0] = longi_i
            new_x_i_t = self.longitudinal_lpv.change_coordinates(x_i_local_unrotated,
                                                                 back=False,
                                                                 interval=True)
            self.longitudinal_lpv.x_i_t += new_x_i_t.mean(axis=0) - self.longitudinal_lpv.x_i_t.mean(axis=0)
            self.previous_target_lane_index = self.target_lane_index

        # Step
        self.longitudinal_lpv.step(dt)
        self.lateral_lpv.step(dt)

        # Backward coordinates change
        x_i_long = self.longitudinal_lpv.change_coordinates(self.longitudinal_lpv.x_i_t, back=True, interval=True)
        x_i_lat = self.lateral_lpv.change_coordinates(self.lateral_lpv.x_i_t, back=True, interval=True)

        # Conversion from rectified to true coordinates
        target_lane = self.road.network.get_lane(self.target_lane_index)
        position_i = interval_local_to_absolute(x_i_long[:, 0], x_i_lat[:, 0], target_lane)
        self.interval.position = position_i
        self.interval.velocity = x_i_long[:, 2]
        self.interval.heading = x_i_lat[:, 1]

    def predictor_init(self):
        """
            Initialize the LPV models used for interval prediction
        """
        position_i = self.interval.position
        target_lane = self.road.network.get_lane(self.target_lane_index)
        longi_i, lat_i = interval_absolute_to_local(position_i, target_lane)
        v_i = self.interval.velocity
        psi_i = self.interval.heading - self.lane.heading_at(longi_i.mean())

        # Longitudinal predictor
        if not self.longitudinal_lpv:
            front_interval = self.get_front_interval()

            # LPV specification
            if front_interval:
                f_longi_i, f_lat_i = interval_absolute_to_local(front_interval.position, target_lane)
                f_pos = f_longi_i[0]
                f_vel = front_interval.velocity[0]
            else:
                f_pos, f_vel = 0, 0
            x0 = [longi_i[0], f_pos, v_i[0], f_vel]
            center = [-self.DISTANCE_WANTED - self.target_velocity * self.TIME_WANTED,
                      0,
                      self.target_velocity,
                      self.target_velocity]
            noise = 1
            b = np.array([1, 0, 0, 0])[:, np.newaxis]
            d_i = np.array([[-1], [1]]) * noise
            c = [self.target_velocity, self.target_velocity, 0, 0]
            a0, da = self.longitudinal_matrix_polytope()
            self.longitudinal_lpv = LPV(x0, a0, da, b, d_i, c, center)

            # Lateral predictor
            if not self.lateral_lpv:
                # LPV specification
                x0 = [lat_i[0], psi_i[0]]
                center = [0, 0]
                noise = 0.5
                b = np.identity(2)
                d_i = np.array([[-1, 0], [1, 0]]) * noise
                c = [0, 0]
                a0, da = self.lateral_matrix_polytope()
                self.lateral_lpv = LPV(x0, a0, da, b, d_i, c, center)

    def longitudinal_matrix_polytope(self):
        return IntervalVehicle.parameter_box_to_polytope(self.theta_a_i, self.longitudinal_structure)

    def lateral_matrix_polytope(self):
        return IntervalVehicle.parameter_box_to_polytope(self.theta_b_i, self.lateral_structure)

    @staticmethod
    def parameter_box_to_polytope(parameter_box, structure):
        a, phi = structure()
        a_theta = lambda params: a + np.tensordot(phi, params, axes=[0, 0])
        return polytope(a_theta, parameter_box)

    def get_front_interval(self):
        # TODO: For now, we assume the front vehicle follows the models' front vehicle
        front_vehicle, _ = self.road.neighbour_vehicles(self)
        if front_vehicle:
            if isinstance(front_vehicle, IntervalVehicle):
                # Use interval from the observer estimate of the front vehicle
                front_interval = front_vehicle.interval
            else:
                # The front vehicle trajectory interval is not being estimated, so it should be considered as certain.
                # We use a new observer created from that current vehicle state, which will have full certainty.
                front_interval = IntervalVehicle.create_from(front_vehicle).interval
        else:
            front_interval = None
        return front_interval

    def store_trajectories(self):
        """
            Store the current model, min and max states to a trajectory list
        """
        self.trajectory.append(LinearVehicle.create_from(self))
        self.interval_trajectory.append(copy.deepcopy(self.interval))

    def check_collision(self, other):
        """
            For robust planning, we assume that MDPVehicles collide with the uncertainty set of an IntervalVehicle,
            which corresponds to worst-case outcome.

        :param other: the other vehicle
        """
        if not isinstance(other, MDPVehicle):
            return super(IntervalVehicle, self).check_collision(other)

        if not self.COLLISIONS_ENABLED or self.crashed or other is self:
            return

        # Fast rectangular pre-check
        if not utils.point_in_rectangle(other.position,
                                        self.interval.position[0] - self.LENGTH,
                                        self.interval.position[1] + self.LENGTH):
            return

        # Projection of other vehicle to uncertainty rectangle. This is the possible position of this vehicle which is
        # the most likely to collide with other vehicle
        projection = np.minimum(np.maximum(other.position, self.interval.position[0]),
                                self.interval.position[1])
        # Accurate rectangular check
        if utils.rotated_rectangles_intersect((projection, self.LENGTH, self.WIDTH, self.heading),
                                              (other.position, 0.9*other.LENGTH, 0.9*other.WIDTH, other.heading)):
            self.velocity = other.velocity = min(self.velocity, other.velocity)
            self.crashed = other.crashed = True


class VehicleInterval(object):
    def __init__(self, vehicle):
        self.position = np.array([vehicle.position, vehicle.position], dtype=float)
        self.velocity = np.array([vehicle.velocity, vehicle.velocity], dtype=float)
        self.heading = np.array([vehicle.heading, vehicle.heading], dtype=float)
