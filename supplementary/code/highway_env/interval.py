import itertools

import numpy as np


def intervals_product(a, b):
    """
        Compute the product of two intervals
    :param a: interval [a_min, a_max]
    :param b: interval [b_min, b_max]
    :return: the interval of their product ab
    """
    p = lambda x: np.maximum(x, 0)
    n = lambda x: np.maximum(-x, 0)
    return np.array(
        [np.dot(p(a[0]), p(b[0])) - np.dot(p(a[1]), n(b[0])) - np.dot(n(a[0]), p(b[1])) + np.dot(n(a[1]), n(b[1])),
         np.dot(p(a[1]), p(b[1])) - np.dot(p(a[0]), n(b[1])) - np.dot(n(a[1]), p(b[0])) + np.dot(n(a[0]), n(b[0]))])


def intervals_scaling(a, b):
    """
        Scale an intervals
    :param a: matrix a
    :param b: interval [b_min, b_max]
    :return: the interval of their product ab
    """
    p = lambda x: np.maximum(x, 0)
    n = lambda x: np.maximum(-x, 0)
    return np.array(
        [np.dot(p(a), b[0]) - np.dot(n(a), b[1]),
         np.dot(p(a), b[1]) - np.dot(n(a), b[0])])


def vector_interval_section(v_i, direction):
    corners = [[v_i[0, 0], v_i[0, 1]],
               [v_i[0, 0], v_i[1, 1]],
               [v_i[1, 0], v_i[0, 1]],
               [v_i[1, 0], v_i[1, 1]]]
    corners_dist = [np.dot(corner, direction) for corner in corners]
    return np.array([min(corners_dist), max(corners_dist)])


def interval_absolute_to_local(position_i, lane):
    """
        Converts an interval in absolute x,y coordinates to an interval in local (longiturinal, lateral) coordinates
    :param position_i: the position interval [x_min, x_max]
    :param lane: the lane giving the local frame
    :return: the corresponding local interval
    """
    position_corners = [[position_i[0, 0], position_i[0, 1]],
                        [position_i[0, 0], position_i[1, 1]],
                        [position_i[1, 0], position_i[0, 1]],
                        [position_i[1, 0], position_i[1, 1]]]
    corners_local = np.array([lane.local_coordinates(c) for c in position_corners])
    longitudinal_i = np.array([min(corners_local[:, 0]), max(corners_local[:, 0])])
    lateral_i = np.array([min(corners_local[:, 1]), max(corners_local[:, 1])])
    return longitudinal_i, lateral_i


def interval_local_to_absolute(longitudinal_i, lateral_i, lane):
    """
        Converts an interval in local (longiturinal, lateral) coordinates to an interval in absolute x,y coordinates
    :param longitudinal_i: the longitudinal interval [L_min, L_max]
    :param lateral_i: the lateral interval [l_min, l_max]
    :param lane: the lane giving the local frame
    :return: the corresponding absolute interval
    """
    corners_local = [[longitudinal_i[0], lateral_i[0]],
                     [longitudinal_i[0], lateral_i[1]],
                     [longitudinal_i[1], lateral_i[0]],
                     [longitudinal_i[1], lateral_i[1]]]
    corners_absolute = np.array([lane.position(*c) for c in corners_local])
    position_i = np.array([np.amin(corners_absolute, axis=0), np.amax(corners_absolute, axis=0)])
    return position_i


def polytope(parametrized_f, params_intervals):
    """

    :param parametrized_f: parametrized matrix function
    :param params_intervals: axes: [min, max], params
    :return: a0, d_a polytope that represents the matrix interval
    """
    params_means = params_intervals.mean(axis=0)
    a0 = parametrized_f(params_means)
    vertices_id = itertools.product([0, 1], repeat=params_intervals.shape[1])
    d_a = []
    for vertex_id in vertices_id:
        params_vertex = params_intervals[vertex_id, np.arange(len(vertex_id))]
        d_a.append(parametrized_f(params_vertex) - parametrized_f(params_means))
    d_a = list({d_a_i.tostring(): d_a_i for d_a_i in d_a}.values())
    return a0, d_a


def is_metzler(matrix):
    return (matrix - np.diag(np.diag(matrix)) >= 0).all()


class LPV(object):
    def __init__(self, x0, a0, da, b=None, d_i=None, c=None, center=None, x_i=None):
        """
        A Linear Parameter-Varying system:
                    dx = (a0 + sum(da))(x - center) + bd + c
        :param x0: initial state
        :param a0: nominal dynamics
        :param da: list of dynamics deviations
        :param b: perturbation matrix
        :param d_i: perturbation bounds
        :param c: constant known perturbation
        :param center: asymptotic state
        :param x_i: initial state interval
        """
        self.x0 = np.array(x0, dtype=float)
        self.a0 = np.array(a0, dtype=float)
        self.da = [np.array(da_i) for da_i in da]
        self.b = np.array(b) if b is not None else np.zeros((*self.x0.shape, 1))
        self.d_i = np.array(d_i) if d_i is not None else np.zeros((2, 1))
        self.c = np.array(c) if c is not None else np.zeros(self.x0.shape)
        self.center = np.array(center) if center is not None else np.zeros(self.x0.shape)
        self.coordinates = None

        self.x_i = np.array(x_i) if x_i is not None else np.array([self.x0, self.x0])
        self.x_i_t = None

        self.update_coordinates_frame(self.a0)

    def update_coordinates_frame(self, a0):
        """
            Ensure that the dynamics matrix A0 is Metzler.

            If not, design a coordinate transformation and apply it to the model and state interval.
        :param a0: the dynamics matrix A0
        """
        self.coordinates = None
        # Rotation
        if not is_metzler(a0):
            eig_v, transformation = np.linalg.eig(a0)
            if np.isreal(eig_v).all():
                self.coordinates = (transformation, np.linalg.inv(transformation))
            else:
                print("Non Metzler A0 with complex eigenvalues: ", eig_v)
        else:
            self.coordinates = (np.eye(a0.shape[0]), np.eye(a0.shape[0]))

        # Forward coordinates change of states and models
        self.a0 = self.change_coordinates(self.a0, matrix=True)
        self.da = self.change_coordinates(self.da, matrix=True)
        # self.b = self.change_coordinates(self.b, offset=False)
        self.c = self.change_coordinates(self.c, offset=False)
        self.x_i_t = np.array(self.change_coordinates([x for x in self.x_i]))

    def set_control(self, control):
        self.c = self.change_coordinates(control, offset=False)

    def change_coordinates(self, value, matrix=False, back=False, interval=False, offset=True):
        """
            Perform a change of coordinate: rotation and centering.

        :param value: the object to transform
        :param matrix: is it a matrix or a vector?
        :param back: if True, transform back to the original coordinates
        :param interval: when transforming an interval, lossy interval arithmetic must be used to preserve the inclusion
                         property.
        :param offset: should we apply the centering or not
        :return: the transformed object
        """
        if self.coordinates is None:
            return value
        transformation, transformation_inv = self.coordinates
        if interval:
            if back:
                value = intervals_scaling(transformation,
                    value[:, :, np.newaxis]).squeeze() + offset * np.array([self.center, self.center])
                return value
            else:
                value = value - offset * np.array([self.center, self.center])
                value = intervals_scaling(transformation_inv, value[:, :, np.newaxis]).squeeze()
                return value
        elif matrix:  # Matrix
            if back:
                return transformation @ value @ transformation_inv
            else:
                return transformation_inv @ value @ transformation
        elif isinstance(value, list):  # List
            return [self.change_coordinates(v, back) for v in value]
        else:
            if back:
                return transformation @ value + offset * self.center
            else:
                return transformation_inv @ (value - offset * self.center)

    def step(self, dt):
        if is_metzler(self.a0):
            self.x_i_t = self.step_interval_predictor(self.x_i_t, dt)
        else:
            self.x_i_t = self.step_interval_predictor(self.x_i_t, dt)

    def step_simple_predictor(self, x_i, dt):
        """
            Step an interval predictor with box uncertainty.

        :param x_i: state interval at time t
        :param dt: time step
        :return: state interval at time t+dt
        """
        a0, da, b, d_i, c = self.a0, self.da, self.b, self.d_i, self.c
        a_i = a0 + sum(intervals_product([0, 1], [da_i, da_i]) for da_i in da)
        dx_i = intervals_product(a_i, x_i) + intervals_product([b, b], d_i) + c
        return x_i + dx_i*dt

    def step_interval_predictor(self, x_i, dt):
        """
            Step an interval predictor with polytopic uncertainty.

        :param x_i: state interval at time t
        :param dt: time step
        :return: state interval at time t+dt
        """
        a0, da, b, d_i, c = self.a0, self.da, self.b, self.d_i, self.c
        p = lambda x: np.maximum(x, 0)
        n = lambda x: np.maximum(-x, 0)
        da_p = sum(p(da_i) for da_i in da)
        da_n = sum(n(da_i) for da_i in da)
        x_m, x_M = x_i[0, :, np.newaxis], x_i[1, :, np.newaxis]
        d_m, d_M = d_i[0, :, np.newaxis], d_i[1, :, np.newaxis]
        dx_m = a0 @ x_m - da_p @ n(x_m) - da_n @ p(x_M) + p(b) @ d_m - n(b) @ d_M + c[:, np.newaxis]
        dx_M = a0 @ x_M + da_p @ p(x_M) + da_n @ n(x_m) + p(b) @ d_M - n(b) @ d_m + c[:, np.newaxis]
        dx_i = np.array([dx_m.squeeze(axis=-1), dx_M.squeeze(axis=-1)])
        return x_i + dx_i * dt
