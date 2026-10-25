"""Student-owned constrained least squares for the calibrated joint maps.

Shared by the normal fitter and physical parameter report. No simulator truth
or experiment files are dependencies. Quadratic extrema are analytic; higher
polynomial degrees use real stationary roots of the corresponding polynomial.
"""

import numpy as np
from numpy.polynomial import Polynomial


def distortion_bounds(model, params):
    from src.simulation import constants

    degree = model.n_poly
    values = np.asarray(params, dtype=float)
    if values.shape != (22 + 6 * degree,) or not np.isfinite(values).all():
        raise ValueError('Invalid kinematic parameter vector')
    scales = np.maximum(abs(model.theta_min), abs(model.theta_max))
    coefficients = values[22:].reshape(6, degree)
    reports, margins, jacobian = [], [], []

    def critical(poly, lower, upper):
        roots = poly.roots()
        return [float(root.real) for root in roots
                if abs(root.imag) <= 1e-9 * (1 + abs(root.real)) and lower < root.real < upper]

    for k in range(6):
        lower, upper = (np.array([model.theta_min[k], model.theta_max[k]])
                        + model.joint_offsets[k]) / scales[k]
        polynomial = Polynomial(np.r_[model.joint_offsets[k], coefficients[k]])
        slope = polynomial.deriv() / scales[k]
        x = np.array([lower, upper] + critical(polynomial.deriv(), lower, upper))
        y = polynomial(x)
        imin, imax = int(np.argmin(y)), int(np.argmax(y))
        magnitude, derivative_limit = constants.DELTA_SYS[k], constants.OMEGA_SYS[k]
        margin = [1 - y[imax] / magnitude, 1 + y[imin] / magnitude]
        powers = np.arange(1, degree + 1)
        gradients = [-x[imax]**powers / magnitude, x[imin]**powers / magnitude]
        endpoints = [lower, upper]
        points = endpoints + critical(slope.deriv(), lower, upper)
        derivatives = slope(np.array(points))
        # For degree <=2 the four endpoint inequalities are exact and smooth,
        # even when both endpoint derivatives coincide. For higher degrees,
        # additionally constrain the minimum/maximum over stationary points.
        constrained_points = endpoints.copy()
        if degree > 2:
            constrained_points.extend([points[int(np.argmin(derivatives))],
                                       points[int(np.argmax(derivatives))]])
        for point in constrained_points:
            value = float(slope(point))
            gradient = powers * point**(powers - 1) / (scales[k] * derivative_limit)
            margin.extend([1 - value / derivative_limit, 1 + value / derivative_limit])
            gradients.extend([-gradient, gradient])
        block = np.zeros((len(margin), len(values)))
        block[:, 22 + k * degree:22 + (k + 1) * degree] = np.asarray(gradients)
        jacobian.append(block)
        margins.extend(margin)
        reports.append(dict(joint=k + 1, minimum=float(y[imin]), maximum=float(y[imax]),
                            max_absolute=float(max(abs(y))), magnitude_limit=float(magnitude),
                            magnitude_ratio=float(max(abs(y)) / magnitude),
                            minimum_derivative=float(min(derivatives)),
                            maximum_derivative=float(max(derivatives)),
                            max_absolute_derivative=float(max(abs(derivatives))),
                            derivative_limit=float(derivative_limit),
                            derivative_ratio=float(max(abs(derivatives)) / derivative_limit),
                            endpoint_derivatives=np.asarray(slope(np.array(endpoints)))))
    return reports, np.asarray(margins), np.vstack(jacobian)


def solve_constrained(model, data, initial, initial_residual, scales, lower, upper, options):
    """Return a candidate and diagnostics, with no mutation of model.params."""
    from time import perf_counter
    from scipy.optimize import minimize, nnls

    started = perf_counter()
    active = np.arange(1, len(initial))
    counts = dict(residual_calls=1, iterations=0, njev=0, nfev=1)

    def numerical_jacobian(params):
        jacobian = np.empty((len(initial_residual), len(active)))
        step = 1e-4 * np.maximum(1., abs(params / scales))
        for column, k in enumerate(active):
            delta = np.zeros_like(params)
            delta[k] = step[k] * scales[k]
            jacobian[:, column] = (model.residuals(params + delta, data)
                                  - model.residuals(params - delta, data)) / (2 * step[k])
        counts['residual_calls'] += 2 * len(active)
        counts['njev'] += 1
        return jacobian

    anchor = numerical_jacobian(initial)
    _, singular, vt = np.linalg.svd(anchor / np.sqrt(len(data)), full_matrices=False)
    if singular[-1] <= 1e-7 * singular[0]:
        raise ValueError('Gauge-fixed Jacobian has additional numerical degeneracy; inspect the measurement design')
    transform = np.zeros((len(initial), len(active)))
    transform[active] = scales[active, None] * (vt.T / singular)
    scaled_transform = transform[active] / scales[active, None]
    cache = dict(z=np.zeros(len(active)), residual=initial_residual,
                 gradient=scaled_transform.T @ anchor.T @ initial_residual / len(data))
    finite_lower, finite_upper = np.flatnonzero(np.isfinite(lower[active])), np.flatnonzero(np.isfinite(upper[active]))

    class EvaluationLimit(Exception):
        pass

    def unpack(z):
        return initial + transform @ z

    def residual(z):
        if not np.array_equal(cache['z'], z):
            if counts['nfev'] >= options['max_nfev']:
                raise EvaluationLimit()
            cache.update(z=z.copy(), residual=model.residuals(unpack(z), data), gradient=None)
            counts['nfev'] += 1
            counts['residual_calls'] += 1
        return cache['residual']

    def objective(z):
        r = residual(z)
        return float(.5 * r @ r / len(data))

    def gradient(z):
        r = residual(z)
        if cache['gradient'] is None:
            cache['gradient'] = scaled_transform.T @ numerical_jacobian(unpack(z)).T @ r / len(data)
        return cache['gradient']

    def constraints(z):
        params = unpack(z)
        return np.r_[distortion_bounds(model, params)[1],
                     (params[active][finite_lower] - lower[active][finite_lower]) / scales[active][finite_lower],
                     (upper[active][finite_upper] - params[active][finite_upper]) / scales[active][finite_upper]]

    def constraint_jacobian(z):
        return np.vstack([distortion_bounds(model, unpack(z))[2] @ transform,
                          scaled_transform[finite_lower], -scaled_transform[finite_upper]])

    def callback(z):
        counts['iterations'] += 1
        if options['verbose']:
            print('constrained iteration', counts['iterations'], 'cost', objective(z) * len(data),
                  'min margin', float(min(constraints(z))), flush=True)

    try:
        result = minimize(objective, np.zeros(len(active)), jac=gradient, method='SLSQP',
                          constraints=[dict(type='ineq', fun=constraints, jac=constraint_jacobian)],
                          callback=callback, options=dict(ftol=options['ftol'],
                                                         maxiter=options['max_nfev'], disp=bool(options['verbose'])))
        z = result.x
        success, raw_status, message = bool(result.success), int(result.status), str(result.message)
    except EvaluationLimit:
        z = cache['z']
        success, raw_status, message = False, 9, 'Maximum objective evaluations exceeded'
    candidate = unpack(z)
    r = residual(z)
    margins = constraints(z)
    # A budget-limited unsuccessful attempt needs no extra numerical Jacobian.
    if success:
        grad = gradient(z)
        binding = margins < 1e-6
        matrix = constraint_jacobian(z)[binding].T
        multipliers = nnls(matrix, grad)[0] if binding.any() else np.zeros(0)
        optimality = float(max(abs(grad - matrix @ multipliers)))
    else:
        optimality = float('inf')
    return candidate, dict(solver='SLSQP', physical_constraints=True,
                           solver_success=success, status=0 if raw_status == 9 else raw_status,
                           raw_solver_status=raw_status, message=message,
                           final_cost=float(.5 * r @ r), optimality=optimality,
                           minimum_normalized_margin=float(min(margins)),
                           constraint_feasible=bool(min(margins) >= -1e-7),
                           reduced_initial_condition_number=float(singular[0] / singular[-1]),
                           **counts, wall_time_s=float(perf_counter()-started),
                           tolerance_note='SLSQP uses ftol; xtol/gtol apply to the explicit unconstrained TRF mode')
