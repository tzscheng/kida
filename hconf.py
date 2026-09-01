"""Hand mounting and equivalent-payload configuration.

The simulator loads the articulated hand model, including its fixed palm body.
The arm controllers do not, so they use one equivalent rigid payload at the TCP.
Replace these values with identified mass properties before hardware validation.
"""

import numpy as np


def _sphere_payload(mass, com, radius):
    inertia = np.eye(3, dtype=float) * (0.4 * mass * radius * radius)
    return {'m': mass, 'c': np.asarray(com, dtype=float), 'I': inertia}


# Mount is ``(hx, hy)``: wrist-to-palm gap and lateral correction. The runner
# mirrors hy for the right arm because its +90 degree mount yaw flips TCP-y.
#
# dg5f: hy=0.0071 cancels the root-origin shift verified on render/hardware.
# dg5s: (0.06, 0.015) was set by eye on the render rather than derived; re-check
# it if the S palm geometry changes.
_GRIPPERS = {
    'h9': {
        'mount': (0.02, 0.0071),
        'payload': _sphere_payload(0.58, [0.06, 0.0, 0.0], 0.03),
    },
    'dg5f': {
        'mount': (0.02, 0.0071),
        'payload': _sphere_payload(1.60, [0.08, 0.0, 0.0], 0.03),
    },
    'dg5s': {
        'mount': (0.06, 0.015),
        'payload': _sphere_payload(0.96, [0.08, 0.0, 0.0], 0.03),
    },
}


def _get_gripper(gripper):
    try:
        return _GRIPPERS[gripper]
    except KeyError:
        raise ValueError(
            f"unknown gripper {gripper!r}; expected one of {sorted(_GRIPPERS)}"
        ) from None


def get_hand_mount(gripper):
    """Return the gripper's ``(hx, hy)`` TCP mount offsets."""
    return _get_gripper(gripper)['mount']


def get_hand_payload(gripper):
    """Return a copy safe to pass directly to ``tact.Model.edit``."""
    payload = _get_gripper(gripper)['payload']
    return {
        'm': payload['m'],
        'c': payload['c'].copy(),
        'I': payload['I'].copy(),
    }


def combine_inertias(first, second):
    """Combine rigid inertias expressed in the same body frame.

    Each ``I`` is about that component's own center of mass. The returned
    inertia is about the combined center of mass.
    """
    m1, m2 = float(first['m']), float(second['m'])
    c1 = np.asarray(first['c'], dtype=float)
    c2 = np.asarray(second['c'], dtype=float)
    I1 = np.asarray(first['I'], dtype=float)
    I2 = np.asarray(second['I'], dtype=float)
    mass = m1 + m2
    if mass <= 0:
        raise ValueError(f'combined mass must be positive, got {mass}')

    com = (m1 * c1 + m2 * c2) / mass

    def shifted(inertia, component_mass, displacement):
        return inertia + component_mass * (
            displacement.dot(displacement) * np.eye(3)
            - np.outer(displacement, displacement)
        )

    inertia = shifted(I1, m1, c1 - com) + shifted(I2, m2, c2 - com)
    return {'m': mass, 'c': com, 'I': inertia}


def attach_hand_payload(model, body, gripper):
    """Add a whole-hand equivalent payload to an existing model body."""
    body_idx = model.fbody[model.fdict[body]]
    bare = {
        'm': model.m[body_idx],
        'c': np.asarray(model.c[body_idx], dtype=float),
        'I': np.asarray(model.I[body_idx], dtype=float),
    }
    model.edit(body, **combine_inertias(bare, get_hand_payload(gripper)))
