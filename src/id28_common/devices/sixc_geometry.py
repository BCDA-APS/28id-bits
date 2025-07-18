"""
Variation of IUCr 6-circle with tangent gamma screw drive.

The IUCr 6-circle is the geometry of Lohmeier & Vlieg.

Example diffractometer class that cannot be built using ``hklpy2.creator()``.
Here, the ``gamma`` axis is computed from an additional EPICS motor ``gamscrew``
axis.  The ``creator()`` has no means to specify custom classes for any of the
motor axes.  At present, it only has configuration for no PV (SoftPositioner) or
PV (EpicsMotor).

SPEC axes: delta theta chi phi mu gamma gamscrew

devices YAML:

    id28_common.devices.sixc_geometry.Sixc:
    - name: sixcidc
      prefix: "28idc:"
      radius: 500.0  # TODO: use actual value
      m_delta: m1
      m_theta: m2
      m_chi: m3
      m_phi: m4
      m_mu: m5
      m_gamscrew: m6

    id28_common.devices.sixc_geometry.SixcSim:
    - name: sixcsim
      radius: 500.0  # TODO: use actual value
"""

import math

import hklpy2
from hklpy2.misc import VirtualPositionerBase
from ophyd import Component
from ophyd import EpicsMotor
from ophyd import FormattedComponent
from ophyd import Kind
from ophyd import SoftPositioner

DEFAULT_RADIUS = 10_000.0
H_OR_N = Kind.hinted | Kind.normal
MOTOR_LABELS = ["motors"]
GEOMETRY = dict(
    solver="hkl_soleil",
    geometry="APS POLAR",  # enhanced version of "PETRA3 P09 EH2"
    # "APS POLAR" adds two modes to "PETRA3 P09 EH2":
    #  "psi constant horizontal" & "psi constant vertical"
    reals="mu theta chi phi gamma delta".split(),
)
SixcBase = hklpy2.diffractometer_class_factory(**GEOMETRY)


class VirtualGamma(VirtualPositionerBase):
    """
    Compute (virtual) angle 'gamma' from screw translation.

    PARAMETERS:

    radius *float* :
        Radius of the gamma rotation.  Default: 10_000.0.  Must have same
        engineering units as gamma screw translation.
    physical_name *str* :
        Name of the screw axis component (in the parent diffractometer).
    """

    def __init__(
        self,
        *,
        radius: float = DEFAULT_RADIUS,
        **kwargs,
    ):
        """."""
        self.radius = radius  # same units as screw
        super().__init__(**kwargs)

    def forward(self, screw: float) -> float:
        """Return gamma angle (degrees) from screw translation."""
        return math.atan2(screw, self.radius) * 180 / math.pi

    def inverse(self, gamma: float) -> float:
        """Return screw translation from gamma angle (degrees)."""
        return self.radius * math.tan(gamma * math.pi / 180)


class SixcSim(SixcBase):
    """
    Simulated IUCr 6-circle with gamma driven by screw translation.

    The IUCr 6-circle is the geometry of Lohmeier & Vlieg.
    """

    gamma = Component(
        VirtualGamma,
        physical_name="gamscrew",
        radius=DEFAULT_RADIUS,
        init_pos=0,  # Updates from gamscrew when connected.
        kind=H_OR_N,
        labels=MOTOR_LABELS,
    )

    gamscrew = Component(
        SoftPositioner,
        limits=(-10, 250),
        init_pos=0,
        kind=H_OR_N,
        labels=MOTOR_LABELS,
    )

    def __init__(
        self,
        *args,
        radius: float = DEFAULT_RADIUS,
        **kwargs,
    ):
        """."""
        super().__init__(*args, **kwargs)
        self.gamma.radius = radius


class Sixc(SixcSim):
    """IUCr 6-circle, gamma driven by screw translation (EPICS motors)."""

    # re-define these as EPICS motors
    mu = FormattedComponent(
        EpicsMotor, "{prefix}{m_mu}", kind=H_OR_N, labels=MOTOR_LABELS
    )
    theta = FormattedComponent(
        EpicsMotor, "{prefix}{m_theta}", kind=H_OR_N, labels=MOTOR_LABELS
    )
    chi = FormattedComponent(
        EpicsMotor, "{prefix}{m_chi}", kind=H_OR_N, labels=MOTOR_LABELS
    )
    phi = FormattedComponent(
        EpicsMotor, "{prefix}{m_phi}", kind=H_OR_N, labels=MOTOR_LABELS
    )
    delta = FormattedComponent(
        EpicsMotor, "{prefix}{m_delta}", kind=H_OR_N, labels=MOTOR_LABELS
    )
    # gamma defined in SixcSim
    gamscrew = FormattedComponent(
        EpicsMotor, "{prefix}{m_gamscrew}", kind=H_OR_N, labels=MOTOR_LABELS
    )

    def __init__(
        self,
        *args,
        m_mu: str = "",
        m_theta: str = "",
        m_chi: str = "",
        m_phi: str = "",
        m_gamscrew: str = "",
        m_delta: str = "",
        **kwargs,
    ):
        """."""
        self.m_mu = m_mu
        self.m_theta = m_theta
        self.m_chi = m_chi
        self.m_phi = m_phi
        self.m_gamscrew = m_gamscrew
        self.m_delta = m_delta
        super().__init__(*args, **kwargs)
