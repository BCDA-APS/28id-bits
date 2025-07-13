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


class GammaPositionerComputed(SoftPositioner):
    """
    Compute angle gamma from screw translation.

    Use this class (instead of ``SoftPositioner``) for the gamma axis in the
    diffractometer class.

    PARAMETERS:

    radius *float* :
        Radius of the gamma rotation.  Default: 10_000.0.  Must have same
        engineering units as gamma screw translation.
    screw_name *str* :
        Name of the screw axis component (in the parent diffractometer).
        Default: ``"gamscrew"``.
    """

    def __init__(
        self,
        *,
        radius: float = DEFAULT_RADIUS,
        screw_name: str = "gamscrew",
        **kwargs,
    ):
        """."""
        self.radius = radius  # same units as screw
        super().__init__(**kwargs)

        self.screw = getattr(self.parent, screw_name)
        if self.screw.connected:
            self._recompute_limits()

    def forward(self, screw: float) -> float:
        """Return gamma angle (degrees) from screw translation."""
        return math.atan2(screw, self.radius) * 180 / math.pi

    def inverse(self, gamma: float) -> float:
        """Return screw translation from gamma angle (degrees)."""
        return self.radius * math.tan(gamma * math.pi / 180)

    @property
    def position(self) -> float:
        """The current position of gamma, as computed from screw position."""
        try:
            screw = self.screw.position
        except AttributeError:
            screw = 0  # during initialization
        return self.forward(screw)

    def _set_position(self, value, **kwargs):
        """Set the current internal position, run the readback subscription."""
        try:
            angle = self.inverse(value)
            self.screw._set_position(angle, **kwargs)
        except AttributeError:
            super()._set_position(value, **kwargs)  # during initialization

    def _recompute_limits(self) -> None:
        """Compute gamma limits from screw translation."""
        try:
            self._limits = tuple(sorted(map(self.forward, self.screw.limits)))
        except AttributeError:
            pass  # during initialization


class SixcSim(SixcBase):
    """
    Simulated IUCr 6-circle with gamma driven by screw translation.

    The IUCr 6-circle is the geometry of Lohmeier & Vlieg.
    """

    gamma = Component(
        GammaPositionerComputed,
        screw_name="gamscrew",
        radius=DEFAULT_RADIUS,
        init_pos=0,
        kind=H_OR_N,
        labels=MOTOR_LABELS,
    )

    gamscrew = Component(
        SoftPositioner,
        limits=(-10, 250),
        init_pos=1.30,
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
        if self.gamscrew.connected:
            self.gamma._recompute_limits()
            self.core.constraints["gamma"].limits = self.gamma.limits


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


# developer code example here:
# def main():
#     """Demonstrate Sixc & SixcSim."""
#     sixc = SixcSim(name="sixc")
#     # print(f"{sixc=!r}")
#     sixc.wh()

#     print(sixc.position_table())

#     sixc = Sixc(
#         name="sixc",
#         prefix="zgp:",
#         m_mu="m1",
#         m_theta="m2",
#         m_chi="m3",
#         m_phi="m4",
#         m_gamscrew="m5",
#         m_delta="m6",
#         radius=1500.0,
#     )
#     sixc.wait_for_connection()
#     sixc.gamma._recompute_limits()
#     sixc.core.constraints["gamma"].limits = sixc.gamma.limits

#     # print(f"{sixc=!r}")
#     sixc.wh()
#     # sixc.wh(full=True)
#     # print(sixc.configuration)
#     # print(f"{sixc.component_names=}")
#     set_diffractometer(sixc)
#     sixc.core.mode = "lifting detector tau"
#     sixc.core.mode = "4-circles bissecting horizonta"
#     print(f"{sixc.core.modes=}")
#     print(f"{cahkl(0.01, .1, 0.05)=}")
#     sixc.core.mode = "psi constant vertical"
#     print(sixc.position_table())


# if __name__ == "__main__":
#     main()
