#!/usr/bin/env python3
"""Núcleo independiente de ROS para arbitrar la movilidad del robot."""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple


Velocity = Tuple[float, float, float]
ZERO_VELOCITY: Velocity = (0.0, 0.0, 0.0)


class MobilitySource(str, Enum):
    """Fuentes que pueden pedir movimiento de la base."""

    STAND = "stand"
    NAVIGATION = "navigation"
    ALIGNMENT = "alignment"
    MANUAL = "manual"
    TEST = "test"


@dataclass(frozen=True)
class AuthorityResult:
    """Resultado de adquirir o liberar la autoridad."""

    accepted: bool
    reason: str


@dataclass
class SourceCommand:
    """Último comando recibido de una fuente y cuándo llegó."""

    velocity: Velocity = ZERO_VELOCITY
    received_at: float = float("-inf")


class MobilityAuthority:
    """Mantiene un único dueño y descarta comandos de las demás fuentes.

    El tiempo llega desde afuera para poder usar un reloj monotónico tanto en
    las pruebas como en el nodo ROS. La seguridad de red no debe congelarse si
    se pausa el reloj simulado.
    """

    def __init__(
        self,
        lease_timeout_s: float = 0.75,
        command_timeout_s: float = 0.35,
    ):
        self.lease_timeout_s = lease_timeout_s
        self.command_timeout_s = command_timeout_s
        self.owner = MobilitySource.STAND
        self.requester = "stand_hold"
        self.lease_renewed_at = float("inf")
        self.transition_reason = "arranque seguro"
        self.transition_count = 0
        self.rejected_commands = 0
        self.commands: Dict[MobilitySource, SourceCommand] = {
            source: SourceCommand() for source in MobilitySource
        }

    def acquire(
        self,
        source: MobilitySource,
        requester: str,
        now: float,
    ) -> AuthorityResult:
        """Concede control si el recurso está libre o la fuente puede interrumpir."""
        if source == MobilitySource.STAND:
            return AuthorityResult(False, "stand es el estado seguro automático")
        if not requester:
            return AuthorityResult(False, "falta identificar al solicitante")

        if self.owner == source and self.requester == requester:
            self.lease_renewed_at = now
            return AuthorityResult(True, "concesión renovada")

        # El operador puede detener una autonomía activa. Las demás fuentes
        # deben esperar a que el dueño entregue el recurso explícitamente.
        can_preempt = source == MobilitySource.MANUAL
        if self.owner != MobilitySource.STAND and not can_preempt:
            return AuthorityResult(
                False,
                f"movilidad ocupada por {self.owner.value}:{self.requester}",
            )

        previous = f"{self.owner.value}:{self.requester}"
        self.owner = source
        self.requester = requester
        self.lease_renewed_at = now
        self.transition_reason = f"adquirido por {source.value}:{requester} desde {previous}"
        self.transition_count += 1
        return AuthorityResult(True, self.transition_reason)

    def release(
        self,
        source: MobilitySource,
        requester: str,
        reason: str = "liberación solicitada",
    ) -> AuthorityResult:
        """Libera sólo si coincide exactamente con el dueño vigente."""
        if self.owner != source or self.requester != requester:
            return AuthorityResult(
                False,
                f"no es el dueño vigente ({self.owner.value}:{self.requester})",
            )
        self._return_to_stand(reason)
        return AuthorityResult(True, self.transition_reason)

    def submit_command(
        self,
        source: MobilitySource,
        velocity: Velocity,
        now: float,
    ) -> bool:
        """Guarda un comando sólo si proviene de la fuente autorizada."""
        if source != self.owner:
            self.rejected_commands += 1
            return False

        self.commands[source] = SourceCommand(velocity=velocity, received_at=now)
        if source != MobilitySource.STAND:
            self.lease_renewed_at = now
        return True

    def tick(self, now: float) -> Velocity:
        """Vence concesiones y devuelve el único comando seleccionable."""
        if self.owner != MobilitySource.STAND:
            if now - self.lease_renewed_at > self.lease_timeout_s:
                expired = f"{self.owner.value}:{self.requester}"
                self._return_to_stand(f"concesión vencida de {expired}")

        command = self.commands[self.owner]
        if now - command.received_at > self.command_timeout_s:
            return ZERO_VELOCITY
        return command.velocity

    def status(self, now: float) -> dict:
        """Estado serializable para tablero, logs y pruebas."""
        command = self.commands[self.owner]
        command_age = None
        if command.received_at != float("-inf"):
            command_age = max(0.0, now - command.received_at)

        lease_age: Optional[float] = None
        if self.owner != MobilitySource.STAND:
            lease_age = max(0.0, now - self.lease_renewed_at)

        return {
            "owner": self.owner.value,
            "requester": self.requester,
            "command_age_s": command_age,
            "lease_age_s": lease_age,
            "transition_reason": self.transition_reason,
            "transition_count": self.transition_count,
            "rejected_commands": self.rejected_commands,
        }

    def _return_to_stand(self, reason: str):
        self.owner = MobilitySource.STAND
        self.requester = "stand_hold"
        self.lease_renewed_at = float("inf")
        self.transition_reason = reason
        self.transition_count += 1
