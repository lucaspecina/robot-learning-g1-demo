#!/usr/bin/env python3
"""Cliente de la Jetson para decisiones lentas del servidor externo."""
import base64
import json
import os
import time
import urllib.error
import urllib.request
import uuid


DEFAULT_SERVER_URL = "http://172.30.0.20:8000"
DEFAULT_TIMEOUT_S = 15.0
FAILURE_THRESHOLD = 3
OPEN_INTERVAL_S = 30.0


class RemoteIntelligenceError(RuntimeError):
    pass


class CircuitOpenError(RemoteIntelligenceError):
    pass


def validate_reading(reading: dict) -> dict:
    required = {"readable", "hour", "minute", "text"}
    if not isinstance(reading, dict) or set(reading) != required:
        raise RemoteIntelligenceError(
            "la lectura no tiene el formato esperado"
        )
    if not isinstance(reading["readable"], bool):
        raise RemoteIntelligenceError("readable no es booleano")
    if not isinstance(reading["hour"], int) or not 0 <= reading["hour"] <= 23:
        raise RemoteIntelligenceError("hour está fuera de rango")
    if (
        not isinstance(reading["minute"], int)
        or not 0 <= reading["minute"] <= 59
    ):
        raise RemoteIntelligenceError("minute está fuera de rango")
    if not isinstance(reading["text"], str):
        raise RemoteIntelligenceError("text no es una cadena")
    if reading["readable"]:
        expected_text = f"{reading['hour']:02d}:{reading['minute']:02d}"
        if reading["text"] != expected_text:
            raise RemoteIntelligenceError(
                "el texto y los campos numéricos no coinciden"
            )
    return reading


class IntelligenceClient:
    def __init__(
        self,
        server_url: str = None,
        timeout_s: float = None,
        opener=None,
        monotonic=None,
    ):
        self.server_url = (
            server_url
            or os.environ.get("INTELLIGENCE_SERVER_URL")
            or DEFAULT_SERVER_URL
        ).rstrip("/")
        self.timeout_s = timeout_s or float(
            os.environ.get(
                "INTELLIGENCE_TIMEOUT_S",
                DEFAULT_TIMEOUT_S,
            )
        )
        self.opener = opener or urllib.request.urlopen
        self.monotonic = monotonic or time.monotonic
        self.consecutive_failures = 0
        self.opened_at = None

    def _allow_request(self):
        if self.opened_at is None:
            return
        elapsed = self.monotonic() - self.opened_at
        if elapsed < OPEN_INTERVAL_S:
            raise CircuitOpenError(
                "el servidor sigue temporalmente bloqueado después de fallas"
            )
        # Se permite un pedido de prueba. Si falla, el circuito se abre de
        # nuevo; si funciona, el contador se limpia.
        self.opened_at = None

    def _record_success(self):
        self.consecutive_failures = 0
        self.opened_at = None

    def _record_failure(self):
        self.consecutive_failures += 1
        if self.consecutive_failures >= FAILURE_THRESHOLD:
            self.opened_at = self.monotonic()

    def read_clock(self, image: bytes) -> dict:
        self._allow_request()
        request_id = str(uuid.uuid4())
        body = json.dumps(
            {
                "image_base64": base64.b64encode(image).decode("ascii"),
            }
        ).encode("utf-8")
        request = urllib.request.Request(
            f"{self.server_url}/v1/read-clock",
            data=body,
            headers={
                "Content-Type": "application/json",
                "X-Request-ID": request_id,
            },
            method="POST",
        )

        try:
            with self.opener(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read())
        except (
            OSError,
            TimeoutError,
            urllib.error.HTTPError,
            urllib.error.URLError,
            json.JSONDecodeError,
        ) as error:
            self._record_failure()
            raise RemoteIntelligenceError(
                f"falló la lectura remota: {type(error).__name__}"
            ) from error

        if (
            not payload.get("ok")
            or payload.get("request_id") != request_id
            or not isinstance(payload.get("reading"), dict)
        ):
            self._record_failure()
            raise RemoteIntelligenceError(
                "el servidor devolvió una respuesta inválida"
            )

        try:
            reading = validate_reading(payload["reading"])
        except RemoteIntelligenceError:
            self._record_failure()
            raise
        self._record_success()
        return {
            **reading,
            "elapsed_s": payload.get("elapsed_s"),
            "request_id": request_id,
        }
