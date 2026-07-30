#!/usr/bin/env python3
"""Inteligencia remota de la demo, fuera de los lazos rápidos del robot.

El servidor recibe artefactos pequeños y decisiones lentas. Nunca publica
velocidades ni controla articulaciones. Si está caído o la red se corta, la
Jetson debe pausar la misión mientras el control local mantiene al G1 estable.
"""
import base64
import binascii
import json
import os
import threading
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openai import OpenAI


PORT = 8000
MAX_REQUEST_BYTES = 2_000_000
MODEL_TIMEOUT_S = 10.0
MODEL_MAX_RETRIES = 1

CLOCK_SCHEMA = {
    "type": "json_schema",
    "json_schema": {
        "name": "clock_reading",
        "strict": True,
        "schema": {
            "type": "object",
            "properties": {
                "readable": {"type": "boolean"},
                "hour": {"type": "integer"},
                "minute": {"type": "integer"},
                "text": {"type": "string"},
            },
            "required": ["readable", "hour", "minute", "text"],
            "additionalProperties": False,
        },
    },
}


class ServiceConfigurationError(RuntimeError):
    pass


class InvalidImageError(ValueError):
    pass


class InvalidModelResponseError(ValueError):
    """Conserva la salida literal cuando el modelo responde algo inválido."""

    def __init__(self, message: str, raw_output: str = None):
        super().__init__(message)
        self.raw_output = raw_output


class ObjectDetectorUnavailableError(RuntimeError):
    pass


def validate_reading(reading: dict) -> dict:
    required = {"readable", "hour", "minute", "text"}
    if set(reading) != required:
        raise InvalidModelResponseError(
            "la respuesta no contiene exactamente los campos esperados"
        )
    if not isinstance(reading["readable"], bool):
        raise InvalidModelResponseError("readable no es booleano")
    if not isinstance(reading["hour"], int) or not 0 <= reading["hour"] <= 23:
        raise InvalidModelResponseError("hour está fuera de rango")
    if (
        not isinstance(reading["minute"], int)
        or not 0 <= reading["minute"] <= 59
    ):
        raise InvalidModelResponseError("minute está fuera de rango")
    if not isinstance(reading["text"], str):
        raise InvalidModelResponseError("text no es una cadena")
    if reading["readable"]:
        expected_text = f"{reading['hour']:02d}:{reading['minute']:02d}"
        if reading["text"] != expected_text:
            raise InvalidModelResponseError(
                "el texto y los campos numéricos no coinciden"
            )
    return reading


def decode_image(image_base64: str) -> bytes:
    if not isinstance(image_base64, str) or not image_base64:
        raise InvalidImageError("falta image_base64")
    try:
        image = base64.b64decode(image_base64, validate=True)
    except (binascii.Error, ValueError) as error:
        raise InvalidImageError("image_base64 no es válido") from error
    if not 100 <= len(image) <= 1_500_000:
        raise InvalidImageError(
            f"la imagen tiene un tamaño inválido: {len(image)} bytes"
        )
    if not image.startswith(b"\xff\xd8") or not image.endswith(b"\xff\xd9"):
        raise InvalidImageError("la imagen no es un JPEG completo")
    return image


class ClockReader:
    def __init__(
        self,
        client=None,
        deployment: str = None,
        base_url: str = None,
        credential: str = None,
    ):
        self.deployment = deployment or os.environ.get(
            "AZURE_VISION_DEPLOYMENT"
        )
        if client is not None:
            self.client = client
            return

        resolved_base_url = base_url or os.environ.get(
            "AZURE_OPENAI_BASE_URL"
        )
        resolved_credential = credential or os.environ.get(
            "AZURE_INFERENCE_CREDENTIAL"
        )
        missing = [
            name
            for name, value in (
                ("AZURE_OPENAI_BASE_URL", resolved_base_url),
                ("AZURE_INFERENCE_CREDENTIAL", resolved_credential),
                ("AZURE_VISION_DEPLOYMENT", self.deployment),
            )
            if not value
        ]
        if missing:
            raise ServiceConfigurationError(
                "faltan variables: " + ", ".join(missing)
            )

        self.client = OpenAI(
            base_url=resolved_base_url,
            api_key=resolved_credential,
            timeout=MODEL_TIMEOUT_S,
            max_retries=MODEL_MAX_RETRIES,
        )

    def read(self, image: bytes) -> dict:
        image_url = (
            "data:image/jpeg;base64,"
            + base64.b64encode(image).decode("ascii")
        )
        response = self.client.chat.completions.create(
            model=self.deployment,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "Leé únicamente el reloj digital visible. "
                        "Si no es legible, readable debe ser false."
                    ),
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "Devolvé la hora mostrada.",
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image_url,
                                "detail": "low",
                            },
                        },
                    ],
                },
            ],
            response_format=CLOCK_SCHEMA,
            max_tokens=100,
        )
        if not response.choices:
            raise InvalidModelResponseError("el modelo no devolvió opciones")
        message = response.choices[0].message.content
        if not message:
            raise InvalidModelResponseError("el modelo devolvió contenido vacío")
        try:
            reading = json.loads(message)
        except json.JSONDecodeError as error:
            raise InvalidModelResponseError(
                "el modelo no devolvió JSON válido",
                raw_output=message,
            ) from error
        try:
            reading = validate_reading(reading)
        except InvalidModelResponseError as error:
            raise InvalidModelResponseError(
                str(error),
                raw_output=message,
            ) from error
        return {
            "reading": reading,
            # El tablero debe mostrar lo que realmente devolvió el modelo, no
            # una reconstrucción posterior a partir de los campos validados.
            "raw_output": message,
            "model": self.deployment,
        }


class IntelligenceService:
    def __init__(self, clock_reader=None, object_detector=None):
        self.configuration_error = None
        self.object_detector = object_detector
        self.object_detector_error = None
        self.object_detector_lock = threading.Lock()
        if clock_reader is not None:
            self.clock_reader = clock_reader
            return
        try:
            self.clock_reader = ClockReader()
        except ServiceConfigurationError as error:
            self.clock_reader = None
            self.configuration_error = str(error)

    @property
    def configured(self) -> bool:
        return self.clock_reader is not None

    @property
    def object_detector_ready(self) -> bool:
        return self.object_detector is not None

    def get_object_detector(self):
        """Carga el modelo sólo cuando una tarea realmente lo necesita."""
        if self.object_detector is not None:
            return self.object_detector
        with self.object_detector_lock:
            if self.object_detector is not None:
                return self.object_detector
            if self.object_detector_error is not None:
                raise ObjectDetectorUnavailableError(
                    self.object_detector_error
                )
            try:
                try:
                    from .open_vocabulary_detector import (
                        OpenVocabularyDetector,
                    )
                except ImportError:
                    from open_vocabulary_detector import (
                        OpenVocabularyDetector,
                    )
                self.object_detector = OpenVocabularyDetector()
            except Exception as error:
                self.object_detector_error = (
                    f"{type(error).__name__}: {error}"
                )
                raise ObjectDetectorUnavailableError(
                    self.object_detector_error
                ) from error
        return self.object_detector

    def read_clock(self, image: bytes) -> dict:
        if self.clock_reader is None:
            raise ServiceConfigurationError(
                self.configuration_error or "servicio no configurado"
            )
        started_at = time.monotonic()
        model_result = self.clock_reader.read(image)
        return {
            "ok": True,
            **model_result,
            "elapsed_s": round(time.monotonic() - started_at, 3),
        }

    def detect_objects(self, image: bytes, labels) -> dict:
        started_at = time.monotonic()
        detector = self.get_object_detector()
        result = detector.detect(image, labels)
        return {
            "ok": True,
            **result,
            "elapsed_s": round(time.monotonic() - started_at, 3),
        }


class Handler(BaseHTTPRequestHandler):
    service = IntelligenceService()

    def send_json(self, status: HTTPStatus, payload: dict):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self.send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    # Se conserva este campo porque los chequeos existentes lo
                    # usan para saber si el lector de reloj tiene credenciales.
                    "configured": self.service.configured,
                    "clock_reader_configured": self.service.configured,
                    "object_detector_ready": (
                        self.service.object_detector_ready
                    ),
                },
            )
            return
        self.send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "ruta"})

    def do_POST(self):
        request_id = self.headers.get("X-Request-ID") or str(uuid.uuid4())
        if self.path not in ("/v1/read-clock", "/v1/detect-objects"):
            self.send_json(
                HTTPStatus.NOT_FOUND,
                {"ok": False, "error": "ruta", "request_id": request_id},
            )
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            content_length = 0
        if not 0 < content_length <= MAX_REQUEST_BYTES:
            self.send_json(
                HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                {
                    "ok": False,
                    "error": "tamaño de pedido inválido",
                    "request_id": request_id,
                },
            )
            return

        try:
            payload = json.loads(self.rfile.read(content_length))
            if not isinstance(payload, dict):
                raise InvalidImageError("el pedido debe ser un objeto JSON")
            image = decode_image(payload.get("image_base64"))
            if self.path == "/v1/read-clock":
                result = self.service.read_clock(image)
            else:
                result = self.service.detect_objects(
                    image,
                    payload.get("labels"),
                )
            result["request_id"] = request_id
            self.send_json(HTTPStatus.OK, result)
        except InvalidModelResponseError as error:
            response = {
                "ok": False,
                "error": str(error),
                "request_id": request_id,
            }
            if error.raw_output is not None:
                response["raw_output"] = error.raw_output
            self.send_json(HTTPStatus.BAD_GATEWAY, response)
        except (
            json.JSONDecodeError,
            InvalidImageError,
            ValueError,
        ) as error:
            self.send_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "ok": False,
                    "error": str(error),
                    "request_id": request_id,
                },
            )
        except ServiceConfigurationError:
            self.send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {
                    "ok": False,
                    "error": "modelo visual no configurado",
                    "request_id": request_id,
                },
            )
        except ObjectDetectorUnavailableError:
            self.send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {
                    "ok": False,
                    "error": "detector visual no disponible",
                    "request_id": request_id,
                },
            )
        except Exception as error:
            # La respuesta no filtra detalles del proveedor ni credenciales.
            print(
                f"[intelligence] pedido {request_id} falló: "
                f"{type(error).__name__}: {error}",
                flush=True,
            )
            self.send_json(
                HTTPStatus.BAD_GATEWAY,
                {
                    "ok": False,
                    "error": "falló el proveedor visual",
                    "request_id": request_id,
                },
            )

    def log_message(self, *args):
        pass


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(
        f"[intelligence] escuchando en 0.0.0.0:{PORT}; "
        f"lector de reloj configurado: {Handler.service.configured}; "
        "detector abierto: carga diferida",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
