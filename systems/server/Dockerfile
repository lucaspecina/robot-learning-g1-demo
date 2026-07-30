FROM python:3.12-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl iproute2 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir \
        torch==2.13.0 \
        --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

ENV OPEN_VOCAB_MODEL=IDEA-Research/grounding-dino-tiny
ENV OPEN_VOCAB_MODEL_REVISION=a2bb814dd30d776dcf7e30523b00659f4f141c71

# El modelo queda fijado dentro de la imagen para que una caída de Internet no
# convierta la búsqueda local del servidor en una descarga en plena misión.
RUN python3 -c "from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor; \
m='IDEA-Research/grounding-dino-tiny'; r='a2bb814dd30d776dcf7e30523b00659f4f141c71'; \
AutoProcessor.from_pretrained(m, revision=r); \
AutoModelForZeroShotObjectDetection.from_pretrained(m, revision=r)"

WORKDIR /app
COPY intelligence_service.py /app/intelligence_service.py
COPY open_vocabulary_detector.py /app/open_vocabulary_detector.py
CMD ["python3", "/app/intelligence_service.py"]
