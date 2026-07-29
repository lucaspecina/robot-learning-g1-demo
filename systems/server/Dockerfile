FROM python:3.12-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl iproute2 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /app
COPY intelligence_service.py /app/intelligence_service.py
CMD ["python3", "/app/intelligence_service.py"]
