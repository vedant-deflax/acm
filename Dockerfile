FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# System dependencies
RUN apt-get update && apt-get install -y \
    python3.11 \
    python3.11-dev \
    python3-pip \
    curl \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3.11 /usr/bin/python3 && \
    ln -sf /usr/bin/python3 /usr/bin/python

WORKDIR /app

# Install backend dependencies
COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

# Install and build frontend
COPY frontend/package*.json ./frontend/
WORKDIR /app/frontend
RUN npm install
COPY frontend/ .
RUN npm run build

# Back to app root
WORKDIR /app
COPY backend/ ./backend/
COPY data/ ./data/

EXPOSE 8000

CMD ["python3", "-m", "uvicorn", "backend.main:app", "--host", "0.0.0.0", "--port", "8000"]
