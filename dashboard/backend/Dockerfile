# Menggunakan base image Python 3.10 slim agar lebih ringan
FROM python:3.10-slim

# Set working directory di dalam container
WORKDIR /app

# Mencegah Python menulis file .pyc ke disk
ENV PYTHONDONTWRITEBYTECODE=1
# Mencegah Python men-buffer stdout dan stderr
ENV PYTHONUNBUFFERED=1

# Install system dependencies (untuk library yang butuh compile seperti psycopg2/asyncpg/pymavlink)
RUN apt-get update && apt-get install -y \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy file requirements terlebih dahulu (untuk memanfaatkan Docker cache)
COPY requirements.txt .

# Install dependencies Python
RUN pip install --no-cache-dir -r requirements.txt

# Copy seluruh source code aplikasi ke dalam container
COPY . .

# Expose port yang digunakan FastAPI
EXPOSE 8000

# Perintah default untuk menyalakan uvicorn
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
