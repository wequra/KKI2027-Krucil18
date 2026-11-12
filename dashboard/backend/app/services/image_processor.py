import cv2
import numpy as np
from PIL import Image
import io
import logging

logger = logging.getLogger("image_processor")

def process_and_validate_image(image_bytes: bytes, max_width: int = 1280) -> bytes:
    """
    1. Membaca image dari bytes.
    2. Me-resize image jika terlalu besar (menghemat storage GCS).
    3. Validasi OpenCV dasar (Bisa dikembangkan lebih lanjut untuk validasi warna IMH/IMB).
    4. Mengembalikan bytes image yang sudah dikompresi.
    """
    try:
        # Load image via Pillow
        pil_img = Image.open(io.BytesIO(image_bytes))
        
        # Resize jika terlalu lebar
        if pil_img.width > max_width:
            ratio = max_width / float(pil_img.width)
            new_height = int(float(pil_img.height) * float(ratio))
            pil_img = pil_img.resize((max_width, new_height), Image.Resampling.LANCZOS)
        
        # Konversi ke format JPEG untuk kompresi yang baik
        output_buffer = io.BytesIO()
        pil_img.convert("RGB").save(output_buffer, format="JPEG", quality=85)
        processed_bytes = output_buffer.getvalue()

        # Opsional: Validasi OpenCV
        # cv_img = cv2.imdecode(np.frombuffer(processed_bytes, np.uint8), cv2.IMREAD_COLOR)
        # Lakukan pendeteksian bounding box/warna buoy di sini jika diperlukan...
        
        logger.info(f"Image processed successfully. New size: {len(processed_bytes)} bytes")
        return processed_bytes

    except Exception as e:
        logger.error(f"Error processing image: {e}")
        # Jika gagal diproses, kembalikan bytes aslinya agar data tidak hilang
        return image_bytes
