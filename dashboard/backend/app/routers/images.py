# app/routers/images.py
import os
import uuid
from pathlib import Path
from datetime import datetime
from fastapi import APIRouter, Depends, HTTPException, Query, UploadFile, File, Form, status
from fastapi.responses import FileResponse
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, desc
from typing import Optional
from app.core.database import get_db
from app.models.image import MissionImage
from app.schemas.image import ImageResponse, ImageType
from app.core.security import verify_jetson_api_key
from app.services.image_processor import process_and_validate_image

router = APIRouter(prefix="/api/v1/images", tags=["Mission Images"])

# Folder penyimpanan gambar
UPLOAD_DIR = Path(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "uploads", "images"))
UPLOAD_DIR.mkdir(parents=True, exist_ok=True)


@router.post(
    "/",
    response_model=ImageResponse,
    status_code=status.HTTP_201_CREATED,
    dependencies=[Depends(verify_jetson_api_key)]  # Proteksi: hanya Jetson yang boleh upload
)
async def upload_image(
    file: UploadFile = File(..., description="File gambar dari Jetson"),
    image_type: ImageType = Form(..., description="Tipe: SURFACE atau UNDERWATER"),
    captured_at: datetime = Form(..., description="Waktu foto diambil (ISO format)"),
    run_id: Optional[int] = Form(None, description="ID run aktif"),
    grid_position: Optional[str] = Form(None, description="Posisi grid arena (misal: B3)"),
    db: AsyncSession = Depends(get_db)
):
    """
    Upload gambar misi dari Jetson. File disimpan ke filesystem,
    metadata disimpan ke database PostgreSQL.
    """
    # Validasi tipe file
    allowed_types = ["image/jpeg", "image/png", "image/webp"]
    if file.content_type not in allowed_types:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Tipe file tidak didukung: {file.content_type}. Gunakan JPEG, PNG, atau WebP."
        )

    # Buat folder berdasarkan run_id (atau 'unlinked' jika tanpa run)
    run_folder = str(run_id) if run_id else "unlinked"
    save_dir = os.path.join(UPLOAD_DIR, run_folder)
    os.makedirs(save_dir, exist_ok=True)

    # Generate nama file unik: {timestamp}_{uuid}.{ext}
    ext = file.filename.split(".")[-1] if file.filename and "." in file.filename else "jpg"
    unique_name = f"{captured_at.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}.{ext}"
    file_path = os.path.join(save_dir, unique_name)

    # Simpan file ke filesystem
    original_content = await file.read()
    
    # Proses dengan OpenCV / Pillow (Resize, Validate, Convert to JPEG)
    processed_content = process_and_validate_image(original_content)
    file_size = len(processed_content)

    with open(file_path, "wb") as f:
        f.write(processed_content)

    # Simpan metadata ke database
    db_image = MissionImage(
        run_id=run_id,
        image_type=image_type.value,
        file_path=file_path,
        file_size=file_size,
        grid_position=grid_position,
        captured_at=captured_at
    )

    db.add(db_image)
    await db.commit()
    await db.refresh(db_image)

    return db_image


@router.get(
    "/",
    response_model=list[ImageResponse]
)
async def list_images(
    run_id: Optional[int] = Query(None, description="Filter by run ID"),
    image_type: Optional[ImageType] = Query(None, description="Filter: SURFACE atau UNDERWATER"),
    limit: int = Query(50, ge=1, le=200),
    offset: int = Query(0, ge=0),
    db: AsyncSession = Depends(get_db)
):
    """
    Ambil daftar metadata gambar. Bisa difilter berdasarkan run_id dan image_type.
    Terbuka tanpa API Key — untuk dashboard.
    """
    query = select(MissionImage).order_by(desc(MissionImage.captured_at))

    if run_id is not None:
        query = query.where(MissionImage.run_id == run_id)
    if image_type is not None:
        query = query.where(MissionImage.image_type == image_type.value)

    query = query.limit(limit).offset(offset)

    result = await db.execute(query)
    images = result.scalars().all()

    return images


@router.get(
    "/{image_id}",
    response_model=ImageResponse
)
async def get_image(
    image_id: int,
    db: AsyncSession = Depends(get_db)
):
    """
    Ambil metadata satu gambar berdasarkan ID.
    Terbuka tanpa API Key — untuk dashboard.
    """
    image = await _get_image_or_404(db, image_id)
    return image


@router.get(
    "/{image_id}/file",
    response_class=FileResponse
)
async def serve_image_file(
    image_id: int,
    db: AsyncSession = Depends(get_db)
):
    """
    Serve file gambar langsung (untuk preview di dashboard).
    Terbuka tanpa API Key — untuk dashboard.
    """
    image = await _get_image_or_404(db, image_id)

    if not os.path.exists(image.file_path):
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"File gambar tidak ditemukan di filesystem: {image.file_path}"
        )

    return FileResponse(
        path=image.file_path,
        media_type="image/jpeg",
        filename=os.path.basename(image.file_path)
    )


# ─── Helper ───────────────────────────────────────────────────────

async def _get_image_or_404(db: AsyncSession, image_id: int) -> MissionImage:
    """Helper: ambil image atau raise 404."""
    result = await db.execute(
        select(MissionImage).where(MissionImage.id == image_id)
    )
    image = result.scalar_one_or_none()

    if image is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Image dengan ID {image_id} tidak ditemukan."
        )
    return image
