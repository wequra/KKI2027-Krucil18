import logging
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from app.models.run import MissionRun

logger = logging.getLogger("scoring_engine")

# Definisi Skor per Event Type (Berdasarkan rule default)
# Bisa disesuaikan dengan rulebook KKI 2026.
EVENT_SCORES = {
    "SYSTEM_INIT": 0,
    "START": 0,
    "BUOY_PAIR": 10,  # Berhasil melewati gerbang buoy hijau-merah
    "PENALTY": -5,    # Menabrak atau keluar lintasan
    "SURFACE_IMAGING": 15,
    "UNDERWATER_IMAGING": 20,
    "DOCKING": 25,
    "APPROACHING": 0,
    "FINISH": 50,
    "ABORT": -10
}

def calculate_event_score(event_type: str) -> float:
    """Mengembalikan nilai skor untuk jenis event tertentu."""
    return EVENT_SCORES.get(event_type.upper(), 0)

async def update_run_score(db: AsyncSession, run_id: int, score_delta: float):
    """
    Menambahkan (atau mengurangi) skor pada total_score MissionRun.
    """
    if score_delta == 0:
        return

    result = await db.execute(select(MissionRun).filter(MissionRun.id == run_id))
    run = result.scalar_one_or_none()

    if run:
        current_score = run.total_score if run.total_score is not None else 0.0
        run.total_score = current_score + score_delta
        db.add(run)
        await db.commit()
        await db.refresh(run)
        logger.info(f"Updated score for run_id {run_id}. Delta: {score_delta}. New Total: {run.total_score}")
    else:
        logger.warning(f"Could not update score: MissionRun with id {run_id} not found.")
