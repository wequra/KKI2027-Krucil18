"""
Script untuk recreate semua tabel di database.
PERINGATAN: Ini akan MENGHAPUS semua data yang ada!
Hanya gunakan untuk development/testing.
"""
import asyncio
from app.core.database import engine, Base

# Import semua models agar Base.metadata tahu semua tabel
from app.models import event, run, image  # noqa: F401


async def recreate_tables():
    print("Dropping all tables...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
    print("Creating all tables...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    print("Done! Tabel yang dibuat:")
    for table_name in Base.metadata.tables:
        print(f"  - {table_name}")
    await engine.dispose()


if __name__ == "__main__":
    asyncio.run(recreate_tables())
