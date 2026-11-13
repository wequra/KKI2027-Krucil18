# Atomic detection/identity snapshot. Existing ObstacleArray is unchanged.
ObstacleArray obstacles
# New UUID on detector startup/stream reset; IDs are only unique within it.
string tracker_session
# Aligned with obstacles.positions; -1 means untracked.
int64[] track_ids
uint32 image_width
uint32 image_height
