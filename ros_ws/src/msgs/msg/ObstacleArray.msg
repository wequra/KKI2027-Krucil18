std_msgs/Header header

uint8 UNKNOWN=0
uint8 RED_BUOY=1
uint8 GREEN_BUOY=2
uint8 BLUE_DOCKING_BUOY=3
uint8 GREEN_SURFACE_MARKER=4
uint8 BLUE_UNDERWATER_MARKER=5

# Untuk detector kamera: x=u dan y=v adalah koordinat piksel; z=0.
# header.frame_id tetap frame kamera dari pesan input. Range/metre belum tersedia
# sampai camera calibration + estimasi jarak diimplementasikan.
geometry_msgs/Point[] positions
float32[] radii
uint8[] classes
float32[] confidences
