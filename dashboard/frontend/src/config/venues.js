export const VENUES = {
  ITB_JATINANGOR: {
    id: 'ITB_JATINANGOR',
    name: 'ITB Jatinangor Lake',
    image: '/maps/ITB Jatinangor.jpg',
    // [[South Lat, West Lng], [North Lat, East Lng]]
    bounds: [
      [-6.933878, 107.767236], // South-West (J-bottomright Lat, J-topleft Lng)
      [-6.932025, 107.770453], // North-East (J-topleft Lat, J-bottomright Lng)
    ],
    defaultCenter: [-6.932952, 107.768845],
  },
  ITB_GANESA: {
    id: 'ITB_GANESA',
    name: 'ITB Ganesa Campus',
    image: '/maps/ITB Ganesa.jpg',
    // [[South Lat, West Lng], [North Lat, East Lng]]
    bounds: [
      [-6.898556, 107.595375], // South-West (Bawah Kanan Lat, Atas Kiri Lng)
      [-6.884594, 107.620456], // North-East (Atas Kiri Lat, Bawah Kanan Lng)
    ],
    defaultCenter: [-6.891575, 107.607915],
  },
};

export const DEFAULT_VENUE_KEY = 'ITB_JATINANGOR';
