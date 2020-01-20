/*
  declination.cpp - Magnetic declination with lat/lng resolution 5 degrees and 1 byte range according to World Magnetic Model 2015-2020
*/

float magDeclination(Location g) {
  //get 4 closest integer points on the declination mesh: [i, j], [i, j1], [i1, j], [i1, j1] around the given point [id, jd]
  float id = (g.lat + 85) / 5;
  if (id < 0) id = 0;
  if (id > NUM_LAT-1) id = NUM_LAT-1;
  float jd = (g.lng + 180) / 5;
  if (jd < 0) jd = 0;
  if (jd > NUM_LNG-1) jd = NUM_LNG-1;
  int i = floor(id);
  if (i < 0) i = 0;
  if (i > NUM_LAT-1) i = NUM_LAT-1;
  int j = floor(jd);
  if (j < 0) j = 0;
  if (j > NUM_LNG-1) j = NUM_LNG-1;

  //declination is calculated as linear approximation of the 4 closest points
  int i1 = i+1;
  int j1 = j+1;
  int8_t dec00 = (int8_t)pgm_read_byte_near(declination + i*NUM_LNG + j);
  int8_t dec01 = dec00;
  int8_t dec10 = dec00;
  int8_t dec11 = dec00;
  
  if (i1 < NUM_LAT)
    dec10 = (int8_t)pgm_read_byte_near(declination + i1*NUM_LNG + j);
  if (j1 < NUM_LNG)
    dec01 = (int8_t)pgm_read_byte_near(declination + i*NUM_LNG + j1);
  if (i1 < NUM_LAT && j1 < NUM_LNG)
    dec11 = (int8_t)pgm_read_byte_near(declination + i1*NUM_LNG + j1);

  float dec = dec00*(i1-id)*(j1-jd) + dec01*(i1-id)*(jd-j) + dec10*(id-i)*(j1-jd) + dec11*(id-i)*(jd-j);

  return dec;
}

