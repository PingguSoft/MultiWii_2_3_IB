#ifndef IMU_H_
#define IMU_H_

#define BARO_TAB_SIZE   21

#if defined(BARO) || defined(SONAR)
uint8_t getEstimatedAltitude();
#endif

void computeIMU();

#endif /* IMU_H_ */
