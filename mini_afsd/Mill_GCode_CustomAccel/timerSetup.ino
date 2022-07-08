hw_timer_t * timer = NULL;


void setupTimer(unsigned int countTime) {
  timer = timerBegin(0, 80, true);  //Sets prescaler to 1 tick every us
  timerAttachInterrupt(timer, &reportData, true);  // Attach function to timer
  timerAlarmWrite(timer, countTime, true);  // Set alarm to call function at specified interval
}

void startTimer() {
  timerAlarmEnable(timer);
}

void haltTimer() {
  timerEnd(timer);
  timer = NULL;
}

bool timerStarted() {
  return (timer != NULL);
}
