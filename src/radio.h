#ifndef RADIO_H
#define RADIO_H

#include <cstddef>

bool radioBegin();
void radioRequest(const char* station);
void radioStop();
bool radioIsActive();
bool radioIsPlaying();
bool radioTakeNotice(char* dst, size_t cap);

#endif // RADIO_H