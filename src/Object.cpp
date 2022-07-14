#include "ByteTrack/Object.h"

byte_track::Object::Object(const Rect<float> &_rect, int _label, float _prob)
    : rect(_rect), label(_label), prob(_prob) {}
