#include "Box.h"

#define NUM_BOXES 4

class BoxCollection {
 public:
  BoxShim get(uint8_t id);

  void set(uint8_t id, BoxShim box);

  bool has_box(uint8_t id);

  void clear();

  uint8_t size() {return NUM_BOXES;}

 private:
  BoxShim _boxes [NUM_BOXES];

  bool _initialized [NUM_BOXES];
};
