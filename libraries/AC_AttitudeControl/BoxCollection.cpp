#include "BoxCollection.h"

BoxShim BoxCollection::get(uint8_t id) {
  return _boxes[id-1];
}

void BoxCollection::set(uint8_t id, BoxShim box) {
  _boxes[id-1].set_y_ub(box.y_ub());
  _boxes[id-1].set_y_lb(box.y_lb());
  _boxes[id-1].set_vy_ub(box.vy_ub());
  _boxes[id-1].set_vy_lb(box.vy_lb());
  _boxes[id-1].set_x_ub(box.x_ub());
  _boxes[id-1].set_x_lb(box.x_lb());
  _boxes[id-1].set_vx_ub(box.vx_ub());
  _boxes[id-1].set_vx_lb(box.vx_lb());
  _boxes[id-1].set_z_ub(box.z_ub());
  _boxes[id-1].set_z_lb(box.z_lb());
  _boxes[id-1].set_vz_ub(box.vz_ub());
  _boxes[id-1].set_vz_lb(box.vz_lb());
  _boxes[id-1].set_angle_lb(box.angle_lb());
  _boxes[id-1].set_abraking(box.abraking());
  _boxes[id-1].set_lookahead(box.lookahead());
  _boxes[id-1].set_d_ctrl(box.d_ctrl());
  _boxes[id-1].set_smooth(box.smooth());

  _initialized[id-1] = true;
}

void BoxCollection::clear() {
  for (uint8_t i = 0; i < NUM_BOXES; i++) {
    _initialized[i] = false;
  }
}

bool BoxCollection::has_box(uint8_t id) {
  return 1 <= id &&
    id <= NUM_BOXES &&
	 _initialized[id-1];
}
