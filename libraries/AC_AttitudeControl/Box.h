#include <math.h>
#include <algorithm>

// Acceleration due to gravity in cm/s/s.
// Sensors seem to use cm as length unit.
// This constant has same level of precision
// as python simulation.
static const float g = -980.665;
static const float d = 1;

struct control_in {
  float a;
  float theta;
};

struct state {
  float x;
  float y;
  float vx;
  float vy;
};

struct shim_params {
  float amin;
  float ubx;
  float lbx;
  float uby;
  float lby;
  float ubvx;
  float lbvx;
  float ubvy;
  float lbvy;
  float theta_min;
  float theta_max;
};

class BoxShim {
 public:
  BoxShim(shim_params p)
    : _params(p)
  {};

  void set_params(shim_params p) {_params = p;}

  /*
   * Runs the monitor, returning a safe control input.
   */
  control_in monitor(control_in proposed, state st);

 private:

  /*
   * Runs the safety check in one dimension
   */
  bool safe_acc(float a, float v, float y, 
		float ub, float ubv, float amin);

  /*
   * Returns the default action for the position
   * monitor in polar coordinates.
   *
   * The position monitor default action always passes
   * the velocity monitor's safety check, so we don't
   * have to consider it.
   */
  control_in default_action(state st);

  /*
   * Returns the default action for one rectangular dimension.
   */
  float default_rect_action_one_dim(float y, float v,
				    float amin);

  /*
   * Returns polar coordinates such that A >= 0
   * -PI <= theta <= PI
   */
  control_in rect_to_polar (float x, float y);

  /*
   * The safety check for the velocity monitor.
   */
  bool vel_safe_acc(float a, float v, float ubv);

  /*
   * The safety check for the position monitor.
   */
  bool pos_safe_acc(float a, float v, float y, float ub,
		    float amin);

  /*
   * Breaking acceleration in the X direction
   */
  float amin_X();

  /*
   * Breaking acceleration in the Y direction
   */
  float amin_Y();

  /*
   * Parameters of this shim
   */
  shim_params _params;
};
